#include "ch.h"
#include "hal.h"
#include "bcm2835.h"
#include <string.h>
#include <math.h>

#define MAX_AXIS 8
#define BUFFER_SIZE 32


typedef struct {
    uint32_t direction_bits;
    int32_t steps[MAX_AXIS];
    int32_t total_steps;
    int32_t step_position;
    

    float vmax_sq;     //[(mm/timestep)^2]
    float vmax_end_sq; //[(mm/timestep)^2]
} stepper_block_t;


/* 
 Circulating buffer for keeping the stepper blocks to be executed
 
 buffer[buffer_tail] is the current block being executed
 buffer[buffer_head]
 
 Data-sharing:
 
 Take into account that velocity planner (INT) interrupt can pre-empt main code
 and that stepper (FIQ) interrupt can pre-empt both main code and velocity planner
 code.
 
 The main (non-interrupt) code can modify buffer[buffer_head] and can increase
 buffer_head and buffer_next_head
 
 The stepper (FIQ) code can read buffer[buffer_tail] and can increase buffer_tail
 
 The stepper code will never pop the last element; thus there will always be at
 least one element present.
 
 
                empty                                  full
                _______                              _______
               |       |                            |       |
 head      --> |       | <-- tail     head      --> |       |
               |   |   |                            |   |   | <--tail
               |  \ /  |                            |  \ /  | 
               |_______|                            |_______|
               
               
                one item
                _______
               |       |
               |       | <-- tail    
 head      --> |   |   |
               |  \ /  |
               |_______|
               

 

 */
static stepper_block_t buffer[BUFFER_SIZE];
static volatile unsigned int buffer_head; // head == tail --> empty
static volatile unsigned int buffer_tail;

static inline unsigned int buffer_next_index(unsigned int index) {
    index++;
    if (index == BUFFER_SIZE) index = 0;
    return index;
}

static inline unsigned int buffer_prev_index(unsigned int index) {
    if (index == 0) index = BUFFER_SIZE;
    return index-1;
}


static inline int buffer_empty(void) {
    return buffer_head == buffer_tail;
}


/*
    Copy a new block into the buffer
    
    Returns 0 on success, -1 on error (buffer full)
    This is the only place where we modify buffer_head 
 */
int stepper_put_buffer(stepper_block_t *block) {
    unsigned int next_head = buffer_next_index(buffer_head);

    if (next_head == buffer_tail) return -1; //buffer was full
    
    // we have free write access to buffer[buffer_head]
    memcpy(&buffer[buffer_head], block, sizeof(stepper_block_t));
    buffer[buffer_head].step_position = 0;
    buffer_head = next_head;
    return 0;
}



typedef struct {
    stepper_block_t *current_block;
    int32_t accu[MAX_AXIS];
} stepper_status_t;

stepper_status_t st;

typedef struct {
    int naxis;
    uint32_t step_mask[MAX_AXIS];
    uint32_t dir_mask[MAX_AXIS];
} stepper_config_t;

stepper_config_t stepper_config = {
    naxis: 2,
    step_mask: {BIT(22), BIT(24), 0,0,0,0,0,0},
    dir_mask: {BIT(23), BIT(25), 0,0,0,0,0,0},
};


static uint32_t all_step_mask; // initialized by stepper_init
static uint32_t all_dir_mask; // initialized by stepper_init

static const float dt = 0.001f;

static const float vmax = 80.0f * 0.001f;              // mm/timestep
static const float steps_per_mm = (32*400.0f/40.0f);
static const float mm_per_step = (40.0f/(32*400.0f));
static const float amax = 100.0f  * 0.001f * 0.001f;         // mm/(timestep^2)
float current_speed;
unsigned int test_ctr = 0;


void stepper_init(void) {
    /* Initialize buffer */
    buffer_head = 0;
    buffer_tail = 0;
    
    /* Place one dummy block (zero steps) into buffer */
    stepper_block_t block;
    memset(&block, 0, sizeof(stepper_block_t));
    stepper_put_buffer(&block);
    st.current_block = &buffer[0];
    
    /* testing code */
    for(int i=0;i<4;i++) {
        block.direction_bits = 0;
        block.steps[0] = 20000;
        block.steps[1] = 0;
        block.total_steps = 20000;
        block.vmax_sq = vmax*vmax;
        block.vmax_end_sq = vmax*vmax;
        stepper_put_buffer(&block);
        
        block.direction_bits = 0;
        block.steps[0] = 200;
        block.steps[1] = 0;
        block.total_steps = 200;
        block.vmax_sq = vmax*vmax;
        block.vmax_end_sq = vmax*vmax/16;
        stepper_put_buffer(&block);
        
        block.direction_bits = 0;
        block.steps[0] = 10000;
        block.steps[1] = 0;
        block.total_steps = 10000;
        block.vmax_sq = vmax*vmax/16;
        block.vmax_end_sq = 0;
        stepper_put_buffer(&block);
        
        block.direction_bits = 0;
        block.steps[0] = 0;
        block.steps[1] = 20000;
        block.total_steps = 20000;
        block.vmax_sq = vmax*vmax;
        block.vmax_end_sq = 0;
        stepper_put_buffer(&block);
    
        block.direction_bits = stepper_config.dir_mask[0] | stepper_config.dir_mask[1];
        block.steps[0] = 20000;
        block.steps[1] = 20000;
        block.total_steps = 20000;
        block.vmax_sq = vmax*vmax;
        block.vmax_end_sq = 0;
        stepper_put_buffer(&block);
    }
    

    /* Compute all_step_mask and all_dir_mask, they are the binary OR of all
       step_masks and dir_masks, respectively 
    */
    all_step_mask=0;
    all_dir_mask=0;
    for (int i=0; i<stepper_config.naxis; i++) {
        all_step_mask |= stepper_config.step_mask[i];
        all_dir_mask |= stepper_config.dir_mask[i];
    }
    
    
    GPCLR0 = all_step_mask | all_dir_mask;
    
    palSetGroupMode(&IOPORT0, all_step_mask | all_dir_mask, 0, PAL_MODE_OUTPUT);
    
    ARM_TIMER_CTL = 0x003E0000;
    ARM_TIMER_LOD = 1000-1;
    ARM_TIMER_RLD = 1000-1;
    ARM_TIMER_DIV = 0x000000F9;
    ARM_TIMER_CLI = 0;
    ARM_TIMER_CTL = 0x003E00A2;
    
    IRQ_FIQ_CONTROL = 0x80|64; //ARM timer interrupt = FIQ interrupt

}


void app_init(void) { stepper_init(); }

static inline void dmb(void) {
    __asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 5" : : "r" (0) : "memory");
}


static inline float limit(float x, float y) { return (y<x) ? y : x; }

static inline float sq(float x) { return x*x; }

void app_systick(void) {
    // one-pass online velocity planner
    unsigned int my_buffer_tail;
    stepper_block_t *bl;
    float new_speed_sq;
    
    my_buffer_tail = buffer_tail; //atomic copy
    /* FIQ interrupt could update buffer_tail, but data will remain valid
       since only FIQ code can write to buffer. We're already called from interrupt,
       so no INT interrupt will occur. Thus, stepper_put_buffer cannot be called
       while we are processing, so data stays as-is. Only buffer[i]->step_position
       can be changed by FIQ while we are processing
     */

    int cumulative_steps = 0;
    float speed_limit_sq;
    
    // default: maximum acceleration
    new_speed_sq = sq(current_speed + amax);

    bl = &buffer[my_buffer_tail];
    cumulative_steps = bl->total_steps - bl->step_position;
    
    // do not accelerate beyond this block's max velocity
    new_speed_sq = limit(new_speed_sq, bl->vmax_sq);
    
    // compute max current speed in order to be able to decellerate
    // to bl_vmax_end_sq before reaching end of this block
    speed_limit_sq = 2*amax*cumulative_steps*mm_per_step + bl->vmax_end_sq;
    
    new_speed_sq = limit(new_speed_sq, speed_limit_sq);
    
    unsigned int i;
    i = buffer_next_index(my_buffer_tail);
    
    while (i!=buffer_head) {
        bl = &buffer[i];

        // compute max current speed in order to be able to decellerate to
        // bl_vmax_end_sq before reaching end of block bl
        cumulative_steps += bl->total_steps;
    
        speed_limit_sq = 2*amax*cumulative_steps*mm_per_step + bl->vmax_end_sq;
        new_speed_sq = limit(new_speed_sq, speed_limit_sq);
        
        i=buffer_next_index(i);
    }
    
    uint32_t reload_value;
    current_speed = sqrtf(new_speed_sq); // mm per dt

    if (current_speed < 0.001) { // todo determine appropriate value
        reload_value = 10000;
    } else {
        reload_value = (1000*mm_per_step) / current_speed;
        if (reload_value>10000) reload_value = 10000; //minimum 100 steps/s
    }
    //mini_uart_sendhex((int)(current_speed*1000000), 1);
    ARM_TIMER_RLD = reload_value;
    
    test_ctr++;
}


__attribute__((naked)) 
void FiqHandler(void) {
    asm volatile ("stmfd    sp!, {r0-r7, r12, lr}" : : : "memory");
    dmb();
    
    uint32_t step_mask;
    stepper_block_t *bl;
    
    // All step signals low
    GPCLR0 = all_step_mask;
    step_mask = BIT(22)| BIT(24);
    
    bl =  st.current_block;


    
    if (bl->step_position == bl->total_steps) {
        unsigned int next_tail = buffer_next_index(buffer_tail);
        if (next_tail == buffer_head) {
            // would pop the last item, we do not do that
        } else {
            // no need to worry about consistency; FIQ cannot be interrupted
            buffer_tail = next_tail;
            bl = st.current_block = &buffer[buffer_tail];
            int accu_init = bl->total_steps >> 1;
            for (int i = 0; i< stepper_config.naxis; i++) {
                st.accu[i]=-accu_init;
            }
        }
    }
    
    
    // Set direction signals
    GPCLR0 = all_dir_mask & ~bl->direction_bits;    
    GPSET0 = bl->direction_bits;

    // Determine which axes to step
    step_mask = 0;    
    if (bl->step_position == bl->total_steps) { 
        // block completed; apparently there was no new block to load.
        // do nothing
    } else {
        // Bresenham algorithm: determine which axes to step
        for(int i=0; i<stepper_config.naxis; i++) {
            st.accu[i] += bl->steps[i];
            if (st.accu[i] > 0) {
                st.accu[i] -= bl->total_steps;
                step_mask |= stepper_config.step_mask[i];
            }
        }
        bl->step_position = bl->step_position + 1;
    }
    dmb();
    // Wait until pulse length has elapsed
    uint32_t wait_val = ARM_TIMER_RLD - 2; //2 us pulse length
    while(ARM_TIMER_VAL > wait_val); 

    ARM_TIMER_CLI = 0;
    dmb();
    GPSET0 = step_mask; // generate step pulse
    dmb();
    asm volatile ("ldmfd    sp!, {r0-r7, r12, lr}" : : : "memory");    
    asm volatile ("subs    pc, lr, #4" : : : "memory");    

}