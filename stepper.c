/*
 This file implements the core motion algorithms. It consists mainly
 of two parts: the step-generator and the velocity controller.
 
 The step-generator and the velocity controller share a 'block-buffer'
 which contains blocks (straight-line sections) to be executed. The
 buffer is implemented as a ring-buffer. The 'tail' (the oldest item
 in the buffer) is the one that is currently being executed by the
 step-generator. Each block prescribes the number of steps for each
 stepper motor, as well as the maximum speed during the block and at
 the end of the block.
 
 The step-generator is driven by the ARM timer interrupt, and runs in
 the ARM fast interrupt request(FIQ) interrupt handler. The FIQ
 interrupt gives it the highest priority in the system (FIQ can
 interrupt 'normal' INT interrupts, and FIQ should never be disabled
 while INT could sometimes be disabled at critical sections). The
 step-generator mainly does one thing: at every timer tick, it
 determines which steppers to advance, and generates the required tick
 pulses. It does this using a Bresenham algorithm. When a block is
 completed, the step-generator fetches the next block from the block
 buffer (discarding the old one). Note that the last block in the buffer
 is never discarded, i.e. there is always a 'current' block.
 
 The velocity controller runs at a (usually lower) fixed rate, driven
 by the app_systick() hook. Every system tick, it looks ahead through
 the block buffer, determining the maximum current speed such that none
 of the maximum speeds prescribed by the oncoming blocks will be
 violated (given the maximum acceleration). It adjusts the tick
 frequency of the ARM timer which drives the step-generator, thereby
 controlling the speed of the steppers.
 
 In its current implementation, the velocity controller only deals with
 one common acceleration value (in steps/s^2) for all axis.



 */


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


/*
 Copy a new block into the buffer
    
 Returns 0 on success, -1 on error (buffer full)
 This is the only place where buffer_head is modified
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


/*
 Current state of the stepper algorithm
 
*/

typedef struct {
    stepper_block_t *current_block;
    int32_t accu[MAX_AXIS];         //Bresenham accumulator
    
    uint32_t all_step_mask; // binary OR of all step_masks; initialized by stepper_init
    uint32_t all_dir_mask;  // binary OR of all dir_masks; initialized by stepper_init
} stepper_status_t;

static stepper_status_t st;

/*
 Hardware configuration of the machine: I/O mapping of the axes
*/
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




static const float dt = 0.001f;

static float vmax = 80.0f * 0.001f;              // mm/timestep
static float steps_per_mm = (32*400.0f/40.0f);
static float mm_per_step = (40.0f/(32*400.0f));
static float amax = 100.0f  * 0.001f * 0.001f;         // mm/(timestep^2)
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
        if (i == 1) break;
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
       step_masks and dir_masks, respectively. These are precomputed for use
       by the step-generator.
    */
    st.all_step_mask=0;
    st.all_dir_mask=0;
    for (int i=0; i<stepper_config.naxis; i++) {
        st.all_step_mask |= stepper_config.step_mask[i];
        st.all_dir_mask |= stepper_config.dir_mask[i];
    }
    
    
    GPCLR0 = st.all_step_mask | st.all_dir_mask;
    
    palSetGroupMode(&IOPORT0, st.all_step_mask | st.all_dir_mask, 0, PAL_MODE_OUTPUT);
    
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


/*
 Single-pass online velocity planner
 
 Default: accelerate upto maximum velocity of the current block.
 
 Then, for each subsequent block, compute the accumulated distance from
 the present position. Determine the maximum current velocity such that
 decelleration downto the maximum velocity at each position is possible.
 Limit the current velocity to the minimum of those.
 
 Several computations are done using the squared value of parameters.
 These are denoted by the suffix _sq . This is done to reduce the number
 of computationally expensive square root calculations.
 
*/

void app_systick(void) {

    unsigned int my_buffer_tail;
    stepper_block_t *bl;
    
    my_buffer_tail = buffer_tail; //atomic copy
    /* FIQ interrupt could update buffer_tail, but data will remain valid
       since only FIQ code can write to buffer. We're already called from interrupt,
       so no INT interrupt will occur. Thus, stepper_put_buffer cannot be called
       while we are processing, so data stays as-is. Only buffer[i]->step_position
       can be changed by FIQ while we are processing
     */

    int cumulative_steps = 0; // Total steps from present position upto point
    float new_speed_sq;       // Lowest speed limit found so far
    float speed_limit_sq;     // Temporary: calculation of a speed limit for a given section
    
    // default: maximum acceleration
    new_speed_sq = sq(current_speed + amax);

    // Find the current block, and the number of steps still to go in that block
    bl = &buffer[my_buffer_tail];
    cumulative_steps = bl->total_steps - bl->step_position;
    
    // do not accelerate beyond this block's max velocity
    new_speed_sq = limit(new_speed_sq, bl->vmax_sq);
    
    // compute max current speed in order to be able to decellerate
    // to bl->vmax_end_sq before reaching end of this block
    speed_limit_sq = 2*amax*cumulative_steps*mm_per_step + bl->vmax_end_sq;
    new_speed_sq = limit(new_speed_sq, speed_limit_sq);
    
    // Loop through the buffer from current to the future
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
    
    // be prepared to stop at the end of the buffer
    speed_limit_sq = 2*amax*cumulative_steps*mm_per_step;
    new_speed_sq = limit(new_speed_sq, speed_limit_sq);    
    
    
    // Compute the timer reload value for the found speed
    uint32_t reload_value;
    current_speed = sqrtf(new_speed_sq); // mm per dt

    if (current_speed < 0.001) { // todo determine appropriate value
        reload_value = 10000;
    } else {
        reload_value = (1000*mm_per_step) / current_speed;
        if (reload_value>10000) reload_value = 10000; //minimum 100 steps/s
    }
    
    ARM_TIMER_RLD = reload_value;
}


__attribute__((naked)) 
void FiqHandler(void) {
    // FIQ handler init code; push all registers that are shared with other
    // modes than FIQ mode
    asm volatile ("stmfd    sp!, {r0-r7, r12, lr}" : : : "memory");
    dmb();
    
    uint32_t step_mask;
    stepper_block_t *bl;
    
    // All step signals low
    GPCLR0 = st.all_step_mask;
    
    // Get next block, if required
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
    GPCLR0 = st.all_dir_mask & ~bl->direction_bits;    
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
    
    
    // FIQ handler exit code
    dmb();
    asm volatile ("ldmfd    sp!, {r0-r7, r12, lr}" : : : "memory");    
    asm volatile ("subs    pc, lr, #4" : : : "memory");    

}