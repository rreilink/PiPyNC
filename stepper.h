#ifndef _app_h
#define _app_h

#define MAX_AXIS 8


/*
 Hardware configuration of the machine: I/O mapping of the axes
*/
typedef struct {
    int naxis;
    uint32_t step_mask[MAX_AXIS];
    uint32_t dir_mask[MAX_AXIS];
    int core_axes[2];
    float steps_per_mm[MAX_AXIS];
    float machine_steps_per_mm;
    float max_acceleration;
} stepper_config_t;

extern stepper_config_t stepper_config;
extern char* stepper_config_structdef;

int stepper_queuemove(float target[], float vmax, uint32_t home_mask);
void stepper_abort(void);
void stepper_init_buffer(void);

#endif