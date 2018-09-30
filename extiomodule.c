/*
 * I2C addresses: 
 *  - MCP23017 0x20 + RW
 *  - ADC Pi Zero 0x6C (channels 1-4)
 *                0x6D (channels 5-8)
 *
 */

#include "Python.h"
#include "pythread.h"
#include "ch.h"
#include "hal.h"

static I2CDriver *channel = &I2C1;

#define NUM_OUTPUTS 16
#define PWM_MAX 100

typedef struct {
    int pwm_mode;
    int value;
    int value_copy; // copy of value for use during the current PWM cycle
} out_t;

static out_t outputs[NUM_OUTPUTS]={0};

CondVar extio_trigger_cv;

static int started = 0;
 
static void ExtIoThread(void *interp) {
  uint8_t gpout[3];
  uint16_t bits, bit_value;
  int pwm_phase = 0;
  
  Mutex mtx;
  msg_t result;
  
  gpout[0] = 0x14; // Address of GPIO extender OLATA
  gpout[1] = 0;
  gpout[2] = 0;
 
  chRegSetThreadName("extio");
  
  chMtxInit(&mtx);

  PyThreadState *tstate = PyThreadState_New((PyInterpreterState *) interp);

  while (TRUE) {

    chMtxLock(&mtx);
    chCondWait (&extio_trigger_cv);
    chMtxUnlock();

    bits = 0;
    bit_value = 1;
    for(int i=0;i<NUM_OUTPUTS;i++) {
        if (pwm_phase == 0) outputs[i].value_copy = outputs[i].value;
        if (pwm_phase < outputs[i].value_copy) bits |= bit_value;
        bit_value<<=1;
    }
    
    pwm_phase++;
    if (pwm_phase>=PWM_MAX) pwm_phase = 0;
    
    gpout[1] = bits;
    gpout[2] = bits >> 8;
  
    if (channel->state >= I2C_READY) {
        i2cAcquireBus(channel);
        result = i2cMasterTransmitTimeout(channel, 0x20, gpout, 3, NULL, 0, CH_FREQUENCY/10);
        i2cReleaseBus(channel);
    }
  
  }

}

static PyObject*
extio_start(PyObject *self, PyObject *args) {
    if (!started) {
        started = 1;
        PySys_WriteStdout("0\n");
        PyThread_start_new_thread(ExtIoThread, (void*) PyThreadState_GET()->interp);

    }
    Py_RETURN_NONE;
}


static PyObject*
extio_set_pwm_mode(PyObject *self, PyObject *args) {
    int output, pwm_mode;
    if (!PyArg_ParseTuple(args, "ip", &output, &pwm_mode)) {
        return NULL;
    } 
    
    if (output<0 || output >=NUM_OUTPUTS) {
        return PyErr_Format(PyExc_ValueError, "output should be 0 <= output < %d", NUM_OUTPUTS);
    }
    
    outputs[output].pwm_mode = output ? 1 : 0;
    Py_RETURN_NONE;    
}

static PyObject*
extio_set_output(PyObject *self, PyObject *args) {
    int output, value;
    if (!PyArg_ParseTuple(args, "ii", &output, &value)) {
        return NULL;
    } 
    
    if (output<0 || output >=NUM_OUTPUTS) {
        return PyErr_Format(PyExc_ValueError, "output should be 0 <= output < %d", NUM_OUTPUTS);
    }
    
    if (outputs[output].pwm_mode) {
        if (value<0) {
            value = 0;
        } else if (value>PWM_MAX) {
            value = PWM_MAX;
        }
    } else {
        value = value ? PWM_MAX : 0;
    }
    outputs[output].value = value;
    
    Py_RETURN_NONE;    
}


static PyMethodDef ExtioMethods[] = {
    {"start", extio_start, METH_NOARGS, "Start I/O task"},
    {"set_pwm_mode", extio_set_pwm_mode, METH_VARARGS, "Set output PWM mode on or off"},
    {"set_output", extio_set_output, METH_VARARGS, "Set output value"},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

static struct PyModuleDef extiomodule = {
    PyModuleDef_HEAD_INIT,
    "extio",   /* name of module */
    NULL, /* module documentation, may be NULL */
    -1,       /* size of per-interpreter state of the module,
                 or -1 if the module keeps state in global variables. */
    ExtioMethods
};




PyMODINIT_FUNC
PyInit_extio(void) {    
    return PyModule_Create(&extiomodule);
}


