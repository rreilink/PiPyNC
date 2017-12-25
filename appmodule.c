#include "Python.h"


extern volatile float current_speed;
extern unsigned int test_ctr;

static PyObject *
_app_getstate(PyObject *self, PyObject *args) {

    if (!PyArg_ParseTuple(args, "")) {
        return NULL;
    }  

    return Py_BuildValue("fi", current_speed, test_ctr);
}

int dummy;

static PyObject *
_app_setacceleration(PyObject *self, PyObject *args) {
    if (!PyArg_ParseTuple(args, "i", &dummy)) {
        return NULL;
    }  

    Py_RETURN_NONE;
    
}

static PyObject *
_app_setspeed(PyObject *self, PyObject *args) {
    if (!PyArg_ParseTuple(args, "i", &dummy)) {
        return NULL;
    }  

    Py_RETURN_NONE;
    
}


static PyMethodDef _AppMethods[] = {
    {"getstate", _app_getstate, METH_VARARGS, "Get stepper state"},
    {"setacc", _app_setacceleration, METH_VARARGS, "Set stepper acceleration"},
    {"setspeed", _app_setspeed, METH_VARARGS, "Set stepper speed"},
    
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

static struct PyModuleDef _appmodule = {
    PyModuleDef_HEAD_INIT,
    "app",   /* name of module */
    NULL, /* module documentation, may be NULL */
    -1,       /* size of per-interpreter state of the module,
                 or -1 if the module keeps state in global variables. */
    _AppMethods
};


PyMODINIT_FUNC
PyInit_app(void) {
    PyObject *module = NULL;
    
    module = PyModule_Create(&_appmodule);
    if (!module) goto error;

    return module;
    
error:
    Py_XDECREF(module);
    return NULL;
}
