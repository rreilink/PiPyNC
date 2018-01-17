#include "Python.h"
#include "stepper.h"



static PyObject *
_app_getstate(PyObject *self, PyObject *args) {

    if (!PyArg_ParseTuple(args, "")) {
        return NULL;
    }  

    return Py_BuildValue("fi", 0.0, 0);
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


static PyObject *
_app_moveto(PyObject *self, PyObject *args) {
    float target_f[MAX_AXIS]; // TODO: number of axes
    PyObject *target;
    PyObject *iter;
    PyObject *item;
    int i;
    float vmax;
    
    for(int i=0;i<MAX_AXIS;i++) target_f[i] = 0.0f;

    if (!PyArg_ParseTuple(args, "Of", &target, &vmax)) {
        return NULL;
    } 

    // Iterate over target, convert items to float
    iter = PyObject_GetIter(target);
    if (!iter) return NULL; //target is not iterable

    for(i=0;i<stepper_config.naxis;i++) {
        item = PyIter_Next(iter);
        if (item==NULL) {
            if (!PyErr_Occurred()) {
                // No error, but end of iterator too early
                PyErr_SetString(PyExc_ValueError, "too few items in target");
            }
            goto error;
        }
        target_f[i] = PyFloat_AsDouble(item);
        if (PyErr_Occurred()) goto error;
        Py_DECREF(item);
        
    }
    
    // Check that the iterator is exhausted when naxis items have been popped
    item = PyIter_Next(iter);
    if (PyErr_Occurred()) goto error;
    if (item != NULL) {
        PyErr_SetString(PyExc_ValueError, "too many items in target");
        goto error;
    }
    
    Py_DECREF(iter);
    
    // Perform the move
    if (stepper_queuemove(target_f, vmax)!=0) { // Error
        PyErr_SetString(PyExc_RuntimeError, "Queue full");
        return NULL;
    }

    Py_RETURN_NONE;
    
error:
    Py_XDECREF(item);
    Py_XDECREF(iter);
    
    return NULL;
}


static PyObject *
_app_moveto2(PyObject *self, PyObject *args) {
    float target[MAX_AXIS]; // TODO: number of axes
    int i;
    float vmax;
    
    for(int i=0;i<8;i++) target[i] = 0.0f;
    
    if (!PyArg_ParseTuple(args, "fff", &target[0], &target[1], &vmax)) {
        return NULL;
    }  

    if (stepper_queuemove(target, vmax)!=0) { // Error
        PyErr_SetString(PyExc_RuntimeError, "Queue full");
        return NULL;
    }

    Py_RETURN_NONE;
    
}


static PyMethodDef _AppMethods[] = {
    {"getstate", _app_getstate, METH_VARARGS, "Get stepper state"},
    {"setacc", _app_setacceleration, METH_VARARGS, "Set stepper acceleration"},
    {"queuemove", _app_moveto, METH_VARARGS, "Queue movement to given coordinate"},
    
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

static struct PyModuleDef _appmodule = {
    PyModuleDef_HEAD_INIT,
    "_app",   /* name of module */
    NULL, /* module documentation, may be NULL */
    -1,       /* size of per-interpreter state of the module,
                 or -1 if the module keeps state in global variables. */
    _AppMethods
};



// TODO: combine add_memoryview and add_string as module_add
static int add_memoryview(PyObject *module, void* data, ssize_t size, char* name) {
    PyObject *memoryview = NULL;
    memoryview = PyMemoryView_FromMemory(data, size, PyBUF_WRITE);
    
    if (memoryview == NULL) return -1;
    if (PyObject_SetAttrString(module, name, memoryview) != 0) {
        Py_DECREF(memoryview);
        return -1;
    }
    return 0;

}

static int add_string(PyObject *module, char* data, char* name) {
    PyObject *str;
    str = PyUnicode_FromString(data);
    
    if (str == NULL) return -1;
    if (PyObject_SetAttrString(module, name, str) != 0) {
        Py_DECREF(str);
        return -1;
    }
    return 0;

}

PyMODINIT_FUNC
PyInit_app(void) {
    PyObject *module = NULL;
    
    module = PyModule_Create(&_appmodule);
    if (!module) return NULL;
    
    if (add_memoryview(module, &stepper_config, sizeof(stepper_config), "_stepper_config_data") == -1) goto error;
    if (add_string(module, stepper_config_structdef, "_stepper_config_structdef")==-1) goto error;
    
    return module;
    
error:
    Py_DECREF(module);
    return NULL;
}
