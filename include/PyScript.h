#ifndef PY_SCRIPT_H
#define PY_SCRIPT_H

#include <string>
#include <Python.h>

#include <rtabmap/utilite/ULogger.h>

static bool PY_SCRIPT_ACTIVE = false;

class PyScript {
private:
    std::string moduleName_;
    PyObject* module_;
public:
    PyScript(const std::string& moduleName);
    ~PyScript();
    template<typename... Args>
    void call(const std::string& func, const std::string& fmt, Args&&... args) {
        PyObject* pyResult = PyObject_CallMethod(module_, func.c_str(), fmt.c_str(), std::forward<Args>(args)...);
        if(pyResult != NULL) {
            if(PyObject_IsTrue(pyResult)) { 
                Py_DECREF(pyResult);
                UERROR("Failed to invoke function [%s] in [%s]:", func.c_str(), moduleName_.c_str());
                PyErr_Print();
                assert(false);
            }
            Py_DECREF(pyResult);
        } else {
            UERROR("Failed to invoke function [%s] in [%s]:", func.c_str(), moduleName_.c_str());
            PyErr_Print();
            assert(false);
        }
    }
};

#endif // PY_SCRIPT_H