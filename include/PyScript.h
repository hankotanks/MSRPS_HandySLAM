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
private:
    PyScript(const std::string& moduleName);
    ~PyScript();
private:
    PyScript(const PyScript&) = delete;
    PyScript(PyScript&&) = delete;
    PyScript& operator=(const PyScript&) = delete;
    PyScript& operator=(PyScript&&) = delete;
public:
    static PyScript& get();
    template<typename... Args>
    // returns truthy if call succeeded
    bool call(const std::string& func, const std::string& fmt, Args&&... args) {
        PyObject* pyResult = PyObject_CallMethod(module_, func.c_str(), fmt.c_str(), std::forward<Args>(args)...);
        if(pyResult != NULL) {
            if(pyResult == Py_None) return true;
            bool result = PyObject_IsTrue(pyResult);
            Py_DECREF(pyResult);
            return result;
        } else {
            UERROR("Failed to invoke function [%s] in [%s]:", func.c_str(), moduleName_.c_str());
            PyErr_Print();
            return false;
        }
    }
};

#endif // PY_SCRIPT_H