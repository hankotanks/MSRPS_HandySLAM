#include "PyScript.h"

#include <cstdarg>
#include <string>
#include <Python.h>

#include <rtabmap/utilite/ULogger.h>

PyScript& PyScript::get() {
    static PyScript instance(HANDY_PY_SCRIPT_MODULE_NAME);
    return instance;
}

PyScript::PyScript(const std::string& moduleName) : moduleName_(moduleName) {
    PyConfig config;
    PyConfig_InitIsolatedConfig(&config);
    PyConfig_SetBytesString(&config, &config.program_name, moduleName_.c_str());

    Py_InitializeFromConfig(&config);
    PyConfig_Clear(&config);

    const char* pathExt[] = {
        HANDY_DIR_PY_ENV_PURE,
        HANDY_DIR_PY_ENV_PLAT,
        HANDY_DIR_SCRIPTS,
        HANDY_DIR_PROMPTDA
    };

    PyObject* pyPath = PySys_GetObject("path");
    PyObject* pyPathTemp;
    for(const char* path : pathExt) {
        pyPathTemp = PyUnicode_FromString(path);
        PyList_Append(pyPath, pyPathTemp);
        Py_DECREF(pyPathTemp);
    }

    PyObject* pyModule = PyUnicode_DecodeFSDefault(moduleName_.c_str());
    module_ = PyImport_Import(pyModule);
    if(module_ == NULL) {
        UERROR("Failed to load module [%s].", moduleName_.c_str());
        assert(false);
    }

    Py_DECREF(pyModule);
}

PyScript::~PyScript() {
    Py_DECREF(module_);
    Py_Finalize();
}
