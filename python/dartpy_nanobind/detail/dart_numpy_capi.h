// NumPy 2 float64 fast export; older NumPy uses nanobind ndarray export.
#pragma once

#include <Python.h>
#include <cstdint>

namespace dart_numpy_capi {

constexpr bool uses_numpy2_capi(const char *version) noexcept {
    return version[0] == '2' && version[1] == '.';
}
static_assert(!uses_numpy2_capi("1.26.4"));
static_assert(uses_numpy2_capi("2.0.0"));
static_assert(uses_numpy2_capi("2.2.5"));

// Return -1 on Python errors; cache only a successful runtime selection.
inline int numpy2_export() noexcept {
    static int selected = -1;
    if (selected >= 0)
        return selected;
    PyObject *module = PyImport_ImportModule("numpy");
    if (!module)
        return -1;
    PyObject *version = PyObject_GetAttrString(module, "__version__");
    Py_DECREF(module);
    if (!version)
        return -1;
    const char *text = PyUnicode_AsUTF8AndSize(version, nullptr);
    if (text)
        selected = uses_numpy2_capi(text);
    Py_DECREF(version);
    return selected;
}


inline void **array_api() noexcept {
    // shortcut: GIL and one interpreter only, revisit for isolated interpreters.
    static void **cached = nullptr;
    if (cached)
        return cached;
    PyObject *module = PyImport_ImportModule("numpy._core.multiarray");
    if (!module)
        return nullptr;
    PyObject *capsule = PyObject_GetAttrString(module, "_ARRAY_API");
    Py_DECREF(module);
    if (!capsule)
        return nullptr;
    void **api = (void **) PyCapsule_GetPointer(capsule, nullptr);
    Py_DECREF(capsule);
    if (!api)
        return nullptr;

    using Version = unsigned int (*)();
    unsigned int abi = ((Version) api[0])();
    if (abi != 0x02000000 || ((Version) api[211])() < 0x00000012) {
        PyErr_Format(PyExc_RuntimeError,
                     "Eigen C API export requires NumPy 2 ABI (got 0x%x)", abi);
        return nullptr;
    }
    cached = api;
    return api;
}

// owner is borrowed; SetBaseObject consumes the extra reference on all paths.
inline PyObject *from_data(int ndim, const size_t *shape,
                           const int64_t *element_strides, void *data,
                           PyObject *owner, bool writable = true) noexcept {
    static_assert(sizeof(double) == 8, "This export requires float64 doubles");
    if (ndim < 1 || ndim > 2 || !owner) {
        PyErr_SetString(PyExc_ValueError,
                        "Eigen C API export requires 1/2 dimensions and an owner");
        return nullptr;
    }
    Py_ssize_t dims[2], strides[2];
    bool empty = false;
    for (int i = 0; i < ndim; ++i) {
        if (shape[i] > (size_t) PY_SSIZE_T_MAX ||
            element_strides[i] > PY_SSIZE_T_MAX / 8 ||
            element_strides[i] < PY_SSIZE_T_MIN / 8) {
            PyErr_SetString(PyExc_OverflowError,
                            "Eigen shape/stride exceeds NumPy index range");
            return nullptr;
        }
        dims[i] = (Py_ssize_t) shape[i];
        strides[i] = (Py_ssize_t) element_strides[i] * 8;
        empty |= shape[i] == 0;
    }
    if (!data) {
        if (!empty) {
            PyErr_SetString(PyExc_ValueError, "Nonempty Eigen array has null data");
            return nullptr;
        }
        // A null pointer asks NumPy to allocate; empty Eigen storage needs a view.
        static double empty_storage = 0;
        data = &empty_storage;
    }
    void **api = array_api();
    if (!api)
        return nullptr;
    using NewArray = PyObject *(*)(PyTypeObject *, int, const Py_ssize_t *, int,
                                  const Py_ssize_t *, void *, int, int,
                                  PyObject *);
    using SetBase = int (*)(PyObject *, PyObject *);
    // NumPy public API: slots 2/93/282, NPY_DOUBLE=12, WRITEABLE=0x0400.
    PyObject *result = ((NewArray) api[93])(
        (PyTypeObject *) api[2], ndim, dims, 12, strides, data, 0,
        writable ? 0x0400 : 0, nullptr);
    if (!result)
        return nullptr;
    Py_INCREF(owner);
    if (((SetBase) api[282])(result, owner) < 0) {
        Py_DECREF(result);
        return nullptr;
    }
    return result;
}

} // namespace dart_numpy_capi
