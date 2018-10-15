#ifndef DART_PYTHON_UTIL_H_
#define DART_PYTHON_UTIL_H_
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

// See below. Based off of: http://stackoverflow.com/a/25403872/111426
#define DISCARD_FUNCTION_RETURN(_FUNC_)\
    ::dart::python::util::discard_function_return<decltype(&_FUNC_), &_FUNC_>
#define DISCARD_METHOD_RETURN(_CLASS_, _FUNC_)\
    ::dart::python::util::discard_method_return<_CLASS_, decltype(&_FUNC_), &_FUNC_>

namespace dart {
namespace python {
namespace util {

// Define a wrapper function that returns void. This requires variadic
// templates and C++11 perfect forwarding gymnastics.
// Source: http://stackoverflow.com/a/25403872/111426
template <typename Fn, Fn fn, typename... Args>
void discard_function_return(Args&&... args)
{
    fn(std::forward<Args>(args)...);
}

// This is the same as discard_function_return for class methods. It takes an
// additional template argument (the class) and an additional function argument
// (the instance). This is compatible with Boost.Python.
template <typename T, typename Fn, Fn fn, typename... Args>
void discard_method_return(T *instance, Args&&... args)
{
    (instance->*fn)(std::forward<Args>(args)...);
}

// from_python converter for iterable collections. The container_type must
// have: (1) have a container_type::value_type typedef and (2) a constructor
// that accepts two forward_iterator<container_type::value_type>.
// 
// Source:
// https://misspent.wordpress.com/2009/09/27/how-to-write-boost-python-converters/
template <typename container_type>
struct collection_from_python
{
    typedef typename container_type::value_type content_type;

    collection_from_python()
    {
        ::boost::python::converter::registry::push_back(
            &convertible,
            &construct,
            ::boost::python::type_id<container_type>()
        );
    }
 
    static void *convertible(PyObject *object_py_raw)
    {
        PyObject *iterator_py_raw = PyObject_GetIter(object_py_raw);
        if (iterator_py_raw) {
            Py_DECREF(iterator_py_raw);
            return object_py_raw;
        } else {
            PyErr_Clear();
            return NULL;
        }
    }
 
    static void construct(
        PyObject *object_py_raw,
        boost::python::converter::rvalue_from_python_stage1_data *data)
    {
        using ::boost::python::borrowed;
        using ::boost::python::converter::rvalue_from_python_storage;
        using ::boost::python::handle;
        using ::boost::python::object;
        using ::boost::python::stl_input_iterator;
        using ::std::make_pair;

        // Wrap the PyObject in a Boost.Python class.
        object object_py(handle<>(borrowed(object_py_raw)));

        // Allocate storage for the output object using placement-new.
        void *storage
            = reinterpret_cast<rvalue_from_python_storage<container_type> *>(data)
                ->storage.bytes;

        new (storage) container_type(
            stl_input_iterator<content_type>(object_py),
            stl_input_iterator<content_type>()
        );

        data->convertible = storage;
    }
};

} // namespace util
} // namespace python
} // namespace dart

#endif
