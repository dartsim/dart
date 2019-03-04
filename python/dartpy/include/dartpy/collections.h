#ifndef DARTPY_COLLECTIONS_H_
#define DARTPY_COLLECTIONS_H_
#include <vector>
#include <boost/python.hpp>

namespace dart {
namespace python {
namespace util {

template <class T, class ValueType = T>
struct vector_to_python
{
  vector_to_python()
  {
    boost::python::to_python_converter<
      std::vector<T>, vector_to_python<T, ValueType>>();
  }

  static PyObject* convert(const std::vector<T>& vec)
  {
    boost::python::list* py_list = new boost::python::list();

    for (const T& element : vec)
      py_list->append(ValueType(element));

    return py_list->ptr();
  }
};

} // namespace util
} // namespace python
} // namespace dart

#endif // ifndef DARTPY_COLLECTIONS_H_
