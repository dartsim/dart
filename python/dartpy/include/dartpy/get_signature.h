#ifndef DART_PYTHON_GET_SIGNATURE_H_
#define DART_PYTHON_GET_SIGNATURE_H_
#include <boost/mpl/erase.hpp>
#include <boost/mpl/next.hpp>
#include <boost/utility.hpp>
#include <boost/function_types/is_callable_builtin.hpp>
#include <boost/mpl/size.hpp>
#include <boost/mpl/greater.hpp>
#include <boost/mpl/size_t.hpp>

// Source: https://mail.python.org/pipermail/cplusplus-sig/2014-February/017103.html

namespace boost {
namespace python {
namespace detail {

template<typename T, typename Enable = void>
struct fun_sign
{
  typedef T type;
};

template<typename T>
struct fun_sign<
  T,
  typename enable_if<
    mpl::greater<
      mpl::size<T>,
      mpl::size_t<1>
    >
  >::type
>
{
  typedef typename mpl::erase<
    T,
    typename mpl::next<
      typename mpl::begin<
        T
      >::type
    >::type
  >::type type;
};

template<typename T>
typename disable_if<typename function_types::is_callable_builtin<T>, typename fun_sign<typename function_types::components<decltype(&T::operator())>::types>::type>::type
get_signature(T, void* = 0)
{
return typename fun_sign<typename function_types::components<decltype(&T::operator())>::types>::type();
}

} // namespace detail
} // namespace python
} // namespace boost

#endif // ifndef DART_PYTHON_GET_SIGNATURE_H_
