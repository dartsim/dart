#ifndef DART_PYTHON_POINTERS_H_
#define DART_PYTHON_POINTERS_H_
#include <memory>
#include <dartpy/config.h>
#include "types.h"

#ifdef BOOST_CONFIG_HPP
#error "pointers.h must be included before any Boost headers"
#endif

#include "get_signature.h"
#include <dart/dynamics/dynamics.hpp>

#include <dart/dynamics/dynamics.hpp>
// TODO(JS): This includes Boost headers, which conflicts with 
// the above ifdef statement. Confirm whether this doesn't cause
// any issues.

namespace boost {

#ifndef DARTPY_HAS_STD_SHARED_GET_POINTER
template <class T>
T* get_pointer(const std::shared_ptr<T>& p) 
{
  return p.get();
}
#endif // ifndef DARTPY_HAS_STD_SHARED_GET_POINTER

template <class BodyNodeT>
BodyNodeT* get_pointer(
  const dart::dynamics::TemplateBodyNodePtr<BodyNodeT>& p)
{
  return p.get();
}

template <class NodeT, class BodyNodeT>
NodeT* get_pointer(
  const dart::dynamics::TemplateNodePtr<NodeT, BodyNodeT>& p)
{
  return p.get();
}

template <class JointT, class BodyNodeT>
JointT* get_pointer(
  const dart::dynamics::TemplateJointPtr<JointT, BodyNodeT>& p)
{
  return p.get();
}

template <class DegreeOfFreedomT, class BodyNodeT>
DegreeOfFreedomT* get_pointer(
  const dart::dynamics::TemplateDegreeOfFreedomPtr<
    DegreeOfFreedomT, BodyNodeT>& p)
{
  return p.get();
}

} // namespace boost

// We CANNOT include anything that defines get_pointer() before we define our
// own overrides. This file does so indirectly.
#include <boost/python.hpp>
#include <boost/python/pointee.hpp>

namespace boost {
namespace python {

template <class BodyNodeT>
struct pointee<dart::dynamics::TemplateBodyNodePtr<BodyNodeT>>
{
  using type = BodyNodeT;
};

template <class NodeT, class BodyNodeT>
struct pointee<dart::dynamics::TemplateNodePtr<NodeT, BodyNodeT>>
{
  using type = NodeT;
};

template <class JointT, class BodyNodeT>
struct pointee<dart::dynamics::TemplateJointPtr<JointT, BodyNodeT>>
{
  using type = JointT;
};

template <class DegreeOfFreedomT, class BodyNodeT>
struct pointee<dart::dynamics::TemplateDegreeOfFreedomPtr<
  DegreeOfFreedomT, BodyNodeT>>
{
  using type = DegreeOfFreedomT;
};

} // namespace python
} // namespace boost

namespace boost {
namespace python {
namespace detail {

// attempting to instantiate this type will result in a compiler error,
// if that happens it means you're trying to use return_by_smart_pointer
// on a function/method that doesn't return a pointer!
template <class R>
struct return_by_smart_ptr_requires_a_pointer_return_type
# if defined(__GNUC__) && __GNUC__ >= 3 || defined(__EDG__)
    {}
# endif
    ;


// this is where all the work is done, first the plain pointer is
// converted to a smart pointer, and then the smart pointer is embedded
// in a Python object
template <class Ptr>
struct make_owning_smart_ptr_holder
{
    template <class T>
    static PyObject *execute(T* p)
    {
        Ptr ptr(p);
        return objects::make_ptr_instance<T,
                objects::pointer_holder<Ptr, T>
            >::execute(ptr);
    }
};

} // namespace detail

template <class Ptr>
struct return_by_smart_ptr
{
    template <class T>
    struct apply
    {
        typedef typename mpl::if_c<
            boost::is_pointer<T>::value,
            to_python_indirect<T,
                detail::make_owning_smart_ptr_holder<Ptr> >,
            detail::return_by_smart_ptr_requires_a_pointer_return_type<T>
        >::type type;
    };
};

template <class T, class Ptr>
struct smart_ptr_from_python
{
    smart_ptr_from_python()
    {
        converter::registry::insert(&convertible, &construct, type_id<Ptr>()
#ifndef BOOST_PYTHON_NO_PY_SIGNATURES
            , &converter::expected_from_python_type_direct<T>::get_pytype
#endif
        );
    }

private:
    static void *convertible(PyObject *p)
    {
        if (p == Py_None) {
            return p;
        }
        return converter::get_lvalue_from_python(p,
            converter::registered<T>::converters);
    }
    
    static void construct(PyObject* source,
                          converter::rvalue_from_python_stage1_data* data)
    {
        void *const storage
            = ((converter::rvalue_from_python_storage<Ptr>*)data)->storage.bytes;

        // Deal with the "None" case.
        if (data->convertible == source) {
            new (storage) Ptr();
        } else {
#if 0
            boost::shared_ptr<void> hold_convertible_ref_count(
              (void*)0, shared_ptr_deleter(handle<>(borrowed(source))) );
            // use aliasing constructor
            new (storage) Ptr(
                hold_convertible_ref_count,
                static_cast<T*>(data->convertible));
#endif
            new (storage) Ptr(static_cast<T *>(data->convertible));
        }
        
        data->convertible = storage;
    }
};

}} // namespace boost::python

#endif // ifndef DART_PYTHON_POINTERS_H_
