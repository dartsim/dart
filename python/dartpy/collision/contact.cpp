#include "collision/contact.hpp"

#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/Contact.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

nb::object get_user_data(const dart::collision::Contact& contact)
{
  if (!contact.userData)
    return nb::none();
  return nb::capsule(contact.userData);
}

void set_user_data(dart::collision::Contact& contact, const nb::handle& handle)
{
  if (handle.is_none()) {
    contact.userData = nullptr;
    return;
  }
  nb::capsule capsule = nb::cast<nb::capsule>(handle);
  contact.userData = capsule.data();
}

} // namespace

void defContact(nb::module_& m)
{
  using Contact = dart::collision::Contact;

  nb::class_<Contact>(m, "Contact")
      .def(nb::init<>())
      .def_rw("point", &Contact::point)
      .def_rw("normal", &Contact::normal)
      .def_rw("force", &Contact::force)
      .def_rw("collisionObject1", &Contact::collisionObject1)
      .def_rw("collisionObject2", &Contact::collisionObject2)
      .def_rw("penetrationDepth", &Contact::penetrationDepth)
      .def_rw("triID1", &Contact::triID1)
      .def_rw("triID2", &Contact::triID2)
      .def_prop_rw("userData", &get_user_data, &set_user_data)
      .def_static("getNormalEpsilon", &Contact::getNormalEpsilon)
      .def_static("getNormalEpsilonSquared", &Contact::getNormalEpsilonSquared)
      .def_static("isZeroNormal", &Contact::isZeroNormal, nb::arg("normal"))
      .def_static(
          "isNonZeroNormal", &Contact::isNonZeroNormal, nb::arg("normal"));
}

} // namespace dart::python_nb
