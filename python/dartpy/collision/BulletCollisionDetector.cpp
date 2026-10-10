// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/config.hpp>

#if HAVE_BULLET

  #include <dart/collision/bullet/bullet.hpp>

namespace dart {
namespace python {

void BulletCollisionDetector(nb::module_& m)
{
  dartnb::dart_class<
      dart::collision::BulletCollisionDetector,
      dart::collision::CollisionDetector>(m, "BulletCollisionDetector")
      .def(dartnb::factory(
          +[]() -> std::shared_ptr<dart::collision::BulletCollisionDetector> {
            return dart::collision::BulletCollisionDetector::create();
          }))
      .def(
          "cloneWithoutCollisionObjects",
          +[](const dart::collision::BulletCollisionDetector* self)
              -> std::shared_ptr<dart::collision::CollisionDetector> {
            return self->cloneWithoutCollisionObjects();
          })
      .def(
          "getType",
          +[](const dart::collision::BulletCollisionDetector* self)
              -> const std::string& { return self->getType(); },
          nb::rv_policy::reference_internal)
      .def(
          "createCollisionGroup",
          +[](dart::collision::BulletCollisionDetector* self)
              -> std::shared_ptr<dart::collision::CollisionGroup> {
            return self->createCollisionGroupAsSharedPtr();
          })
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::collision::BulletCollisionDetector::getStaticType();
          },
          nb::rv_policy::reference_internal);
}

} // namespace python
} // namespace dart

#endif // HAVE_BULLET
