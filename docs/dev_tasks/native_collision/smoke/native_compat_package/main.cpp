#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/collision/collision_detector.hpp>
#include <dart/collision/dart/dart_collision_detector.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>

#include <memory>
#include <string>

namespace {

bool isDartBacked(
    const std::shared_ptr<dart::collision::CollisionDetector>& detector)
{
  return dynamic_cast<dart::collision::DartCollisionDetector*>(detector.get())
         != nullptr;
}

} // namespace

int main()
{
  auto* factory = dart::collision::CollisionDetector::getFactory();
  const std::shared_ptr<dart::collision::CollisionDetector> fcl
      = factory->create("fcl");
  const std::shared_ptr<dart::collision::CollisionDetector> bullet
      = factory->create("bullet");
  const std::shared_ptr<dart::collision::CollisionDetector> ode
      = factory->create("ode");

  if (!fcl || !bullet || !ode) {
    return 1;
  }

  if (std::string(fcl->getTypeView()) != "dart"
      || std::string(bullet->getTypeView()) != "dart"
      || std::string(ode->getTypeView()) != "dart") {
    return 2;
  }

  const std::shared_ptr<dart::collision::CollisionDetector> fclClass
      = dart::collision::FCLCollisionDetector::create();
  const std::shared_ptr<dart::collision::CollisionDetector> bulletClass
      = dart::collision::BulletCollisionDetector::create();
  const std::shared_ptr<dart::collision::CollisionDetector> odeClass
      = dart::collision::OdeCollisionDetector::create();

  if (!isDartBacked(fclClass) || !isDartBacked(bulletClass)
      || !isDartBacked(odeClass)) {
    return 3;
  }

  if (std::string(fclClass->getTypeView()) != "fcl"
      || std::string(bulletClass->getTypeView()) != "bullet"
      || std::string(odeClass->getTypeView()) != "ode") {
    return 4;
  }

  return 0;
}
