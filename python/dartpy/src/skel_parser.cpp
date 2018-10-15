#include <dartpy/pointers.h>
#include <dartpy/template_registry.h>
#include <dart/dart.hpp>
#include <dart/collision/bullet/bullet.hpp>
#include <dart/optimizer/optimizer.hpp>
#include <dart/optimizer/nlopt/nlopt.hpp>
#include <dart/planning/planning.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/gui/gui.hpp>

/* precontent */
#include <boost/python.hpp>
#include <cmath>

/* postinclude */

void skel_parser()
{
::boost::python::object parent_object(::boost::python::scope().attr("utils").attr("skel"));
::boost::python::scope parent_scope(parent_object);

::boost::python::def("readWorld", [](const dart::common::Uri & uri) -> dart::simulation::WorldPtr { return dart::utils::SkelParser::readWorld(uri); }, (::boost::python::arg("uri")));
::boost::python::def("readWorld", [](const dart::common::Uri & uri, const dart::common::ResourceRetrieverPtr & retriever) -> dart::simulation::WorldPtr { return dart::utils::SkelParser::readWorld(uri, retriever); }, (::boost::python::arg("uri"), ::boost::python::arg("retriever")));
::boost::python::def("readWorldXML", [](const std::string & xmlString) -> dart::simulation::WorldPtr { return dart::utils::SkelParser::readWorldXML(xmlString); }, (::boost::python::arg("xmlString")));
::boost::python::def("readWorldXML", [](const std::string & xmlString, const dart::common::Uri & baseUri) -> dart::simulation::WorldPtr { return dart::utils::SkelParser::readWorldXML(xmlString, baseUri); }, (::boost::python::arg("xmlString"), ::boost::python::arg("baseUri")));
::boost::python::def("readWorldXML", [](const std::string & xmlString, const dart::common::Uri & baseUri, const dart::common::ResourceRetrieverPtr & retriever) -> dart::simulation::WorldPtr { return dart::utils::SkelParser::readWorldXML(xmlString, baseUri, retriever); }, (::boost::python::arg("xmlString"), ::boost::python::arg("baseUri"), ::boost::python::arg("retriever")));
::boost::python::def("readSkeleton", [](const dart::common::Uri & uri) -> dart::dynamics::SkeletonPtr { return dart::utils::SkelParser::readSkeleton(uri); }, (::boost::python::arg("uri")));
::boost::python::def("readSkeleton", [](const dart::common::Uri & uri, const dart::common::ResourceRetrieverPtr & retriever) -> dart::dynamics::SkeletonPtr { return dart::utils::SkelParser::readSkeleton(uri, retriever); }, (::boost::python::arg("uri"), ::boost::python::arg("retriever")));

}

/* footer */
