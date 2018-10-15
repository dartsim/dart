{{{header}}}
{{#includes}}
#include <{{{.}}}>
{{/includes}}

#include <boost/python.hpp>
#include <cmath>
#include <iostream>

/* main postinclude */

BOOST_PYTHON_MODULE({{module.name}})
{
::boost::python::docstring_options options(true, true, false);

{{{precontent}}}
{{#module.namespaces}}{{#name}}
  ::boost::python::scope(){{!
    }}{{#scope}}{{#name}}.attr("{{name}}"){{/name}}{{/scope}}.attr("{{name}}") = {{!
    }}::boost::python::object(::boost::python::handle<>({{!
        }}::boost::python::borrowed(::PyImport_AddModule({{!
            }}"{{module.name}}{{#scope}}{{#name}}.{{name}}{{/name}}{{/scope}}.{{name}}"))));
{{/name}}{{/module.namespaces}}

{{#module.bindings}}
  void {{.}}();
  try {
    {{.}}();
  } catch(...) {
    std::cerr << "Exception in: {{.}}" << std::endl;
    throw;
  }

{{/module.bindings}}
{{{postcontent}}}
}
{{{footer}}}
