{{header}}
{{#includes}}
#include <{{.}}>
{{/includes}}
{{#sources}}
#include <{{.}}>
{{/sources}}
#include <boost/python.hpp>
{{postinclude}}
{{#module.bindings}}
void {{.}}();
{{/module.bindings}}

BOOST_PYTHON_MODULE({{module.name}})
{
    {{precontent}}

    ::boost::python::docstring_options options(true, true, false);

{{#module.namespaces}}{{#name}}
    ::boost::python::scope(){{!
        }}{{#scope}}{{#name}}.attr("{{name}}"){{/name}}{{/scope}}.attr("{{name}}") = {{!
        }}::boost::python::object(::boost::python::handle<>({{!
              }}::boost::python::borrowed(::PyImport_AddModule({{!
                  }}"{{module.name}}{{#scope}}{{#name}}.{{name}}{{/name}}{{/scope}}.{{name}}"))));
{{/name}}{{/module.namespaces}}

{{#module.bindings}}
    {{.}}();
{{/module.bindings}}

    {{postcontent}}
}
{{footer}}
