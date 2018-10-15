{{{header}}}
{{#includes}}
#include <{{{.}}}>
{{/includes}}
{{{precontent}}}
#include <boost/python.hpp>
#include <cmath>

/* postinclude */

void {{function.mangled_name}}()
{
::boost::python::object parent_object(::boost::python::scope(){{!
    }}{{#function.scope}}{{#name}}.attr("{{name}}"){{/name}}{{/function.scope}});
::boost::python::scope parent_scope(parent_object);

{{#function.overloads}}{{!
    }}::boost::python::def("{{name}}", []({{#params}}{{{type}}} {{name}}{{^last}}, {{/last}}{{/params}}) -> {{{return_type}}} { {{!
    }}return {{{qualified_call}}}({{#params}}{{name}}{{^last}}, {{/last}}{{/params}}); }{{!
    }}{{#return_value_policy}}, ::boost::python::return_value_policy<::boost::python::{{{.}}} >(){{/return_value_policy}}{{!
    }}{{#params?}}, ({{#params}}::boost::python::arg("{{name}}"){{^last}}, {{/last}}{{/params}}){{/params?}});
{{/function.overloads}}

}
{{{postcontent}}}
{{{footer}}}
