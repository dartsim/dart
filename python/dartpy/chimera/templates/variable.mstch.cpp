{{{header}}}
{{#includes}}
#include <{{{.}}}>
{{/includes}}
{{{precontent}}}
#include <boost/python.hpp>
#include <cmath>

/* postinclude */

void {{variable.mangled_name}}()
{
::boost::python::object parent_object(::boost::python::scope(){{!
    }}{{#variable.scope}}{{#name}}.attr("{{name}}"){{/name}}{{/variable.scope}});
::boost::python::scope parent_scope(parent_object);

::boost::python::scope().attr("{{variable.name}}") = {{{variable.qualified_name}}};
}
{{{postcontent}}}
{{{footer}}}
