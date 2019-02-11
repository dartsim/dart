{{header}}
{{#includes}}
#include <{{.}}>
{{/includes}}
{{#sources}}
#include <{{.}}>
{{/sources}}
#include <boost/python.hpp>
{{postinclude}}

void {{enum.mangled_name}}()
{
    {{precontent}}

    ::boost::python::object parent_object(::boost::python::scope(){{!
        }}{{#enum.scope}}{{#name}}.attr("{{name}}"){{/name}}{{/enum.scope}});
    ::boost::python::scope parent_scope(parent_object);

    ::boost::python::enum_<{{enum.type}}>("{{enum.name}}"){{#enum.values}}
        .value("{{name}}", {{qualified_name}}){{/enum.values}}
    ;

    {{postcontent}}
}
{{footer}}
