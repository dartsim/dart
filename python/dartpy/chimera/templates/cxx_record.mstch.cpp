{{header}}
{{#includes}}
#include <{{.}}>
{{/includes}}
{{#sources}}
#include <{{.}}>
{{/sources}}
#include <boost/python.hpp>
{{postinclude}}

namespace {

{{#class.comment?}}{{!
}}constexpr char {{class.mangled_name}}_docstring[] = R"CHIMERA_STRING({{!
}}{{#class.comment}}{{!
}}{{.}}
{{/class.comment}}{{!
}})CHIMERA_STRING";
{{/class.comment?}}

{{#class.methods}}{{!
}}{{#comment?}}{{!
}}constexpr char {{mangled_name}}_docstring[] = R"CHIMERA_STRING({{!
}}{{#comment}}{{!
}}{{.}}
{{/comment}}{{!
}})CHIMERA_STRING";

{{/comment?}}{{!
}}{{/class.methods}}

} // namespace

void {{class.mangled_name}}()
{
    {{precontent}}

    ::boost::python::object parent_object(::boost::python::scope(){{!
        }}{{#class.scope}}{{#name}}.attr("{{name}}"){{/name}}{{/class.scope}});
    ::boost::python::scope parent_scope(parent_object);

    ::boost::python::class_<{{class.type}}{{^class.is_copyable}}, {{!
        }}::boost::noncopyable{{/class.is_copyable}}{{#class.held_type}}, {{!
        }}{{.}}{{/class.held_type}}{{#class.bases?}}, {{!
        }}::boost::python::bases<{{!
            }}{{#class.bases}}{{qualified_name}}{{^last}}, {{/last}}{{/class.bases}}{{!
        }} >{{/class.bases?}} >("{{class.name}}"{{!
        }}{{#class.comment?}}, {{class.mangled_name}}_docstring{{/class.comment?}}{{!
        }}, boost::python::no_init){{!

/* constructors */}}
{{#class.constructors}}{{!
}}{{#overloads}}{{!
    }}        .def("__init__", ::boost::python::make_constructor({{!
        }}+[]({{#params}}{{type}} {{name}}{{^last}}, {{/last}}{{/params}}){{!
        }} -> {{class.type}} * { {{!
        }}return new {{class.type}}({{#params}}{{name}}{{^last}}, {{/last}}{{/params}}); }{{!
        }}, ::boost::python::default_call_policies(){{!
        }}{{#params?}}, ({{#params}}::boost::python::arg("{{name}}"){{^last}}, {{/last}}{{/params}}){{/params?}}))
{{/overloads}}{{!
}}{{/class.constructors}}{{!

/* member functions */}}
{{#class.methods}}{{!
}}{{^is_static}}{{!
}}{{#overloads}}{{!
    }}        .def("{{name}}", +[]({{#is_const}}const {{/is_const}}{{class.type}} *self{{#params}}, {{type}} {{name}}{{/params}}){{!
    }}{{#is_void}} { {{/is_void}}{{!
    }}{{^is_void}} -> {{return_type}} { return {{/is_void}}{{!
    }}self->{{call}}({{#params}}{{name}}{{^last}}, {{/last}}{{/params}}); }{{!
    }}{{#return_value_policy}}, ::boost::python::return_value_policy<::boost::python::{{.}} >(){{/return_value_policy}}{{!
    }}{{#comment?}}, {{mangled_name}}_docstring{{/comment?}}{{!
    }}{{#params?}}, ({{#params}}::boost::python::arg("{{name}}"){{^last}}, {{/last}}{{/params}}){{/params?}})
{{/overloads}}{{!
}}{{/is_static}}{{!
}}{{/class.methods}}{{!

/* static member functions */}}
{{#class.methods}}{{!
}}{{#is_static}}{{!
}}{{#overloads}}{{!
    }}        .def("{{name}}", +[]({{#params}}{{type}} {{name}}{{^last}}, {{/last}}{{/params}}){{!
    }}{{#is_void}} { {{/is_void}}{{!
    }}{{^is_void}} -> {{return_type}} { return {{/is_void}}{{!
    }}{{qualified_call}}({{#params}}{{name}}{{^last}}, {{/last}}{{/params}}); }{{!
    }}{{#return_value_policy}}, ::boost::python::return_value_policy<::boost::python::{{.}} >(){{/return_value_policy}}{{!
    }}{{#params?}}, ({{#params}}::boost::python::arg("{{name}}"){{^last}}, {{/last}}{{/params}}){{/params?}})
{{/overloads}}{{!
}}{{/is_static}}{{!
}}{{/class.methods}}
{{#class.static_methods}}{{!
    }}        .staticmethod("{{.}}")
{{/class.static_methods}}{{!

/* fields */}}
{{#class.fields}}{{!
    }}{{#is_assignable}}        .def_readwrite{{/is_assignable}}{{!
    }}{{^is_assignable}}        .def_readonly{{/is_assignable}}{{!
    }}("{{name}}", &{{qualified_name}})
{{/class.fields}}{{!

/* static fields */
/* TODO: Add make_setter if this property is assignable */}}
{{#class.static_fields}}{{!
    }}        .add_static_property("{{name}}", {{!
    }}::boost::python::make_getter({{qualified_name}}))
{{/class.static_fields}}
    ;

    {{postcontent}}
}
{{footer}}
