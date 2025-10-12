Code Style Guide
================

This section describes the code style used in DART project.

C++ Style Guide
---------------

Macro Definitions
~~~~~~~~~~~~~~~~~

In DART, we use macros to define compile-time constants and control code flow.
All macros in our codebase are prefixed with ``DART_`` to distinguish them from
other identifiers. Macros that control optional dependencies and features
follow a consistent naming convention:

* ``DART_HAS_<optional_dep>``: A boolean value that is set to true when an
  optional dependency is detected in the system.
* ``DART_ENABLE_<optional_feature>``: A boolean value that is set to true if
  the optional feature should be enabled when the requirements are met.
* ``DART_ENABLED_<optional_feature>``: A boolean value that is set to true if
  the optional feature is enabled.

We use all-caps for all macro names to ensure consistency and to visually
distinguish macros from other types of variables.

Python Style Guide
------------------

Naming Conventions
~~~~~~~~~~~~~~~~~~

This project uses different naming conventions for the C++ code and the Python
bindings. In the C++ code, function names are in camelCase and variables and
member variables use snake_case, whereas in the Python bindings, both function
names and variables use snake_case.

Here are the naming conventions used in the Python bindings:

* Function names are in snake_case, with words separated by underscores (e.g.
  `calculate_average`).
* Class names are in CamelCase, with the first letter of each word in uppercase
  (e.g. `MyClass`).
* Variables and member variables are in snake_case, with words separated by
  underscores (e.g. `my_variable`).
* Constants are in ALL_CAPS, with words separated by underscores (e.g.
  `MY_CONSTANT`).
* Namespaces are represented by modules, and are in lowercase, with words
  separated by underscores (e.g. `my_module.my_namespace`).

For example, the identity member function is called isIdentity in C++:

.. code-block:: cpp

   auto so3 = SO3();
   bool is_identity = so3.isIdentity();

while it is called is_identity in Python:

.. code-block:: python

   so3 = SO3()
   is_identity = so3.is_identity()

Motivations for Different Naming Conventions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The reason for using different naming conventions in the C++ code and the
Python bindings is to follow the conventions that are most commonly used in
each language. The camelCase convention is more common for function names in
the C++ community, while the snake_case convention is more common for function
names in the Python community.

By using the standard naming conventions in each language, we can make the code
more readable and easier to understand for developers who are familiar with
each language. Consistency within each language is important, but it's also
crucial to document the conventions clearly so that other developers can
understand how to use the code and what the naming conventions mean in each
context. Additionally, following the naming conventions of each language can
help with integration with other Python modules or projects.

CMake Style Guide
-----------------

TODO
