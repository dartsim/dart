cmake_minimum_required(VERSION 3.22.1)

project(dart-examples)

# None GUI examples
add_subdirectory(hello_world)
add_subdirectory(cylindrical_constraint)
add_subdirectory(speed_test)

# OSG renderer examples
add_subdirectory(contact_benchmark)
add_subdirectory(demos)
add_subdirectory(adaptive_soft_contact)
add_subdirectory(soft_worm)
