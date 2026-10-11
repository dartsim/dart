function(dart_gui_filament_generate_material_headers out_headers generated_dir)
  set(_headers)
  foreach(
    _material
    IN
    ITEMS
      default_lit
      textured_lit
      transparent_lit
      transparent_textured_lit
      legacy_lit
      legacy_textured_lit
      legacy_transparent_lit
      legacy_transparent_textured_lit
      debug_color
      imgui
      depth
      point_color
      opaque_point_color
  )
    set(_source_material "${_material}")
    set(_material_options)
    if(_material STREQUAL default_lit)
      set(_symbol kDefaultLitMaterial)
    elseif(_material STREQUAL textured_lit)
      set(_symbol kTexturedLitMaterial)
    elseif(_material STREQUAL transparent_lit)
      set(_symbol kTransparentLitMaterial)
    elseif(_material STREQUAL transparent_textured_lit)
      set(_symbol kTransparentTexturedLitMaterial)
    elseif(_material STREQUAL debug_color)
      set(_symbol kDebugColorMaterial)
    elseif(_material STREQUAL legacy_lit)
      set(_symbol kLegacyLitMaterial)
    elseif(_material STREQUAL legacy_textured_lit)
      set(_symbol kLegacyTexturedLitMaterial)
    elseif(_material STREQUAL legacy_transparent_lit)
      set(_symbol kLegacyTransparentLitMaterial)
    elseif(_material STREQUAL legacy_transparent_textured_lit)
      set(_symbol kLegacyTransparentTexturedLitMaterial)
    elseif(_material STREQUAL point_color)
      set(_symbol kPointColorMaterial)
    elseif(_material STREQUAL opaque_point_color)
      set(_symbol kOpaquePointColorMaterial)
      set(_source_material point_color)
      set(_material_options -Pblending=opaque -Pname=dartOpaquePointColor)
    elseif(_material STREQUAL depth)
      set(_symbol kDepthMaterial)
    else()
      set(_symbol kImGuiMaterial)
    endif()
    set(
      _source
      "${DART_SOURCE_DIR}/dart/gui/detail/materials/${_source_material}.mat"
    )
    set(_binary "${generated_dir}/${_material}.filamat")
    set(_header "${generated_dir}/${_material}_material.hpp")
    set(_dependencies "${_source}")
    if(_source_material MATCHES "textured_lit$|^point_color$")
      list(
        APPEND _dependencies
        "${DART_SOURCE_DIR}/dart/gui/detail/materials/legacy_texture_sampling.glsl"
      )
    endif()
    add_custom_command(
      OUTPUT "${_binary}"
      COMMAND "${CMAKE_COMMAND}" -E make_directory "${generated_dir}"
      COMMAND
        Filament::matc --api opengl --api vulkan --api metal --platform desktop
        ${_material_options} -o "${_binary}" "${_source}"
      DEPENDS ${_dependencies} Filament::matc
      VERBATIM
    )
    add_custom_command(
      OUTPUT "${_header}"
      COMMAND
        "${CMAKE_COMMAND}" "-DINPUT=${_binary}" "-DOUTPUT=${_header}"
        "-DSYMBOL=${_symbol}" -DNAMESPACE=dart::gui::detail -P
        "${DART_SOURCE_DIR}/cmake/dart_binary_to_header.cmake"
      DEPENDS
        "${_binary}"
        "${DART_SOURCE_DIR}/cmake/dart_binary_to_header.cmake"
      VERBATIM
    )
    list(APPEND _headers "${_header}")
  endforeach()
  set(${out_headers} "${_headers}" PARENT_SCOPE)
endfunction()
