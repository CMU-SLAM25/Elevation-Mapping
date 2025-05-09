# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mapping_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mapping_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mapping_FOUND FALSE)
  elseif(NOT mapping_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mapping_FOUND FALSE)
  endif()
  return()
endif()
set(_mapping_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mapping_FIND_QUIETLY)
  message(STATUS "Found mapping: 1.0.0 (${mapping_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mapping' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mapping_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mapping_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mapping_DIR}/${_extra}")
endforeach()
