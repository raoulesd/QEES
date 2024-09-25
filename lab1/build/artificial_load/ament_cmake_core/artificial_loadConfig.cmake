# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_artificial_load_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED artificial_load_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(artificial_load_FOUND FALSE)
  elseif(NOT artificial_load_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(artificial_load_FOUND FALSE)
  endif()
  return()
endif()
set(_artificial_load_CONFIG_INCLUDED TRUE)

# output package information
if(NOT artificial_load_FIND_QUIETLY)
  message(STATUS "Found artificial_load: 0.0.1 (${artificial_load_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'artificial_load' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message(WARNING "${_msg}")
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(artificial_load_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${artificial_load_DIR}/${_extra}")
endforeach()
