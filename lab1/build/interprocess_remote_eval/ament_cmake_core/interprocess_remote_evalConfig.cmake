# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_interprocess_remote_eval_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED interprocess_remote_eval_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(interprocess_remote_eval_FOUND FALSE)
  elseif(NOT interprocess_remote_eval_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(interprocess_remote_eval_FOUND FALSE)
  endif()
  return()
endif()
set(_interprocess_remote_eval_CONFIG_INCLUDED TRUE)

# output package information
if(NOT interprocess_remote_eval_FIND_QUIETLY)
  message(STATUS "Found interprocess_remote_eval: 0.0.0 (${interprocess_remote_eval_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'interprocess_remote_eval' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message(WARNING "${_msg}")
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(interprocess_remote_eval_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${interprocess_remote_eval_DIR}/${_extra}")
endforeach()
