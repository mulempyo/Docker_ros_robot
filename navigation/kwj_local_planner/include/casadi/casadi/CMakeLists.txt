# Turn a runtime file into a file with strings
macro(CASADI_STRINGIFY STRFILE)
  # Start with an empty file
  file(WRITE ${STRFILE} "")
  foreach(FILE ${ARGN})
    # Add declaration of string
    get_filename_component(FILENAME ${FILE} NAME_WE)
    file(APPEND ${STRFILE} "const char* ${FILENAME}_str =")
    # Append file as strings
    file(STRINGS ${FILE} FILE_CONTENTS)
    foreach(LINE ${FILE_CONTENTS})
      string(REPLACE "\\" "\\\\" LINE "${LINE}") # Replace \ with \\
      string(REPLACE "\"" "\\\"" LINE "${LINE}") # Replace " with \"
      file(APPEND ${STRFILE} "\n  \"${LINE}\\n\"")
    endforeach()
    # End declaration
    file(APPEND ${STRFILE} ";\n\n")
  endforeach()
endmacro()

# this function handles libraries and plugins
# there is so much similarity in plugins/libraries, we just
# use a flag and wrap this with casadi_plugin or casadi_library (above)
macro(casadi_library name)
  if(ENABLE_STATIC)
	if(DEFINED CASADI_LIBRARY_PREFIX)
		SET_TARGET_PROPERTIES(${name} PROPERTIES PREFIX ${CMAKE_SHARED_LIBRARY_PREFIX}${CASADI_LIBRARY_PREFIX})
	endif()
    add_library(${name} STATIC ${ARGN})
  endif()
  if(ENABLE_SHARED)
    add_library(${name} SHARED ${ARGN})
    SET_TARGET_PROPERTIES(${name} PROPERTIES GNUtoMS 1)
	if(DEFINED CASADI_LIBRARY_PREFIX)
		SET_TARGET_PROPERTIES(${name} PROPERTIES PREFIX ${CMAKE_SHARED_LIBRARY_PREFIX}${CASADI_LIBRARY_PREFIX})
	endif()
  endif()
  generate_export_header(${name})
  
  file(RELATIVE_PATH TREL_LIB_PREFIX "${CMAKE_INSTALL_PREFIX}" "${LIB_PREFIX}")
  file(RELATIVE_PATH TREL_BIN_PREFIX "${CMAKE_INSTALL_PREFIX}" "${BIN_PREFIX}")

  # Link with the core
  if(NOT "${name}" STREQUAL "casadi")
    # Not core
    target_link_libraries(${name} casadi)
    install(TARGETS ${name}
      LIBRARY DESTINATION ${TREL_LIB_PREFIX}
      ARCHIVE DESTINATION ${TREL_LIB_PREFIX}
      RUNTIME DESTINATION ${TREL_BIN_PREFIX}
    )
  else()
    # Core
    install(TARGETS ${name}
      # IMPORTANT: Add the library to the "export-set"
      EXPORT casadi-targets
      LIBRARY DESTINATION ${TREL_LIB_PREFIX}
      ARCHIVE DESTINATION ${TREL_LIB_PREFIX}
      RUNTIME DESTINATION ${TREL_BIN_PREFIX}
    )
  endif()

  if(WITH_SO_VERSION)
    SET_TARGET_PROPERTIES(${name} PROPERTIES SOVERSION ${PACKAGE_SO_VERSION})
  endif()

  if(WITH_LINT)
    add_style_check_target(${name} "${ARGN}")
    set_property(GLOBAL APPEND PROPERTY LINT_TARGETS  "lint_${name}")
  endif()

  if(WITH_CLANG_TIDY)
    add_clang_tidy_target(${name} "${ARGN}")
    set_property(GLOBAL APPEND PROPERTY CLANG_TIDY_TARGETS  "clang_tidy_${name}")
    set_property(GLOBAL APPEND PROPERTY CLANG_TIDY_FIX_TARGETS  "clang_tidy_fix_${name}")
  endif()

  if(WITH_SPELL)
    add_spell_check_target(${name} "${ARGN}")
    set_property(GLOBAL APPEND PROPERTY SPELL_TARGETS  "spell_${name}")
    set(SPELL_TARGETS ${SPELL_TARGETS} "spell_${name}" PARENT_SCOPE)
  endif()

  set_property(GLOBAL APPEND PROPERTY CASADI_MODULES ${name})
endmacro()

macro(casadi_plugin Type name)
  string(TOLOWER ${Type} type)
  casadi_library(casadi_${type}_${name} ${ARGN})
  set_property(GLOBAL APPEND PROPERTY CASADI_PLUGINS "${Type}::${name}")
endmacro()

macro(casadi_plugin_link_libraries Type name)
  string(TOLOWER ${Type} type)
  target_link_libraries(casadi_${type}_${name} ${ARGN})
endmacro()

if(ENABLE_SHARED)
  if(NOT ENABLE_EXPORT_ALL)
    set(CMAKE_CXX_VISIBILITY_PRESET hidden)
    set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
    if(("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU") AND (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10.0) AND (CMAKE_BUILD_TYPE STREQUAL "Debug"))
      # Bug exposed in recent gcc https://github.com/casadi/casadi/issues/2862
      set(CMAKE_VISIBILITY_INLINES_HIDDEN OFF)
    endif()
  endif()
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CASADI_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_CASADI_C_FLAGS}")

add_subdirectory(core)
add_subdirectory(interfaces)
add_subdirectory(solvers)

if(WITH_LINT)
  get_property(LINT_TARGETS GLOBAL PROPERTY LINT_TARGETS)
  add_custom_target(lint DEPENDS ${LINT_TARGETS} VERBATIM)
endif()
if(WITH_CLANG_TIDY)
  get_property(CLANG_TIDY_TARGETS GLOBAL PROPERTY CLANG_TIDY_TARGETS)
  get_property(CLANG_TIDY_FIX_TARGETS GLOBAL PROPERTY CLANG_TIDY_FIX_TARGETS)
  add_custom_target(clang-tidy DEPENDS ${CLANG_TIDY_TARGETS} VERBATIM)
  add_custom_target(clang-tidy-fix DEPENDS ${CLANG_TIDY_FIX_TARGETS} VERBATIM)
endif()
if(WITH_SPELL)
  get_property(SPELL_TARGETS GLOBAL PROPERTY SPELL_TARGETS)
  add_custom_target(spell DEPENDS ${SPELL_TARGETS} VERBATIM)
endif()

get_property(CASADI_MODULES GLOBAL PROPERTY CASADI_MODULES)
get_property(CASADI_PLUGINS GLOBAL PROPERTY CASADI_PLUGINS)

set(CASADI_MODULES ${CASADI_MODULES} PARENT_SCOPE)
set(CASADI_PLUGINS ${CASADI_PLUGINS} PARENT_SCOPE)

add_custom_target(libs ALL DEPENDS ${CASADI_MODULES})

# Main header files
if(NOT SKIP_CONFIG_H_GENERATION)

  # See issue #3831, RELOCATABILITY
  set(CASADI_CMAKE_CXX_COMPILER ${CMAKE_CXX_COMPILER})
  if(CASADI_CMAKE_CXX_COMPILER MATCHES "shims" OR RELOCATABLE)
    get_filename_component(CASADI_CMAKE_CXX_COMPILER ${CMAKE_CXX_COMPILER} NAME)
  endif()

  if(RELOCATABLE)
    set(CASADI_INSTALL_PREFIX "")
  else()
    set(CASADI_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
  endif()

  set(EXTRA_CXX_FLAGS_FROM_DEFS "")
  foreach(DEF ${EXTRA_CXX_FLAGS_FROM_DEFS_LIST})
    set(EXTRA_CXX_FLAGS_FROM_DEFS "${EXTRA_CXX_FLAGS_FROM_DEFS} ${DEF}")
  endforeach()

  configure_file(config.h.cmake "${CMAKE_CURRENT_BINARY_DIR}/config.h" ESCAPE_QUOTES)
endif()
install(FILES casadi.hpp mem.h casadi_c.h "${CMAKE_CURRENT_BINARY_DIR}/config.h"
        DESTINATION ${INCLUDE_PREFIX}/casadi)
