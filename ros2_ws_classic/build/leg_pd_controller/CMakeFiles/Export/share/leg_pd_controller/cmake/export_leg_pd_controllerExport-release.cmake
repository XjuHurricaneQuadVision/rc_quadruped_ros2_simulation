#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "leg_pd_controller::leg_pd_controller" for configuration "Release"
set_property(TARGET leg_pd_controller::leg_pd_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(leg_pd_controller::leg_pd_controller PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/leg_pd_controller/libleg_pd_controller.so"
  IMPORTED_SONAME_RELEASE "libleg_pd_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS leg_pd_controller::leg_pd_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_leg_pd_controller::leg_pd_controller "${_IMPORT_PREFIX}/lib/leg_pd_controller/libleg_pd_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
