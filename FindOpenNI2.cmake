###############################################################################
# Find OpenNI2
#
# This sets the following variables:
# OPENNI2_FOUND - True if OPENNI2 was found.
# OPENNI2_INCLUDE_DIRS - Directories containing the OPENNI2 include files.
# OPENNI2_LIBRARIES - Libraries needed to use OPENNI.
###############################################################################


# Dependencies
set(OPENNI2_INCLUDE_PATH           PATHS ${PROJECT_SOURCE_DIR}/include/OpenNI-2        NO_DEFAULT_PATH)
set(OPENNI2_LIBRARY_PATH           PATHS ${PROJECT_SOURCE_DIR}/lib                     NO_DEFAULT_PATH)
set(OPENNI2_DRIVER_LIBRARY_PATH    PATHS ${PROJECT_SOURCE_DIR}/lib//OpenNI2/Drivers    NO_DEFAULT_PATH)

#Find headers
find_path(OPENNI2_ROOT_INCLUDE_DIR      NAMES OpenNI.h                   ${OPENNI2_INCLUDE_PATH})
find_path(OPENNI2_DRIVER_INCLUDE_DIR    NAMES Driver/OniDriverAPI.h             ${OPENNI2_INCLUDE_PATH})
find_path(OPENNI2_LINUX_INCLUDE_DIR     NAMES Linux-x86/OniPlatformLinux-x86.h     ${OPENNI2_INCLUDE_PATH})


#main libraries:
find_library(OPENNI2_MAIN_LIBRARY        NAMES OpenNI2     ${OPENNI2_LIBRARY_PATH})
mark_as_advanced(OPENNI2_MAIN_LIBRARY)

find_library(OPENNI2_ONIFILE_LIBRARY     NAMES OniFile     ${OPENNI2_DRIVER_LIBRARY_PATH})
mark_as_advanced(OPENNI2_ONIFILE_LIBRARY)

find_library(OPENNI2_PS1080_LIBRARY      NAMES PS1080      ${OPENNI2_DRIVER_LIBRARY_PATH})
mark_as_advanced(OPENNI2_PS1080_LIBRARY)

# Output variables generation
include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(OPENNI2 DEFAULT_MSG OPENNI2_MAIN_LIBRARY
                                                      OPENNI2_ONIFILE_LIBRARY
                                                      OPENNI2_PS1080_LIBRARY
                                                      OPENNI2_ROOT_INCLUDE_DIR
                                                      OPENNI2_DRIVER_INCLUDE_DIR
                                                      OPENNI2_LINUX_INCLUDE_DIR)

if(OPENNI2_FOUND)
  set(OPENNI2_INCLUDE_DIRS ${OPENNI2_ROOT_INCLUDE_DIR}
                                 ${OPENNI2_DRIVER_INCLUDE_DIR}
                                 ${OPENNI2_LINUX_INCLUDE_DIR})

  set(OPENNI2_LIBRARIES   ${OPENNI2_MAIN_LIBRARY}
                                ${OPENNI2_ONIFILE_LIBRARY}
                                ${OPENNI2_PS1080_LIBRARY})
endif(OPENNI2_FOUND)

#message("=> ***** OPENNI2_FOUND = ${OPENNI2_FOUND}")
#message("=> ***** OPENNI2_INCLUDE_DIRS = ${OPENNI2_INCLUDE_DIRS}")
#message("=> ***** OPENNI2_LIBRARIES = ${OPENNI2_LIBRARIES}")
