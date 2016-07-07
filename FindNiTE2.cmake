###############################################################################
# Find NITE2
#
# This sets the following variables:
# NITE2_FOUND - True if OPENNI2 was found.
# NITE2_INCLUDE_DIRS - Directories containing the OPENNI2 include files.
# NITE2_LIBRARIES - Libraries needed to use OPENNI.
###############################################################################

set(NITE2_INCLUDE_PATH           PATHS /usr/include/NiTE-2   NO_DEFAULT_PATH)
set(NITE2_LIBRARY_PATH           PATHS /usr/lib              NO_DEFAULT_PATH)

#Find headers
find_path(NITE2_INCLUDE_DIR             NAMES   NiTE.h                   ${NITE2_INCLUDE_PATH})

#main libraries:
find_library(NITE2_LIBRARY        NAMES    NiTE2       ${NITE2_LIBRARY_PATH})
mark_as_advanced(NITE2_LIBRARY)

find_library(NITE2_DRIVER_LIBRARY        NAMES    NiTE2       ${NITE2_LIBRARY_PATH})
mark_as_advanced(NITE2_LIBRARY)


# Output variables generation
include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(NITE2 DEFAULT_MSG NITE2_LIBRARY
                                                    NITE2_INCLUDE_DIR)

if(NITE2_FOUND)
  set(NITE2_INCLUDE_DIRS  ${NITE2_INCLUDE_DIR})
  set(NITE2_LIBRARIES     ${NITE2_LIBRARY})
endif(NITE2_FOUND)

#message("=> ***** NITE2_FOUND = ${OPENNI2_FOUND}")
#message("=> ***** NITE2_INCLUDE_DIRS = ${NITE2_INCLUDE_DIRS}")
#message("=> ***** NITE2_LIBRARIES = ${NITE2_LIBRARIES}")
