# $Id: CMakeLists.txt 3857 2016-12-23 13:08:43Z bradbell $
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell
#
# CppAD is distributed under multiple licenses. This distribution is under
# the terms of the
#                     GNU General Public License Version 3.
#
# A copy of this license is included in the COPYING file of this distribution.
# Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
# -----------------------------------------------------------------------------

# Specify build type for this directory and all its subdirectories
# (subdirectories do not specify build type).
# Possible values are DEBUG, RELEASE, RelWithDebInfo and MinSizeRel
# CppAD developers can use DEBUG for debugging speed tests.
SET(CMAKE_BUILD_TYPE RELEASE)

# Initialize list of tests as empty
SET(check_speed_depends "")

# The CMakeLists.txt file in the specified source directory is processed
# before the current input file continues beyond this command.
# add_subdirectory(source_dir [binary_dir] [EXCLUDE_FROM_ALL])
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(cppad)
ADD_SUBDIRECTORY(double)
ADD_SUBDIRECTORY(example)
IF ( cppad_profile_flag )
	ADD_SUBDIRECTORY(profile)
ENDIF ( cppad_profile_flag )
#
IF ( adolc_prefix )
	ADD_SUBDIRECTORY(adolc)
ENDIF( adolc_prefix )
IF ( fadbad_prefix )
	ADD_SUBDIRECTORY(fadbad)
ENDIF( fadbad_prefix )
IF ( sacado_prefix )
	ADD_SUBDIRECTORY(sacado)
ENDIF( sacado_prefix )

# check_speed
ADD_CUSTOM_TARGET(check_speed DEPENDS ${check_speed_depends} )
MESSAGE(STATUS "make check_speed: available")

# Change check depends in parent environment
add_to_list(check_depends check_speed)
SET(check_depends "${check_depends}" PARENT_SCOPE)
