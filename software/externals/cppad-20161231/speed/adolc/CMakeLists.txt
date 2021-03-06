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
# Build the speed/adolc directory tests
# Inherit build type from ../CMakeList.txt

# assert adolc_prefix is defined
IF ( NOT adolc_prefix )
	MESSAGE(FATAL_ERROR
		"speed/adolc/CMakeLists.txt: adolc_prefix = ${adolc_prefix}"
	)
ENDIF ( NOT adolc_prefix )


# Adds flags to the compiler command line for sources in the current directory
# and below. This command can be used to add any flags, but it was originally
# intended to add preprocessor definitions.
ADD_DEFINITIONS("-DCPPAD_ADOLC_SPEED")

# Local include directories to search (not in package_prefix/includdir)
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/../src )

# add_executable(<name> [WIN32] [MACOSX_BUNDLE] [EXCLUDE_FROM_ALL]
#                 source1 source2 ... sourceN
# )
ADD_EXECUTABLE(speed_adolc EXCLUDE_FROM_ALL ../main.cpp alloc_mat.cpp
	det_lu.cpp
	det_minor.cpp
	mat_mul.cpp
	ode.cpp
	poly.cpp
	sparse_hessian.cpp
	sparse_jacobian.cpp
)

# Compiler flags for cppad source
add_cppad_cxx_flags( speed_adolc )

# List of libraries to be linked into the specified target
TARGET_LINK_LIBRARIES(speed_adolc
	speed_src
	${cppad_lib}
	adolc
	${colpack_libs}
)

# Add the check_speed_adolc target
ADD_CUSTOM_TARGET(
	check_speed_adolc
	speed_adolc correct 54321 colpack
	DEPENDS
	speed_adolc speed_src
)
MESSAGE(STATUS "make check_speed_adolc: available")

# Change check depends in parent environment
add_to_list(check_speed_depends check_speed_adolc)
SET(check_speed_depends "${check_speed_depends}" PARENT_SCOPE)
