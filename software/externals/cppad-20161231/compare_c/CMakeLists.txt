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
# $begin compare_c$$ $newlinech #$$
# $spell
#	det
#	cpp
# $$
#
# $section Compare Speed of C and C++$$
# $index compare, speed C and C++$$
# $index speed, compare C and C++$$
# $index C, compare speed with C++$$
# $index C++, compare speed with C$$
#
# $head Syntax$$
# $codei%compare_c/det_by_minor_c
# %$$
# $codei compare_c/det_by_minor_cpp$$
#
# $head Purpose$$
# Compares the speed of the exact same source code compiled
# using C versus C++.
#
# $childtable%
#	compare_c/det_by_minor.c
# %$$
#
# $end
# -----------------------------------------------------------------------------

# Specifies build type for this directory. Possible values are
# empty, Debug, Release, RelWithDebInfo and MinSizeRel
SET(CMAKE_BUILD_TYPE RELEASE)

# Loop though the C and C++ compilers
FOREACH( com c cpp )
	# Copy a file to another location and modify its contents.
	# configure_file(InputFile OutputFile [COPYONLY] [ESCAPE_QUOTES] [@ONLY])
	SET( source det_by_minor_${com}.${com} )
	CONFIGURE_FILE(
		${CMAKE_CURRENT_SOURCE_DIR}/det_by_minor.c
		${CMAKE_CURRENT_BINARY_DIR}/${source}
		COPYONLY
	)
	ADD_EXECUTABLE( det_by_minor_${com} EXCLUDE_FROM_ALL ${source})
	#
	IF( ${com} STREQUAL cpp )
		# These are C++ compiler flags (may not be valid for C)
		add_cppad_cxx_flags(det_by_minor_${com})
	ENDIF( ${com} STREQUAL cpp )
	#
	# Add target the executes this program
	ADD_CUSTOM_TARGET(check_det_by_minor_${com}
		det_by_minor_${com}
		DEPENDS det_by_minor_${com}
	)
	MESSAGE(STATUS "make check_det_by_minor_${com}: available")

ENDFOREACH(com)

# check_compare_c target
ADD_CUSTOM_TARGET( check_compare_c
	DEPENDS check_det_by_minor_c check_det_by_minor_cpp
)
MESSAGE(STATUS "make check_compare_c: available")

# Add check_compare_c to check depends in parent environment
add_to_list(check_depends check_compare_c)
SET(check_depends "${check_depends}" PARENT_SCOPE)
