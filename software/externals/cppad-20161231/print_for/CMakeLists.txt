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
# Build the print_for directory tests
# Inherit environment from ../CMakeList.txt

# Specifies build type for this directory. Possible values are
# empty, Debug, Release, RelWithDebInfo and MinSizeRel
SET(CMAKE_BUILD_TYPE DEBUG)

# add_executable(<name> [WIN32] [MACOSX_BUNDLE] [EXCLUDE_FROM_ALL]
#                 source1 source2 ... sourceN
# )
ADD_EXECUTABLE(print_for EXCLUDE_FROM_ALL print_for.cpp)

# Add extra compiler flags
add_cppad_cxx_flags(print_for)

# check_print_for target
ADD_CUSTOM_TARGET(check_print_for print_for DEPENDS print_for)
MESSAGE(STATUS "make check_print_for: available")

# Change check depends in parent environment
add_to_list(check_depends check_print_for)
SET(check_depends "${check_depends}" PARENT_SCOPE)
