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
# Build the introduction/* directory tests
#
SET(check_introduction_depends "")

ADD_SUBDIRECTORY(exp_apx)
ADD_SUBDIRECTORY(get_started)

# Add the check_introduction target
ADD_CUSTOM_TARGET(check_introduction DEPENDS ${check_introduction_depends})
MESSAGE(STATUS "make check_introduction: available")

# Change check depends in parent environment
add_to_list(check_depends check_introduction)
SET(check_depends "${check_depends}" PARENT_SCOPE)
