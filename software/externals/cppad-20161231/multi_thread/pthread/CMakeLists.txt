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
# Build the multi_thread/pthread directory tests
# Inherit build type from ../CMakeList.txt

# Local include directories to search (not in package_prefix/include)
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/.. )

# add_executable(<name> [WIN32] [MACOSX_BUNDLE] [EXCLUDE_FROM_ALL]
#                 source1 source2 ... sourceN
# )
ADD_EXECUTABLE(multi_thread_pthread EXCLUDE_FROM_ALL ../thread_test.cpp
	../../speed/src/microsoft_timer.cpp
	../multi_newton_time.cpp
	../multi_newton.cpp
	../multi_newton_work.cpp
	../team_example.cpp
	../harmonic_time.cpp
	../harmonic.cpp
	../harmonic_work.cpp
	a11c_pthread.cpp
	simple_ad_pthread.cpp
	team_pthread.cpp
)

# Compiler flags for cppad source
add_cppad_cxx_flags( multi_thread_pthread )

# Paths in which the linker will search for libraries,
# only applies to targets created after it is called
LINK_DIRECTORIES( ${pthread_lib_path} )

# List of libraries to be linked into the specified target
TARGET_LINK_LIBRARIES(multi_thread_pthread ${pthread_lib})

# Add the check_multi_thread_pthread target
ADD_CUSTOM_TARGET(check_multi_thread_pthread
	multi_thread_pthread simple_ad
	DEPENDS multi_thread_pthread
)
MESSAGE(STATUS "make check_multi_thread_pthread: available")

# Change check depends in parent environment
add_to_list(check_multi_thread_depends check_multi_thread_pthread)
SET(check_multi_thread_depends "${check_multi_thread_depends}" PARENT_SCOPE)
