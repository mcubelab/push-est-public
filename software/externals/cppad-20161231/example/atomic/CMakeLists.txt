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
# Specifies build type for this directory. Possible values are
# empty, Debug, Release, RelWithDebInfo and MinSizeRel
SET(CMAKE_BUILD_TYPE DEBUG)
#
# eigen_sources and CPPAD_HAS_EIGEN
SET(sources eigen_mat_inv.cpp eigen_cholesky.cpp eigen_mat_mul.cpp)
sources_libs_define(eigen "${sources}" "" HAS_EIGEN)
#
ADD_EXECUTABLE(example_atomic EXCLUDE_FROM_ALL
	${eigen_sources}
	atomic.cpp
	checkpoint.cpp
	extended_ode.cpp
	for_sparse_hes.cpp
	for_sparse_jac.cpp
	forward.cpp
	get_started.cpp
	mat_mul.cpp
	mul_level.cpp
	norm_sq.cpp
	ode.cpp
	reciprocal.cpp
	rev_sparse_hes.cpp
	rev_sparse_jac.cpp
	reverse.cpp
	set_sparsity.cpp
	tangent.cpp
)

# Extra compiler flags
add_cppad_cxx_flags( example_atomic )

# Add the check_example_atomic target
ADD_CUSTOM_TARGET(check_example_atomic
	example_atomic
	DEPENDS example_atomic
)
MESSAGE(STATUS "make check_example_atomic: available")

# add to check check_example_depends in parent environment
add_to_list(check_example_depends check_example_atomic)
SET(check_example_depends "${check_example_depends}" PARENT_SCOPE)
