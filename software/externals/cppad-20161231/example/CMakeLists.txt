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
# Build the example directory tests
# Inherit environment from ../CMakeList.txt

# Specifies build type for this directory. Possible values are
# empty, Debug, Release, RelWithDebInfo and MinSizeRel
SET(CMAKE_BUILD_TYPE DEBUG)

# initialize check_depends
SET(check_example_depends example)

# atomic examples
ADD_SUBDIRECTORY(atomic)

# optimize examples
ADD_SUBDIRECTORY(optimize)

# adolc_sources, adolc_libs, and CPPAD_HAS_ADOLC
SET(sources mul_level_adolc.cpp mul_level_adolc_ode.cpp)
sources_libs_define(adolc "${sources}" adolc HAS_ADOLC)

# eigen_sources and CPPAD_HAS_EIGEN
SET(sources eigen_det.cpp eigen_array.cpp)
sources_libs_define(eigen "${sources}" "" HAS_EIGEN)

# colpack_prefix
SET(colpack_sources
	colpack_hes.cpp
	colpack_jac.cpp
)

# ipopt_prefix
IF( ipopt_prefix )
	ADD_SUBDIRECTORY(ipopt_solve)
	add_to_list(check_depends check_example_ipopt_solve)
ENDIF( ipopt_prefix )


# add_executable(<name> [WIN32] [MACOSX_BUNDLE] [EXCLUDE_FROM_ALL]
#                 source1 source2 ... sourceN
# )
ADD_EXECUTABLE(example EXCLUDE_FROM_ALL example.cpp
	${adolc_sources}
	${eigen_sources}
	${colpack_sources}
	abort_recording.cpp
	abs.cpp
	acos.cpp
	acosh.cpp
	ad_assign.cpp
	ad_ctor.cpp
	add.cpp
	add_eq.cpp
	ad_fun.cpp
	ad_in_c.cpp
	ad_input.cpp
	ad_output.cpp
	asin.cpp
	asinh.cpp
	atan2.cpp
	atan.cpp
	atanh.cpp
	azmul.cpp
	base_alloc.hpp
	base_require.cpp
	bender_quad.cpp
	bool_fun.cpp
	bool_sparsity.cpp
	capacity_order.cpp
	change_param.cpp
	check_for_nan.cpp
	check_numeric_type.cpp
	check_simple_vector.cpp
	compare_change.cpp
	compare.cpp
	complex_poly.cpp
	cond_exp.cpp
	conj_grad.cpp
	cos.cpp
	cosh.cpp
	cppad_vector.cpp
	dependency.cpp
	div.cpp
	div_eq.cpp
	equal_op_seq.cpp
	erf.cpp
	error_handler.cpp
	exp.cpp
	expm1.cpp
	for_one.cpp
	for_sparse_hes.cpp
	for_sparse_jac.cpp
	for_two.cpp
	forward.cpp
	forward_dir.cpp
	forward_order.cpp
	fun_assign.cpp
	fun_check.cpp
	hes_lagrangian.cpp
	hes_lu_det.cpp
	hes_minor_det.cpp
	hessian.cpp
	hes_times_dir.cpp
	independent.cpp
	index_sort.cpp
	integer.cpp
	interface2c.cpp
	interp_onetape.cpp
	interp_retape.cpp
	jac_lu_det.cpp
	jac_minor_det.cpp
	jacobian.cpp
	log10.cpp
	log1p.cpp
	log.cpp
	lu_factor.cpp
	lu_invert.cpp
	lu_ratio.cpp
	lu_solve.cpp
	lu_vec_ad.cpp
	lu_vec_ad.hpp
	lu_vec_ad_ok.cpp
	mul.cpp
	mul_eq.cpp
	mul_level.cpp
	mul_level_ode.cpp
	nan.cpp
	near_equal.cpp
	near_equal_ext.cpp
	number_skip.cpp
	numeric_type.cpp
	num_limits.cpp
	ode_err_control.cpp
	ode_err_maxabs.cpp
	ode_gear_control.cpp
	ode_gear.cpp
	ode_stiff.cpp
	ode_taylor.cpp
	opt_val_hes.cpp
	par_var.cpp
	poly.cpp
	pow.cpp
	pow_int.cpp
	print_for.cpp
	reverse_checkpoint.cpp
	reverse_one.cpp
	reverse_three.cpp
	reverse_two.cpp
	rev_one.cpp
	rev_sparse_hes.cpp
	rev_sparse_jac.cpp
	rev_two.cpp
	romberg_mul.cpp
	romberg_one.cpp
	rosen_34.cpp
	runge45_1.cpp
	runge45_2.cpp
	seq_property.cpp
	set_union.cpp
	sign.cpp
	simple_vector.cpp
	sin.cpp
	sinh.cpp
	sparse_hessian.cpp
	sparse_jacobian.cpp
	sparse_sub_hes.cpp
	sparsity_sub.cpp
	sqrt.cpp
	stack_machine.cpp
	sub.cpp
	sub_eq.cpp
	sub_sparse_hes.cpp
	tan.cpp
	tanh.cpp
	tape_index.cpp
	thread_alloc.cpp
	to_string.cpp
	unary_minus.cpp
	unary_plus.cpp
	value.cpp
	var2par.cpp
	vec_ad.cpp
	vector_bool.cpp
)

# Compiler flags for cppad source
add_cppad_cxx_flags(example)

# List of libraries to be linked into the specified target
TARGET_LINK_LIBRARIES(example
	${cppad_lib}
	${adolc_libs}
	${colpack_libs}
)

# Add the check_example target
ADD_CUSTOM_TARGET(check_example example DEPENDS ${check_example_depends})
MESSAGE(STATUS "make check_example: available")

# Change check depends in parent environment
add_to_list(check_depends check_example)
SET(check_depends "${check_depends}" PARENT_SCOPE)
