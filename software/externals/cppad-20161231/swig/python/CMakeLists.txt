# $Id$
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
# used debug during initial development of swig interface
SET(CMAKE_BUILD_TYPE DEBUG)

assert( SWIG_FOUND )
FIND_PACKAGE(PythonInterp)
FIND_PACKAGE(PythonLibs)
# ----------------------------------------------------------------------------
IF ( PythonLibs_FOUND AND PythonInterp_FOUND)
	# which version of python did we find
	SET(lib    "${PYTHONLIBS_VERSION_STRING}" )
	SET(interp "${PYTHON_VERSION_STRING}" )
	IF( NOT ( "${lib}" STREQUAL "${interp}" ) )
		MESSAGE(FATAL_ERROR,
			"cmake python library ${lib} not same as interpertor ${interp}"
		)
	ENDIF( NOT ( "${lib}" STREQUAL "${interp}" ) )

	# treat like a C++ files
	SET_SOURCE_FILES_PROPERTIES(
		${swig_control_file} PROPERTIES CPLUSPLUS ON
	)

	# swig flags when processing this file
	SET_SOURCE_FILES_PROPERTIES(
		${swig_control_file} PROPERTIES SWIG_FLAGS "-Wall"
	)

	# include directory determined by find PythonLibs
	INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

	# SWIG_ADD_MODULE(name language [ files ])
	SWIG_ADD_MODULE(cppad_swig python ${swig_control_file} empty.cpp)

	MESSAGE(STATUS "swig_generated_sources=${swig_generated_sources}")

	# SWIG_LINK_LIBRARIES(name [ libraries ])
	SWIG_LINK_LIBRARIES(cppad_swig cppad_swig_lib ${PYTHON_LIBRARIES})

	# copy test program to directory with swig module is located
	ADD_CUSTOM_TARGET(check_swig_py
		${CMAKE_COMMAND} -E copy
		${CMAKE_CURRENT_SOURCE_DIR}/check_swig.py
		${CMAKE_CURRENT_BINARY_DIR}/check_swig.py
		DEPENDS
		${CMAKE_CURRENT_SOURCE_DIR}/check_swig.py
	)

	# check_swig_python
	ADD_CUSTOM_TARGET(check_swig_python
		python check_swig.py
		DEPENDS ${SWIG_MODULE_cppad_swig_REAL_NAME} check_swig_py
	)
	MESSAGE(STATUS "make check_swig_python: available")

	# Change check depends in parent environment
	add_to_list(check_swig_depends check_swig_python)
	SET(check_swig_depends "${check_swig_depends}" PARENT_SCOPE)

ENDIF ( PythonLibs_FOUND AND PythonInterp_FOUND)
