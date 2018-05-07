# $Id: optional_package.cmake 3803 2016-03-19 05:07:48Z bradbell $
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell
#
# CppAD is distributed under multiple licenses. This distribution is under
# the terms of the
#                     Eclipse Public License Version 1.0.
#
# A copy of this license is included in the COPYING file of this distribution.
# Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
# -----------------------------------------------------------------------------
# optional_package(prefix_variable system_include description)
#
# prefix_variable: (out)
# is a PATH variable that holds the install prefix for this optional package.
#
# system_include: (in)
# If this is true, the include files for this package should be treated as
# system files (no warnings).
#
# description: (in)
# is a description for the install prefix for this optional package.
#
# If package_variable is not set by the cmake command line (or gui),
# it is set to the default value NOTFOUND.
#
# If package_variable is set by the cmake command line, the following is done:
# 1. All the valid include subdirectories are added using INCLUDE_DIRECTORIES.
# 2. All the valid library subdirectories are added using LINK_DIRECTORIES.
# The set of valid include and library directories are determined by
# cmake_install_includedirs and cmakd_install_libdirs respectively.
#
# description: (in)
#
MACRO(optional_package prefix_variable system_include description)
	SET(${prefix_variable} NOTFOUND CACHE PATH "${description}")
	SET(prefix ${${prefix_variable}} )
	MESSAGE(STATUS "${prefix_variable} = ${prefix}")
	IF ( prefix )
		# List of preprocessor include file search directories
		FOREACH(dir ${cmake_install_includedirs})
			IF(IS_DIRECTORY ${prefix}/${dir} )
				IF( ${system_include} )
					INCLUDE_DIRECTORIES( SYSTEM ${prefix}/${dir} )
					MESSAGE(STATUS "    Found SYSTEM ${prefix}/${dir}")
				ELSE( ${system_include} )
					INCLUDE_DIRECTORIES( ${prefix}/${dir} )
					MESSAGE(STATUS "    Found ${prefix}/${dir}")
				ENDIF( ${system_include} )
			ENDIF(IS_DIRECTORY ${prefix}/${dir} )
		ENDFOREACH(dir)
		# Paths in which the linker will search for libraries,
		# only applies to targets created after it is called
		FOREACH(dir ${cmake_install_libdirs})
			IF(IS_DIRECTORY ${prefix}/${dir} )
				LINK_DIRECTORIES( ${prefix}/${dir} )
				MESSAGE(STATUS "    Found ${prefix}/${dir}")
			ENDIF(IS_DIRECTORY ${prefix}/${dir} )
		ENDFOREACH(dir)
	ENDIF ( prefix )
ENDMACRO(optional_package)
