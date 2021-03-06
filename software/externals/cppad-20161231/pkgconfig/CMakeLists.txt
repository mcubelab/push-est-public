# $Id: CMakeLists.txt 3608 2015-01-23 12:20:38Z bradbell $
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell
#
# CppAD is distributed under multiple licenses. This distribution is under
# the terms of the 
#                     GNU General Public License Version 3.
#
# A copy of this license is included in the COPYING file of this distribution.
# Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
# -----------------------------------------------------------------------------
# Construct cppad.pc from cppad.pc.in; see
# http://people.freedesktop.org/~dbn/pkg-config-guide.html
#
# Setting that are always included
SET(cppad_pkgconfig_cflags "-I${cppad_abs_includedir}")
SET(cppad_pkgconfig_libs "")
SET(cppad_pkgconfig_requires "")
#
# Settings that are only included if cppad is configured with ipopt.
# (Note that ipopt has a pkgconfig file.)
IF( ipopt_prefix )
	SET(cppad_pkgconfig_requires "ipopt")
	SET(cppad_pkgconfig_libs "-L${cppad_abs_libdir} -lcppad_ipopt")
ENDIF( ipopt_prefix )
#
# Settings that are only included if cppad is configured with colpack
# (Note that colpack does not have a pkgconfig file.)
IF( colpack_prefix )
	SET(cppad_pkgconfig_libs "${cppad_pkgconfig_libs} -lColPack")
	FOREACH(dir ${cmake_install_libdirs})
		IF(IS_DIRECTORY ${colpack_prefix}/${dir} )
			SET(cppad_pkgconfig_libs 
				"${cppad_pkgconfig_libs} -L${colpack_prefix}/${dir}"
			)
		ENDIF(IS_DIRECTORY ${colpack_prefix}/${dir} )
	ENDFOREACH(dir)
	FOREACH(dir ${cmake_install_includedirs})
		IF(IS_DIRECTORY ${colpack_prefix}/${dir} )
			SET(cppad_pkgconfig_cflags 
				"${cppad_pkgconfig_cflags} -I${colpack_prefix}/${dir}"
			)
		ENDIF(IS_DIRECTORY ${colpack_prefix}/${dir} )
	ENDFOREACH(dir)
ENDIF( colpack_prefix )
#
MESSAGE(STATUS "cppad_pkgconfig_cflags = ${cppad_pkgconfig_cflags}")
MESSAGE(STATUS "cppad_pkgconfig_libs = ${cppad_pkgconfig_libs}")
MESSAGE(STATUS "cppad_pkgconfig_requires = ${cppad_pkgconfig_requires}")
#
# Copy a file to another location and modify its contents.
# configure_file(InputFile OutputFile [COPYONLY] [ESCAPE_QUOTES] [@ONLY])
CONFIGURE_FILE(
	${CMAKE_CURRENT_SOURCE_DIR}/cppad.pc.in             
	${CMAKE_CURRENT_BINARY_DIR}/cppad.pc 
)
CONFIGURE_FILE(
	${CMAKE_CURRENT_SOURCE_DIR}/cppad-uninstalled.pc.in 
	${CMAKE_CURRENT_BINARY_DIR}/cppad-uninstalled.pc
)
# During install copy cppad.pc to CMAKE_INSTALL_PREFIX/
# install(FILES files... DESTINATION <dir>
#	[PERMISSIONS permissions...]
#	[CONFIGURATIONS [Debug|Release|...]]
#	[COMPONENT <component>]
#	[RENAME <name>] [OPTIONAL])
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/cppad.pc DESTINATION ${cppad_abs_datadir}/pkgconfig )

