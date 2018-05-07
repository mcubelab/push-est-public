#! /bin/bash -e
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
if [ ! -e "bin/check_makefile.sh" ]
then
	echo "bin/check_makefile.sh: must be executed from its parent directory"
	exit 1
fi
# -----------------------------------------------------------------------------
echo "Checking debugging flags in all CMakeLists.txt files."
echo "-------------------------------------------------------"
list=`bin/ls_files.sh | sed -n -e '/CMakeLists.txt$/p'`
ok="yes"
for file in $list
do
	case $file in

		# no build type cases
		CMakeLists.txt | \
		cppad/CMakeLists.txt | \
		cppad_ipopt/example/CMakeLists.txt | \
		cppad_ipopt/speed/CMakeLists.txt | \
		cppad_ipopt/src/CMakeLists.txt | \
		cppad_ipopt/test/CMakeLists.txt | \
		cppad_lib/CMakeLists.txt | \
		introduction/CMakeLists.txt | \
		multi_thread/bthread/CMakeLists.txt | \
		multi_thread/openmp/CMakeLists.txt | \
		multi_thread/pthread/CMakeLists.txt | \
		pkgconfig/CMakeLists.txt | \
		speed/adolc/CMakeLists.txt | \
		speed/cppad/CMakeLists.txt | \
		speed/double/CMakeLists.txt | \
		speed/example/CMakeLists.txt | \
		speed/fadbad/CMakeLists.txt | \
		speed/profile/CMakeLists.txt | \
		speed/sacado/CMakeLists.txt | \
		speed/src/CMakeLists.txt )
		check="CMAKE_BUILD_TYPE"
		if grep -i "$check" $file > /dev/null
		then
			echo "'$check' appears in $file"
			ok='no'
		fi
		;;

		# release build type cases
		compare_c/CMakeLists.txt | \
		cppad_ipopt/CMakeLists.txt | \
		speed/CMakeLists.txt )
		check="CMAKE_BUILD_TYPE RELEASE"
		if ! grep "$check" $file > /dev/null
		then
			echo "'$check' does not appear in $file"
			ok='no'
		fi
		;;

		# debug build type cases
		example/CMakeLists.txt | \
		example/atomic/CMakeLists.txt | \
		example/ipopt_solve/CMakeLists.txt | \
		example/optimize/CMakeLists.txt | \
		introduction/exp_apx/CMakeLists.txt | \
		introduction/get_started/CMakeLists.txt | \
		multi_thread/CMakeLists.txt | \
		print_for/CMakeLists.txt | \
		swig/CMakeLists.txt | \
		swig/example/CMakeLists.txt | \
		swig/python/CMakeLists.txt | \
		swig/octave/CMakeLists.txt | \
		test_more/CMakeLists.txt )
		check="CMAKE_BUILD_TYPE DEBUG"
		if ! grep "$check" $file > /dev/null
		then
			echo "'$check' does not appear in $file"
			ok='no'
		fi
		;;

		# unknown CMakeLists.txt file
		*)
		echo "$file is not known to check_cmake.sh"
		ok='no'
		;;

	esac
done
echo "-------------------------------------------------------"
if [ "$ok" = "yes" ]
then
	echo "Ok: nothing is between the two dashed lines above"
	exit 0
else
	echo "Error: nothing should be between the two dashed lines above"
	exit 1
fi
