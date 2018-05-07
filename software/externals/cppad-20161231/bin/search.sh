#! /bin/bash -e
# $Id: search.sh 3781 2016-01-18 16:16:22Z bradbell $
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
if [ ! -e "bin/search.sh" ]
then
	echo "bin/search.sh: must be executed from its parent directory"
	exit 1
fi
if [ "$1" == "" ]
then
	echo 'usage: bin/search.sh pattern'
	echo '       Search for pattern in the cppad source code'
	exit 1
fi
pattern="$1"
# -----------------------------------------------------------------------------
list=`bin/ls_files.sh | sed \
	-e '/^configure$/d' \
	-e '/^makefile.in$/d' \
	-e '/\/makefile.in$/d' \
	-e '/^compile/d'`
grep -l "$pattern" $list
# ----------------------------------------------------------------------------
exit 0
