#! /bin/bash -e
# $Id: test.sh 2506 2012-10-24 19:36:49Z bradbell $
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-12 Bradley M. Bell
#
# CppAD is distributed under multiple licenses. This distribution is under
# the terms of the 
#                     GNU General Public License Version 3.
#
# A copy of this license is included in the COPYING file of this distribution.
# Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
# -----------------------------------------------------------------------------
./print_for | tee test.log
sed -e '/^Test passes/,\$$d' < test.log > test.1
sed -e '1,/^Test passes/d'   < test.log > test.2
if ! diff test.1 test.2 ; then exit 1 ; fi
