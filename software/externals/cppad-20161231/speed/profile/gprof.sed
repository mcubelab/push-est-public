#! /bin/bash -e
# $Id: gprof.sed.in 2506 2012-10-24 19:36:49Z bradbell $
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-11 Bradley M. Bell
#
# CppAD is distributed under multiple licenses. This distribution is under
# the terms of the
#                     GNU General Public License Version 3.
#
# A copy of this license is included in the COPYING file of this distribution.
# Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
# -----------------------------------------------------------------------------
# remove template information
s/<[^<>]*>//g
s/<[^<>]*>//g
s/<[^<>]*>//g
s/<[^<>]*>//g
s/<[^<>]*>//g
# remove argument information
s/([^()]*)//g
s/([^()]*)//g
# remove names space information
s/[a-zA-Z0-9_]*:://g
s/[a-zA-Z0-9_]*:://g
s/[a-zA-Z0-9_]*:://g
s/[a-zA-Z0-9_]*:://g
