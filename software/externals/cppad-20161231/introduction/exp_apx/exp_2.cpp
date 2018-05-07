// $Id: exp_2.cpp 3779 2016-01-01 11:26:11Z bradbell $
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */
// BEGIN C++
# include <cmath>           // define fabs function
# include "exp_2.hpp"       // definition of exp_2 algorithm
bool exp_2(void)
{	double x     = .5;
	double check = 1 + x + x * x / 2.;
	bool   ok    = std::fabs( exp_2(x) - check ) <= 1e-10;
	return ok;
}
// END C++
