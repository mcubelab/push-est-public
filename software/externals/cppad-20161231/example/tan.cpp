// $Id: tan.cpp 3788 2016-02-09 15:50:06Z bradbell $
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

/*
$begin tan.cpp$$
$spell
	tan
	tan
$$

$section The AD tan Function: Example and Test$$


$code
$srcfile%example/tan.cpp%0%// BEGIN C++%// END C++%1%$$
$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
# include <cmath>
# include <limits>

bool Tan(void)
{	bool ok = true;

	using CppAD::AD;
	using CppAD::NearEqual;
	double eps = 10. * std::numeric_limits<double>::epsilon();

	// domain space vector
	size_t n  = 1;
	double x0 = 0.5;
	CPPAD_TESTVECTOR(AD<double>) x(n);
	x[0]      = x0;

	// declare independent variables and start tape recording
	CppAD::Independent(x);

	// range space vector
	size_t m = 1;
	CPPAD_TESTVECTOR(AD<double>) y(m);
	y[0] = CppAD::tan(x[0]);

	// create f: x -> y and stop tape recording
	CppAD::ADFun<double> f(x, y);

	// check value
	double check = std::tan(x0);
	ok &= NearEqual(y[0] , check,  eps, eps);

	// forward computation of first partial w.r.t. x[0]
	CPPAD_TESTVECTOR(double) dx(n);
	CPPAD_TESTVECTOR(double) dy(m);
	dx[0] = 1.;
	dy    = f.Forward(1, dx);
	check = 1. + std::tan(x0) * std::tan(x0);
	ok   &= NearEqual(dy[0], check, eps, eps);

	// reverse computation of derivative of y[0]
	CPPAD_TESTVECTOR(double)  w(m);
	CPPAD_TESTVECTOR(double) dw(n);
	w[0]  = 1.;
	dw    = f.Reverse(1, w);
	ok   &= NearEqual(dw[0], check, eps, eps);

	// use a VecAD<Base>::reference object with tan
	CppAD::VecAD<double> v(1);
	AD<double> zero(0);
	v[zero]           = x0;
	AD<double> result = CppAD::tan(v[zero]);
	check = std::tan(x0);
	ok   &= NearEqual(result, check, eps, eps);

	return ok;
}

// END C++
