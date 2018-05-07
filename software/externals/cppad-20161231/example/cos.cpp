// $Id: cos.cpp 3856 2016-12-21 05:51:22Z bradbell $
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

/*
$begin cos.cpp$$
$spell
	cos
$$

$section The AD cos Function: Example and Test$$


$code
$srcfile%example/cos.cpp%0%// BEGIN C++%// END C++%1%$$
$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
# include <cmath>

bool Cos(void)
{	bool ok = true;

	using CppAD::AD;
	using CppAD::NearEqual;
	double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

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
	y[0] = CppAD::cos(x[0]);

	// create f: x -> y and stop tape recording
	CppAD::ADFun<double> f(x, y);

	// check value
	double check = std::cos(x0);
	ok &= NearEqual(y[0] , check, eps99, eps99);

	// forward computation of first partial w.r.t. x[0]
	CPPAD_TESTVECTOR(double) dx(n);
	CPPAD_TESTVECTOR(double) dy(m);
	dx[0] = 1.;
	dy    = f.Forward(1, dx);
	check = - std::sin(x0);
	ok   &= NearEqual(dy[0], check, eps99, eps99);

	// reverse computation of derivative of y[0]
	CPPAD_TESTVECTOR(double)  w(m);
	CPPAD_TESTVECTOR(double) dw(n);
	w[0]  = 1.;
	dw    = f.Reverse(1, w);
	ok   &= NearEqual(dw[0], check, eps99, eps99);

	// use a VecAD<Base>::reference object with cos
	CppAD::VecAD<double> v(1);
	AD<double> zero(0);
	v[zero]           = x0;
	AD<double> result = CppAD::cos(v[zero]);
	check = std::cos(x0);
	ok   &= NearEqual(result, check, eps99, eps99);

	return ok;
}

// END C++
