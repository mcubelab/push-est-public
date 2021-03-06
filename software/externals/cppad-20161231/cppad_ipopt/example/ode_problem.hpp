// $Id: ode_problem.hpp 3804 2016-03-20 15:08:46Z bradbell $
# ifndef CPPAD_CPPAD_IPOPT_EXAMPLE_ODE_PROBLEM_HPP
# define CPPAD_CPPAD_IPOPT_EXAMPLE_ODE_PROBLEM_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */
/*
$begin ipopt_nlp_ode_problem.hpp$$
$spell
	cppad_ipopt_nlp
	Nz
	Ny
	Na
$$

$section ODE Inverse Problem Definitions: Source Code$$
$mindex example$$



$code
$srcfile%cppad_ipopt/example/ode_problem.hpp%0%// BEGIN C++%// END C++%1%$$
$$

$end
------------------------------------------------------------------------------
*/
// BEGIN C++
# include "../src/cppad_ipopt_nlp.hpp"

namespace {
	//------------------------------------------------------------------
	typedef Ipopt::Number Number;
	Number a0 = 1.;  // simulation value for a[0]
	Number a1 = 2.;  // simulation value for a[1]
	Number a2 = 1.;  // simulatioln value for a[2]

	// function used to simulate data
	Number y_one(Number t)
	{	Number y_1 =  a0*a1 * (exp(-a2*t) - exp(-a1*t)) / (a1 - a2);
		return y_1;
	}

	// time points were we have data (no data at first point)
	double s[] = { 0.0,        0.5,        1.0,        1.5,        2.0 };
	// Simulated data for case with no noise (first point is not used)
	double z[] = { 0.0,  y_one(0.5), y_one(1.0), y_one(1.5), y_one(2.0) };
	// Number of measurement values
	size_t Nz  = sizeof(z) / sizeof(z[0]) - 1;
	// Number of components in the function y(t, a)
	size_t Ny  = 2;
	// Number of components in the vectro a
	size_t Na  = 3;

	// Initial Condition function, F(a) = y(t, a) at t = 0
	// (for this particular example)
	template <class Vector>
	Vector eval_F(const Vector &a)
	{	Vector F(Ny);
		// y_0 (t) = a[0]*exp(-a[1] * t)
		F[0] = a[0];
		// y_1 (t) =
		// a[0]*a[1]*(exp(-a[2] * t) - exp(-a[1] * t))/(a[1] - a[2])
		F[1] = 0.;
		return F;
	}
	// G(y, a) =  \partial_t y(t, a); i.e. the differential equation
	// (for this particular example)
	template <class Vector>
	Vector eval_G(const Vector &y , const Vector &a)
	{	Vector G(Ny);
		// y_0 (t) = a[0]*exp(-a[1] * t)
		G[0] = -a[1] * y[0];
		// y_1 (t) =
		// a[0]*a[1]*(exp(-a[2] * t) - exp(-a[1] * t))/(a[1] - a[2])
		G[1] = +a[1] * y[0] - a[2] * y[1];
		return G;
	}
	// H(i, y, a) = contribution to objective at i-th data point
	// (for this particular example)
	template <class Scalar, class Vector>
	Scalar eval_H(size_t i, const Vector &y, const Vector &a)
	{	// This particular H is for a case where y_1 (t) is measured
		Scalar diff = z[i] - y[1];
		return diff * diff;
	}
	// function used to count the number of calls to eval_r
	size_t count_eval_r(void)
	{	static size_t count = 0;
		++count;
		return count;
	}
}
// END C++
# endif
