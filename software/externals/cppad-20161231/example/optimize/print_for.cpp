// $Id$
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

/*
$begin optimize_print_for.cpp$$
$spell
	Cpp
$$

$section Example Optimization and Print Forward Operators$$

$code
$srcfile%example/optimize/print_for.cpp%0%// BEGIN C++%// END C++%1%$$
$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

namespace {
	struct tape_size { size_t n_var; size_t n_op; };

	void PrintFor(
		double pos, const char* before, double var, const char* after
	)
	{	if( pos <= 0.0 )
			std::cout << before << var << after;
		return;
	}
	template <class Vector> void fun(
		const std::string& options ,
		const Vector& x, Vector& y, tape_size& before, tape_size& after
	)
	{	typedef typename Vector::value_type scalar;

		// phantom variable with index 0 and independent variables
		// begin operator, independent variable operators and end operator
		before.n_var = 1 + x.size(); before.n_op  = 2 + x.size();
		after.n_var  = 1 + x.size(); after.n_op   = 2 + x.size();

		// Argument to PrintFor is only needed
		// if we are keeping print forward operators
		scalar minus_one = x[0] - 1.0;
		before.n_var += 1; before.n_op += 1;
		if( options.find("no_print_for_op") == std::string::npos )
		{	after.n_var += 1;  after.n_op += 1;
		}

		// print argument to log function minus one, if it is <= 0
		PrintFor(minus_one, "minus_one == ", minus_one , " is <=  0\n");
		before.n_var += 0; before.n_op += 1;
		if( options.find("no_print_for_op") == std::string::npos )
		{	after.n_var += 0;  after.n_op += 1;
		}

		// now compute log
		y[0] = log( x[0] );
		before.n_var += 1; before.n_op += 1;
		after.n_var  += 1; after.n_op  += 1;
	}
}

bool print_for(void)
{	bool ok = true;
	using CppAD::AD;
	using CppAD::NearEqual;
	double eps10 = 10.0 * std::numeric_limits<double>::epsilon();

	// domain space vector
	size_t n  = 1;
	CPPAD_TESTVECTOR(AD<double>) ax(n);
	ax[0] = 1.5;

	// range space vector
	size_t m = 1;
	CPPAD_TESTVECTOR(AD<double>) ay(m);

	for(size_t k = 0; k < 2; k++)
	{	// optimization options
		std::string options = "";
		if( k == 0 )
			options = "no_print_for_op";

		// declare independent variables and start tape recording
		CppAD::Independent(ax);

		// compute function value
		tape_size before, after;
		fun(options, ax, ay, before, after);

		// create f: x -> y and stop tape recording
		CppAD::ADFun<double> f(ax, ay);
		ok &= f.size_var() == before.n_var;
		ok &= f.size_op() == before.n_op;

		// Optimize the operation sequence
		f.optimize(options);
		ok &= f.size_var() == after.n_var;
		ok &= f.size_op() == after.n_op;

		// Check result for a zero order calculation for a different x
		CPPAD_TESTVECTOR(double) x(n), y(m), check(m);
		x[0] = 2.75;
		y    = f.Forward(0, x);
		fun(options, x, check, before, after);
		ok &= NearEqual(y[0], check[0], eps10, eps10);
	}
	return ok;
}
// END C++
