// $Id: ode.cpp 3794 2016-02-29 20:42:44Z bradbell $
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */
/*
$begin fadbad_ode.cpp$$
$spell
	fadiff
	cassert
	namespace
	fabs
	const
	std
	typedef
	diff
	Jacobian
	cstring
	cppad
	hpp
	onetape
	Fadbad
	bool
	CppAD
$$

$section Fadbad Speed: Ode$$
$mindex link_ode speed$$



$head Specifications$$
See $cref link_ode$$.

$head Implementation$$

$srccode%cpp% */
# include <FADBAD++/fadiff.h>
# include <algorithm>
# include <cassert>
# include <cppad/utility/vector.hpp>
# include <cppad/speed/uniform_01.hpp>
# include <cppad/speed/ode_evaluate.hpp>

// list of possible options
# include <map>
extern std::map<std::string, bool> global_option;

namespace fadbad {
	// define fabs for use by ode_evaluate
	fadbad::F<double> fabs(const fadbad::F<double>& x)
	{	return std::max(-x, x); }
}

bool link_ode(
	size_t                     size       ,
	size_t                     repeat     ,
	CppAD::vector<double>      &x         ,
	CppAD::vector<double>      &jacobian
)
{
	// speed test global option values
	if( global_option["atomic"] )
		return false;
	if( global_option["memory"] || global_option["onetape"] || global_option["optimize"] )
		return false;
	// -------------------------------------------------------------
	// setup
	assert( x.size() == size );
	assert( jacobian.size() == size * size );

	typedef fadbad::F<double>       ADScalar;
	typedef CppAD::vector<ADScalar> ADVector;

	size_t i, j;
	size_t p = 0;          // use ode to calculate function values
	size_t n = size;       // number of independent variables
	size_t m = n;          // number of dependent variables
	ADVector X(n), Y(m);   // independent and dependent variables

	// -------------------------------------------------------------
	while(repeat--)
	{	// choose next x value
		CppAD::uniform_01(n, x);
		for(j = 0; j < n; j++)
		{	// set value of x[j]
			X[j] = x[j];
			// set up for X as the independent variable vector
			X[j].diff(j, n);
		}

		// evaluate function
		CppAD::ode_evaluate(X, p, Y);

		// return values with Y as the dependent variable vector
		for(i = 0; i < m; i++)
		{	for(j = 0; j < n; j++)
				jacobian[ i * n + j ] = Y[i].d(j);
		}
	}
	return true;
}
/* %$$
$end
*/
