// $Id: poly.cpp 3794 2016-02-29 20:42:44Z bradbell $
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */
/*
$begin sacado_poly.cpp$$
$spell
	onetape
	cppad
	cpp
	tadiff
	ddp
	Taylor
	dz
	eval
	cppad
	vector Vector
	typedef
	sacado
	Lu
	CppAD
	det
	hpp
	const
	bool
	Tay
	resize
	Coeff
$$

$section Sacado Speed: Second Derivative of a Polynomial$$
$mindex link_poly speed$$


$head Specifications$$
See $cref link_poly$$.

$head Implementation$$


$srccode%cpp% */
# include <cppad/utility/vector.hpp>
# include <cppad/utility/poly.hpp>
# include <cppad/speed/uniform_01.hpp>
# include <Sacado.hpp>

// list of possible options
# include <map>
extern std::map<std::string, bool> global_option;

bool link_poly(
	size_t                     size     ,
	size_t                     repeat   ,
	CppAD::vector<double>     &a        ,  // coefficients of polynomial
	CppAD::vector<double>     &z        ,  // polynomial argument value
	CppAD::vector<double>     &ddp      )  // second derivative w.r.t z
{
	if( global_option["atomic"] )
		return false;
	if( global_option["memory"] || global_option["onetape"] || global_option["optimize"] )
		return false;
	// -----------------------------------------------------
	// setup
	typedef Sacado::Tay::Taylor<double>  ADScalar;
	CppAD::vector<ADScalar>              A(size);

	size_t i;               // temporary index
	ADScalar   Z;           // domain space AD value
	ADScalar   P;           // range space AD value
	unsigned int order = 2; // order of Taylor coefficients
	Z.resize(order+1, false);
	P.resize(order+1, false);

	// choose the polynomial coefficients
	CppAD::uniform_01(size, a);

	// AD copy of the polynomial coefficients
	for(i = 0; i < size; i++)
		A[i] = a[i];

	// ------------------------------------------------------
	while(repeat--)
	{	// get the next argument value
		CppAD::uniform_01(1, z);

		// independent variable value
		Z.fastAccessCoeff(0)   = z[0]; // argument value
		Z.fastAccessCoeff(1)   = 1.;   // first order coefficient
		Z.fastAccessCoeff(2)   = 0.;   // second order coefficient

		// AD computation of the dependent variable
		P = CppAD::Poly(0, A, Z);

		// second derivative is twice second order Taylor coefficient
		ddp[0] = 2. * P.fastAccessCoeff(2);
	}
	// ------------------------------------------------------
	return true;
}
/* %$$
$end
*/
