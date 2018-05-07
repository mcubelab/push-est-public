// $Id: lu_vec_ad.hpp 3804 2016-03-20 15:08:46Z bradbell $
# ifndef CPPAD_EXAMPLE_LU_VEC_AD_HPP
# define CPPAD_EXAMPLE_LU_VEC_AD_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

# include <cppad/cppad.hpp>

namespace CppAD {
	extern CppAD::AD<double> LuVecAD(
		size_t                           n,
		size_t                           m,
		CppAD::VecAD<double>             &Matrix,
		CppAD::VecAD<double>             &Rhs,
		CppAD::VecAD<double>             &Result,
		CppAD::AD<double>                &logdet
	);
}

# endif
