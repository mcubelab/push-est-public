// $Id: adolc_alloc_mat.hpp 3804 2016-03-20 15:08:46Z bradbell $
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */
# ifndef CPPAD_SPEED_ADOLC_ADOLC_ALLOC_MAT_HPP
# define CPPAD_SPEED_ADOLC_ADOLC_ALLOC_MAT_HPP

double** adolc_alloc_mat(size_t m, size_t n);
void adolc_free_mat(double** mat);

# endif
