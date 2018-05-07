// $Id: multi_newton.hpp 3804 2016-03-20 15:08:46Z bradbell $
# ifndef CPPAD_MULTI_THREAD_MULTI_NEWTON_HPP
# define CPPAD_MULTI_THREAD_MULTI_NEWTON_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

extern bool multi_newton(
	CppAD::vector<double> &xout                ,
	void fun(double x, double& f, double& df)  ,
	size_t num_sub                             ,
	double xlow                                ,
	double xup                                 ,
	double epsilon                             ,
	size_t max_itr                             ,
	size_t num_threads
);

# endif
