// $Id: harmonic_time.hpp 3804 2016-03-20 15:08:46Z bradbell $
# ifndef CPPAD_MULTI_THREAD_HARMONIC_TIME_HPP
# define CPPAD_MULTI_THREAD_HARMONIC_TIME_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

bool harmonic_time(
	double& time_out, double test_time, size_t n_thread, size_t mega_sum);

# endif
