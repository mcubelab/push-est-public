// $Id: atomic.cpp 3830 2016-09-18 11:16:05Z bradbell $
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

// system include files used for I/O
# include <iostream>

// C style asserts
# include <cassert>

// for thread_alloc
# include <cppad/utility/thread_alloc.hpp>

// external complied tests
extern bool checkpoint(void);
extern bool eigen_cholesky(void);
extern bool eigen_mat_inv(void);
extern bool eigen_mat_mul(void);
extern bool extended_ode(void);
extern bool for_sparse_hes(void);
extern bool for_sparse_jac(void);
extern bool forward(void);
extern bool get_started(void);
extern bool mat_mul(void);
extern bool mul_level(void);
extern bool norm_sq(void);
extern bool ode(void);
extern bool reciprocal(void);
extern bool rev_sparse_hes(void);
extern bool rev_sparse_jac(void);
extern bool reverse(void);
extern bool set_sparsity(void);
extern bool tangent(void);

namespace {
	// function that runs one test
	static size_t Run_ok_count    = 0;
	static size_t Run_error_count = 0;
	bool Run(bool TestOk(void), const char *name)
	{	bool ok = true;
		ok &= TestOk();
		if( ok )
		{	std::cout << "OK:    " << "atomic: " << name << std::endl;
			Run_ok_count++;
		}
		else
		{	std::cout << "Error: " << "atomic: " << name << std::endl;
			Run_error_count++;
		}
		return ok;
	}
}

// main program that runs all the tests
int main(void)
{	bool ok = true;

	// This line is used by test_one.sh

	// external compiled tests
	ok &= Run( checkpoint,          "checkpoint"     );
	ok &= Run( extended_ode,        "extended_ode"   );
	ok &= Run( for_sparse_hes,      "for_sparse_hes" );
	ok &= Run( for_sparse_jac,      "for_sparse_jac" );
	ok &= Run( forward,             "forward"        );
	ok &= Run( get_started,         "get_started"    );
	ok &= Run( mat_mul,             "mat_mul"        );
	ok &= Run( mul_level,           "mul_level"      );
	ok &= Run( norm_sq,             "norm_sq"        );
	ok &= Run( ode,                 "ode"            );
	ok &= Run( reciprocal,          "reciprocal"     );
	ok &= Run( rev_sparse_hes,      "rev_sparse_hes" );
	ok &= Run( rev_sparse_jac,      "rev_sparse_jac" );
	ok &= Run( reverse,             "reverse"        );
	ok &= Run( set_sparsity,        "set_sparsity"   );
	ok &= Run( tangent,             "tangent"        );
# ifdef CPPAD_HAS_EIGEN
	ok &= Run( eigen_cholesky,      "eigen_cholesky" );
	ok &= Run( eigen_mat_inv,       "eigen_mat_inv"  );
	ok &= Run( eigen_mat_mul,       "eigen_mat_mul"  );
# endif
	// check for errors
	using std::cout;
	using std::endl;
	assert( ok || (Run_error_count > 0) );
	if( CppAD::thread_alloc::free_all() )
	{	Run_ok_count++;
		cout << "OK:    " << "No memory leak detected" << endl;
	}
	else
	{	ok = false;
		Run_error_count++;
		cout << "Error: " << "memory leak detected" << endl;
	}
	// convert int(size_t) to avoid warning on _MSC_VER systems
	if( ok )
		cout << "All " << int(Run_ok_count) << " tests passed." << endl;
	else	cout << int(Run_error_count) << " tests failed." << endl;

	return static_cast<int>( ! ok );
}
