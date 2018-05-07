// $Id: extern_value.hpp 3804 2016-03-20 15:08:46Z bradbell $
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

# ifndef CPPAD_TEST_MORE_EXTERN_VALUE_HPP
# define CPPAD_TEST_MORE_EXTERN_VALUE_HPP

template <class Type>
class extern_value  {
private:
	Type value_;
public:
	extern_value(Type value);
	void set(Type value);
	Type get(void);
};

# endif
