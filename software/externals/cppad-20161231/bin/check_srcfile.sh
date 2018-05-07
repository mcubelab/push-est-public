#! /bin/bash -e
# $Id$
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell
#
# CppAD is distributed under multiple licenses. This distribution is under
# the terms of the
#                     GNU General Public License Version 3.
#
# A copy of this license is included in the COPYING file of this distribution.
# Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
# -----------------------------------------------------------------------------
if [ ! -e "bin/check_srcfile.sh" ]
then
	echo "bin/check_srcfile.sh: must be executed from its parent directory"
	exit 1
fi
cat << EOF > junk.sed
/\$srcfile[^a-z]/! b skip
N
s/^#[ \\t]//
s/^[ \\t]//
s/\\n#[ \\t]//
s/\\n[ \\t]//
s/\$srcfile%//
s/%.*//
p
: skip
EOF
special_case='
bin/check_srcfile.sh
bin/package.sh
cppad/core/cond_exp.hpp
introduction/exp_apx/exp_2.omh
introduction/exp_apx/exp_eps.omh
omh/license.omh
bin/batch_edit.sh
'
# -----------------------------------------------------------------------------
# Make sure that OMhelp srcfile commands refer to same file as command
echo "Checking that OMhelp srcfile commands include from file they appear in."
echo "----------------------------------------------------------------------"
list=`bin/ls_files.sh`
different="no"
for file in $list
do
	ok='no'
	for name in $special_case
	do
		if [ "$file" == "$name" ]
		then
			ok='yes'
		fi
	done
	#
	reference=`sed -n -f junk.sed $file`
	if [ "$reference" == '' ] || [ "$reference" == "$file" ]
	then
		ok='yes'
	fi
	#
	ext=`echo $file | sed -e 's|.*\.||'`
	if [ "$ext" == 'omh' ]
	then
		file_root=`echo $file | sed -e 's|.*/||' -e 's|_hpp\.omh|.hpp|'`
		ref_root=`echo $reference | sed -e 's|.*/||'`
		if [ "$file_root" == "$ref_root" ]
		then
			ok='yes'
		fi
		file_root=`echo $file | sed -e 's|.*/||' -e 's|\.omh|.hpp|'`
		if [ "$file_root" == "$ref_root" ]
		then
			ok='yes'
		fi
	fi
	#
	if [ "$ok" == 'no' ]
	then
		echo "\$srcfile in $file references $reference"
		different="yes"
	fi
done
echo "-------------------------------------------------------------------"
if [ $different = "yes" ]
then
	echo "Error: nothing should be between the two dashed lines above"
	exit 1
else
	echo "OK: nothing is between the two dashed lines above"
	exit 0
fi
