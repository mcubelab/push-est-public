#! /bin/bash -e
# $Id: new_release.sh 3788 2016-02-09 15:50:06Z bradbell $
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
if [ "$0" != 'bin/new_release.sh' ]
then
	echo "bin/new_release.sh: must be executed from its parent directory"
	exit 1
fi
# bash function that echos and executes a command
echo_eval() {
	echo $*
	eval $*
}
# -----------------------------------------------------------------------------
svn_repo="https://projects.coin-or.org/svn/CppAD"
stable_version="20160000"
release='1'
# -----------------------------------------------------------------------------
branch=`git branch | grep '^\*'`
if [ "$branch" != '* master' ]
then
	echo 'new_release.sh: must use master branch version of new_release.sh'
	exit 1
fi
# =============================================================================
# master branch
# =============================================================================
# Make sure local, remote and svn hash codes agree for this stable branch
stable_branch=stable/$stable_version
#
# local hash code
local_hash=`git show-ref $stable_branch | \
	grep "refs/heads/$stable_branch" | \
	sed -e "s| *refs/heads/$stable_branch||"`
#
# remote hash code
remote_hash=`git show-ref $stable_branch | \
	grep "refs/remotes/origin/$stable_branch" | \
	sed -e "s| *refs/remotes/origin/$stable_branch||"`
#
# svn hash code
svn_hash=''
if svn info $svn_repo/$stable_branch >& /dev/null
then
	svn_hash=`svn log $svn_repo/$stable_branch --limit 100 | \
		grep 'end *hash *code:' | head -1 | sed -e 's|end *hash *code: *||'`
fi
#
if [ "$local_hash" == '' ] && [ "$remote_hash" == '' ]
then
	echo "new_release.sh: $stable_branch does not exist"
	echo "Check that master local and remote are the same and then ?"
	echo "	git checkout master"
	echo "	git checkout -b $stable_branch master"
	exit 1
fi
if [ "$local_hash" == '' ] && [ "$remote_hash" != '' ]
then
	echo "new_release.sh: local $stable_branch does not exist."
	echo "	git checkout -b $stable_branch origin/$stable_branch"
	exit 1
fi
#
if [ "$remote_hash" == '' ]
then
	echo "new_release.sh: $stable_branch does not exist ?"
	echo "	git push origin $stable_branch"
	exit 1
fi
if [ "$svn_hash" == '' ]
then
	echo "new_release.sh: svn $stable_branch does not exist."
	echo "Make sure master and $stable_branch have same hash code,"
	echo 'then follow instructions below for push and then svn copy ?'
	echo '	bin/push_git2svn.py trunk'
	echo "	svn copy $svn_repo/trunk \\"
	echo "		$svn_repo/$stable_branch"
	echo "log message: Create $stable_branch from trunk at revision ?"
	exit 1
fi
#
# check local == remote
if [ "$local_hash" != "$remote_hash" ]
then
	echo 'new_release.sh: local and remote differ for this branch'
	echo "local  $stable_branch: $local_hash"
	echo "remote $stable_branch: $remote_hash"
	exit 1
fi
#
# check remote == svn
if [ "$svn_hash" != "$remote_hash" ]
then
	echo 'new_release.sh: remote and svn differ for this branch'
	echo "remote $stable_branch: $remote_hash"
	echo "svn    $stable_branch: $svn_hash"
	echo 'Execute the following command ?'
	echo "	bin/push_git2svn.py $stable_branch"
	exit 1
fi
# -----------------------------------------------------------------------------
# Check that release version does not already exist
#
if svn list $svn_repo/releases | grep "$stable_version.$release"
then
	echo "$svn_repo/releases/$stable_version.$release"
	echo 'already exists. Change the assignment'
	echo "	release=$release"
	echo 'in bin/new_release.sh to a higher release number ?'
	exit 1
fi
#
if git tag --list | grep "$stable_version.$release"
then
	echo 'This git reference tag already exists. Delete old version ?'
	echo "	git tag -d $stable_version.$release"
	echo "	git push --delete origin $stable_version.$release"
	exit 1
fi
# -----------------------------------------------------------------------------
# Make sure master branch does not have uncomitted changes
# before checking out stable branch
list=`git status -s`
if [ "$list" != '' ]
then
	echo "new_release.sh: 'git status -s' is not empty (for master branch)"
	exit 1
fi
# =============================================================================
# stable branch
# =============================================================================
# checkout the stable branch
branch=`git branch | grep '^\*' | sed -e 's|^\* *||'`
if [ "$branch" != "$stable_branch" ]
then
	echo_eval git checkout $stable_branch
fi
# -----------------------------------------------------------------------------
check_one=`bin/version.sh get`
check_two=`grep "cppad-$stable_version\.$release" doc.omh \
	| sed -e 's|cppad-\([0-9.]*\):.*|\1|'`
if [ "$check_one" != "$check_two" ]
then
	echo 'bin/new_release.sh: version number is not correct ?'
	echo "	bin/version.sh set $stable_version.$release"
	echo '	bin/version.sh check'
	echo 'Then commit the changes.'
	exit 1
fi
# -----------------------------------------------------------------------------
# make sure that autotools version of makfiles is up to current version
echo_eval "./build.sh automake"
list=`git status -s`
if [ "$list" != '' ]
then
	echo "new_release.sh: 'git status -s' is not empty"
	echo 'stable branch auto-tools install not up to current version'
	echo 'commit the local changes.'
	exit 1
fi
# -----------------------------------------------------------------------------
read -p 'All checks have passed. More testing or commit release [t/c] ?' \
	response
if [ "$response" != 'c' ]
then
	echo 'Exiting for more testing of stable branch'
	exit 1
fi
# -----------------------------------------------------------------------------
# tag this release
#
echo_eval git tag -a \
	-m \"corresponds $svn_repo/releases/$stable_version.$release\" \
	$stable_version.$release
#
echo_eval git push origin $stable_version.$release
# -----------------------------------------------------------------------------
msg="Creating releases/$stable_version.$release"
rep_stable="$svn_repo/$stable_branch"
rep_release="$svn_repo/releases/$stable_version.$release"
echo_eval svn copy $rep_stable $rep_release -m \"$msg\"
# -----------------------------------------------------------------------------
if [ ! -e build ]
then
	echo_eval mkdir -p build
fi
echo_eval cd build
echo_eval svn checkout $svn_repo/conf conf
#
echo_eval cd conf
#
msg="Update stable and release numbers in projDesc.xml"
echo 'Settting stable and advance release in build/conf/projDesc.xml.'
sed -i projDesc.xml \
-e "/^ *<stable/,/^ *<\/stable/s/[0-9]\{8\}/$stable_version/" \
-e "/^ *<release/,/^ *<\/release/s/[0-9]\{8\}\.[0-9]*/$stable_version.$release/"
#
echo "Use the command the following command to finish the process"
echo "	svn commit -m \"$msg\" build/conf/projDesc.xml"
# =============================================================================
# master branch
# =============================================================================
git checkout master
echo "$0: OK"
exit 0
