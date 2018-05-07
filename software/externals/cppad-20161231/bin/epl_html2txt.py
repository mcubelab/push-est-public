#! /usr/bin/python 
# $Id: epl_html2txt.py 2506 2012-10-24 19:36:49Z bradbell $
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-12 Bradley M. Bell
#
# CppAD is distributed under multiple licenses. This distribution is under
# the terms of the 
#                     GNU General Public License Version 3.
#
# A copy of this license is included in the COPYING file of this distribution.
# Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
# -----------------------------------------------------------------------------
#
import re   # see http://docs.python.org/library/re.html
# --------------------------------------------------------------------------
def remove_simple_cmd(cmd_start, cmd_end, text_in) :
	pattern = '(.*?)' + cmd_start + '(.*?)' + cmd_end
	pattern = re.compile(pattern, re.DOTALL)
	text_out = ''
	start    = 0
	while start < len(text_in):
		match  = pattern.search(text_in, start) 
		if match == None :
			text_out += text_in[start:-1]
			start     = len(text_in)
		else :
			text_out += match.group(1) + match.group(2)
			start     = match.end(0)
	return text_out
# --------------------------------------------------------------------------
def remove_cmd_indent_text(cmd_start, cmd_end, text_in) :
	pattern = '(.*?)' + cmd_start + '(.*?)' + cmd_end
	pattern = re.compile(pattern, re.DOTALL)
	text_out = ''
	start    = 0
	while start < len(text_in):
		match  = pattern.search(text_in, start) 
		if match == None :
			text_out += text_in[start:-1]
			start     = len(text_in)
		else :
			text_out += match.group(1) + '\t'
			text_out += re.sub('\n', '\n\t', match.group(2) )
			start     = match.end(0)
	return text_out
# --------------------------------------------------------------------------
#
file_in  = open('epl-v10.html', 'rb')
data_in  = file_in.read()
# --------------------------------------------------------------------------
# Extract body
pattern  = '<body lang="EN-US">(.*)</body>'
match    = re.search(pattern, data_in, re.DOTALL)
text     = match.group(1)
# --------------------------------------------------------------------------
text = remove_simple_cmd('<b>', '</b>', text)
text = remove_simple_cmd('<p>', '</p>', text)
text = remove_simple_cmd('<h2>', '</h2>', text)
text = remove_cmd_indent_text('<p class="list">', '</p>', text)
# --------------------------------------------------------------------------
data_out = text
file_out = open('epl-v10.txt',  'wb')
file_out.write(data_out)
file_out.close()

