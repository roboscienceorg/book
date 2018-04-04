#!/bin/bash

files=`grep -v "^%" Robotics.tex | grep "\\include{" | grep  -o "\{.*\}" | grep -o "\w*"`

echo "
.. Roboscience master file, created by
   sphinx-quickstart on Fri Feb 16 18:55:27 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root \`toctree\` directive.

Introduction to Robotics
#########################

.. toctree::
   :maxdepth: 2
   :caption: Contents:
   :numbered:
" > index.rst

for i in $files
do
    pandoc -f latex -t rst -o ${i}.rst ${i}.tex

echo "   "${i}>> index.rst
done

echo "
Index
======

* :ref:\`genindex\`
* :ref:\`search\`
" >> index.rst
