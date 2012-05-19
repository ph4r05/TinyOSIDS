#!/bin/bash

# get current directory 
DIR="$( cd -P "$( dirname "$0" )" && pwd )"; 

# change current directory 
cd "$DIR" 

# clean every dir here
for i in `find . -maxdepth 1 -type d`; do cd $i; make clean; cd $DIR; done;

