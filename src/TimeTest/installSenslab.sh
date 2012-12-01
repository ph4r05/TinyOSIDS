#!/bin/bash
CN=$1
if [ "x$CN" == "x" ]; then
	echo "Usage: $0 <number of nodes to update>"
	echo "Usage: $0 <start> <end>"
	exit 1
fi

START=1
END=$2
if [ "x$END" == "x" ]; then
	START=1
	END=$CN
else
	START=$CN
fi

# get current directory 
PWD=`pwd`
DIR="$( cd -P "$( dirname "$0" )" && pwd )"; 
cd $DIR

echo "Compiling..."
make wsn430v14

echo "Building node id versions..."
for ((i=$START; i<=$END; i++)); do make wsn430v14 id,$i; done

echo "Updating firmware..."
for ((i=$START; i<=$END; i++)); do senslab-cli update -x "build/wsn430v14/main.ihex.out-$i" $i; done

echo "DONE"
cd $PWD

