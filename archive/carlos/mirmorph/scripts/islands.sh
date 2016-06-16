#!/bin/bash

CWD=$(pwd)

cd ../ga/sim && ./start -nographics -50 &
cd $CWD
cd ../ga1/sim && ./start -nographics -50 &
wait

cd $CWD

COUNTER=0

while [ $COUNTER -lt 1000 ]; do
	CWD=$(pwd)
	let COUNTER=COUNTER+50;
	cd ../ga/sim && ./loop.sh $COUNTER &
	cd $CWD
	cd ../ga1/sim && ./loop.sh $COUNTER &
	wait

	cd $CWD

	LINE11=$(echo "$a" | sed '2q;d' ../ga/sim/checkpoint.csv);
	LINE12=$(echo "$a" | sed '3q;d' ../ga/sim/checkpoint.csv);
	LINE21=$(echo "$a" | sed '2q;d' ../ga1/sim/checkpoint.csv);
	LINE22=$(echo "$a" | sed '3q;d' ../ga1/sim/checkpoint.csv);

	sed -i -e "2 c $LINE21" ../ga/sim/checkpoint.csv;
	sed -i -e "3 c $LINE22" ../ga/sim/checkpoint.csv;
	sed -i -e "2 c $LINE11" ../ga1/sim/checkpoint.csv;
	sed -i -e "3 c $LINE12" ../ga1/sim/checkpoint.csv;
done

echo "Both finished"