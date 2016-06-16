#!/bin/bash

CWD=$(pwd)

cd ../ga_ico/sim && ./start -nographics -50 &
cd $CWD
cd ../ga_ico1/sim && ./start -nographics -50 &
wait

cd $CWD

COUNTER=0

while [ $COUNTER -lt 1000 ]; do
	CWD=$(pwd)
	let COUNTER=COUNTER+50;
	cd ../ga_ico/sim && ./loop.sh $COUNTER &
	cd $CWD
	cd ../ga_ico1/sim && ./loop.sh $COUNTER &
	wait

	cd $CWD

	LINE11=$(echo "$a" | sed '2q;d' ../ga_ico/sim/checkpoint.csv);
	LINE12=$(echo "$a" | sed '3q;d' ../ga_ico/sim/checkpoint.csv);
	LINE21=$(echo "$a" | sed '2q;d' ../ga_ico1/sim/checkpoint.csv);
	LINE22=$(echo "$a" | sed '3q;d' ../ga_ico1/sim/checkpoint.csv);

	sed -i -e "2 c $LINE21" ../ga_ico/sim/checkpoint.csv;
	sed -i -e "3 c $LINE22" ../ga_ico/sim/checkpoint.csv;
	sed -i -e "2 c $LINE11" ../ga_ico1/sim/checkpoint.csv;
	sed -i -e "3 c $LINE12" ../ga_ico1/sim/checkpoint.csv;
done

echo "Both finished"
