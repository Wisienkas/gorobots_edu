#!/bin/bash

COUNTER=0
while [ $COUNTER -lt 1 ]; do
	./start -nographics -$1 -checkpoint && let COUNTER=COUNTER+1
done