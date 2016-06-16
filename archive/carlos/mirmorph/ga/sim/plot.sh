#!/bin/bash

echo "$1"

gnuplot <<- EOF
	set terminal wxt persist 
	set datafile separator ","
	plot "$1" using 1:2 with lines title 'Best fitness', "$1" using 1:3 with lines title 'Fitness mean'
EOF