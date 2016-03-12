#!/usr/bin/python3

import sys, random, math

def normalizeSignal( s, max_value, min_value ):
    overshoot   = 0
    isPositive  = True

    for i in s:
        if i > 0:
            diff = i - max_value
            if diff > overshoot:
                overshoot   = diff
                isPositive  = True
        else:
            diff = -( i - min_value )
            if diff > overshoot:
                overshoot   = diff
                isPositive  = False

    print( overshoot )

    if overshoot > 0:
        scalingFactor = ( max_value / ( overshoot + max_value ) ) if isPositive else ( abs( min_value ) / ( overshoot + abs( max_value ) ) )

        for i in range( 0, len( s ) ):
             s[i] *= scalingFactor

if __name__ == "__main__":

    with open( "/home/ttimon7/Work/c++_workspace/emg_approximator_esn/resources/emg/emgFCR_40000_139999_lpf_6Hz.txt", "r" ) as s1:
        with open( "/home/ttimon7/Work/c++_workspace/emg_approximator_esn/resources/emg/emgFCR_40000_139999_lpf_6Hz_n.txt", "w" ) as d1:
            data = []
            for i in s1:
                data.append( float( i ) * 1000 )

            normalizeSignal( data, 1.0, 0.0 )

            for i in data:
                d1.write( str( i ) + "\n" )

    print( "Done." )
