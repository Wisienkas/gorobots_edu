#!/usr/bin/python3

import sys, random, math

from sysArgvToKwargs import sysArgvToKwargs

from colored_output import BColors as BC
printe = BC.printe
printw = BC.printw
printi = BC.printi
printok = BC.printok

class GapSpikeCrossTalkFactory( object ):

    def __init__( self, operation, src1, **kwargs ):
        self.min_value      = kwargs.get( "min_value", -1.0 )
        self.max_value      = kwargs.get( "max_value", 1.0 )

        self.src1           = src1
        self.src2           = kwargs.get( "other", None )
        self.dst            = kwargs.get( "dst", "gsx.txt" )

        if operation == "g":
            self.gapMaker( self.src1, self.dst, **kwargs )
        elif operation == "s":
            self.spikeAdder( self.src1, self.dst, **kwargs )
        elif operation == "x":
            self.signalIntertwiner( self.src1, self.src2, self.dst, **kwargs )
        elif operation == "n":
            with open( self.src1, "r", encoding="utf-8" ) as s:
                with open( self.dst, "w+", encoding="utf-8" ) as d:
                    data = []
                    for i in s:
                        data.append( float( i ) )

                    self.normalizeSignal( data )

                    for i in data:
                        d.write( str( i ) + "\n" )

    def gapMaker( self, src, dst, **kwargs ):
        with open( src, "r", encoding="utf-8" ) as s:
            with open( dst, "w+", encoding="utf-8" ) as d:

                contamination_ratio = kwargs.get( "contamination_ratio", .05 )
                min_radius          = kwargs.get( "min_radius", 0 )
                max_radius          = kwargs.get( "max_radius", 4 )
                number_of_gaps      = kwargs.get( "number", 70 )

                self.countSamples( s )

                data = []
                for i in s:
                    data.append( float( i ) )

                missing_samples = int( self.sample_count * contamination_ratio )
                gapCounter      = number_of_gaps
                while gapCounter > 0:

                    gapCounter -= 1

                    position    = random.randint( int( self.sample_count * 0.05 ), int( self.sample_count * 0.95 ) )
                    if missing_samples >= ( gapCounter * ( max_radius * 2 + 1 ) ):
                        radius  = max_radius
                    else:
                        radius  = random.randint( min_radius, max_radius )

                    temp1 = ( missing_samples - ( radius * 2 + 1 ) )
                    temp2 = ( gapCounter * ( min_radius * 2 + 1 ) )
                    if temp1 <= temp2:
                        radius = ( missing_samples - temp2 - 1 ) / 2

                    missing_samples -= ( 1 + radius * 2 )

                    counter = 0
                    lower       = int( position - radius )
                    upper       = int( position + radius + 1 )
                    for i in range( lower,  upper ):
                        counter += 1
                        data[i] = 0.0

                d.writelines( str( i ) + "\n" for i in data )

    def spikeAdder( self, src, dst, **kwargs ):
        with open( src, "r", encoding="utf-8" ) as s:
            with open( dst, "w+", encoding="utf-8" ) as d:

                contamination_ratio = kwargs.get( "contamination_ratio", .05 )
                min_radius          = kwargs.get( "min_radius", 0 )
                max_radius          = kwargs.get( "max_radius", 4 )
                number_of_spikes    = kwargs.get( "number", 70 )
                scalingFactor       = kwargs.get( "scalingFactor", 1.0 )

                self.countSamples( s )

                data = []
                for i in s:
                    data.append( float( i ) * scalingFactor )

                added_samples   = int( self.sample_count * contamination_ratio )
                spikeCounter    = number_of_spikes
                while spikeCounter > 0:

                    spikeCounter -= 1

                    position    = random.randint( int( self.sample_count * 0.05 ), int( self.sample_count * 0.95 ) )
                    if added_samples >= ( spikeCounter * ( max_radius * 2 + 1 ) ):
                        radius  = max_radius
                    else:
                        radius  = random.randint( min_radius, max_radius )

                    temp1       = ( added_samples - ( radius * 2 + 1 ) )
                    temp2       = ( spikeCounter * ( min_radius * 2 + 1 ) )
                    if temp1 <= temp2:
                        radius  = ( added_samples - temp2 - 1 ) / 2

                    added_samples -= ( 1 + radius * 2 )

                    counter     = 0.0
                    lower       = int( position - radius )
                    upper       = int( position + radius + 1 )
                    distance    = upper - lower
                    increment   = ( float( self.max_value ) / 2 )

                    for i in range( lower, upper ):
                        counter += 1

                        if counter < ( distance * 0.3 ):
                            y = increment * ( counter / ( distance * 0.3 ) )
                            data[i] += y if data[i] > 0 else -y

                        elif counter > ( distance * 0.7 ):
                            y = increment * ( float( distance - counter ) / ( distance * 0.3 ) )
                            data[i] += y if data[i] > 0 else -y

                        else:
                            # y = increment
                            data[i] = self.max_value if data[i] > 0 else -self.max_value

                        # data[i] += y if data[i] > 0 else -y

                self.thresholdSignal( data )

                d.writelines( str( i ) + "\n" for i in data )

    def signalIntertwiner( self, src1, src2, dst, **kwargs ):
        with open( src1, "r", encoding="utf-8" ) as s1:
            with open( src2, "r", encoding="utf-8" ) as s2:
                with open( dst, "w+", encoding="utf-8" ) as d:

                    attenuation = kwargs.get( "attenuation", 8 )

                    self.countSamples( s1 )

                    data = []
                    for i in s1:
                        data.append( float( i ) )

                    count = 0
                    for i in s2:
                        data[count] += ( float( i ) / attenuation )

                        count += 1

                    if( kwargs.get( "normalize", False ) ):
                        self.normalizeSignal( data )

                    if( kwargs.get( "threshold", True ) ):
                        self.thresholdSignal( data )

                    d.writelines( str( i ) + "\n" for i in data )

    def countSamples( self, s ):
        self.sample_count = 0

        for i in s:
            self.sample_count += 1

        s.seek( 0 )

    def thresholdSignal( self, s ):
        for i in range( 0, len( s ) ):
            if s[i] > self.max_value:
                s[i] = self.max_value
            elif s[i] < self.min_value:
                s[i] = self.min_value

    def normalizeSignal( self, s ):
        overshoot   = 0
        isPositive  = True

        for i in s:
            if i > 0:
                diff = i - self.max_value
                if diff > overshoot:
                    overshoot   = diff
                    isPositive  = True
            else:
                diff = -( i - self.min_value )
                if diff > overshoot:
                    overshoot   = diff
                    isPositive  = False

        if overshoot > 0:
            scalingFactor = ( self.max_value / ( overshoot + self.max_value ) ) if isPositive else ( abs( self.min_value ) / ( overshoot + abs( self.max_value ) ) )

            for i in range( 0, len( s ) ):
                 s[i] *= scalingFactor

if __name__ == "__main__":

    # Because argument 0 is the script's name
    if len( sys.argv ) < 3:
        printe( "You have to provide at least two arguments.")
        print( "\t\tusage: " + ( ( "python " + sys.argv[0] ) if sys.argv[0][0:2] != "./" else sys.argv[0] ) + \
            " operationToPerform pathToSource [pathToOutput pathToOtherSource]" )
        sys.exit(0)

    else:
        options = [
            "min_value",
            "max_value",
            "other",
            "dst",
            "contamination_ratio",
            "min_radius",
            "max_radius",
            "normalize",
            "number",
            "scalingFactor",
            "attenuation",
            "threshold"
        ]

        GapSpikeCrossTalkFactory( sys.argv[1], sys.argv[2], **sysArgvToKwargs( options, index = 2 ) )

    printi( "Done." )
