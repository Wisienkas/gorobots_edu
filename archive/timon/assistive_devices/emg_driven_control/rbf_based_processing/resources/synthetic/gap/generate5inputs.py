#!/usr/bin/python3

if __name__ == "__main__":
    for s in range( 1, 4 ):
        with open( "synthetic_emg_gap_" + str( s ) + ".txt", "r" ) as src:
            with open( "synthetic_emg_gap_" + str( s ) + "_5i.txt", "w" ) as dst:

                temp = []
                for d in src:
                    temp.append( d.strip( "\n" ) )

                i = 0
                while ( i + 4 ) < ( len( temp ) ):

                    temp2 = ""
                    for j in range( i, i + 5 ):
                        temp2 += temp[j] + "\t"

                    temp2 += "\n"
                    dst.write( temp2 )

                    i += 1;
