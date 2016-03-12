#!/usr/bin/python3

with open( "emgFCR.txt", "r" ) as s:
    with open( "emgFCR_s.txt", "w" ) as d:

        c = 0;
        for l in s:
            if c > 39999 and c < 140000 :
                d.write( l )

            elif c >= 140000:
                break

            c += 1

print( "Done." )
