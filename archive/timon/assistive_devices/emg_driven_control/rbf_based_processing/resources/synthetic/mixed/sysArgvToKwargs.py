#!/use/bin/python3

import sys

from colored_output import BColors as BC
printe = BC.printe
printw = BC.printw
printi = BC.printi
printok = BC.printok

def sysArgvToKwargs( options, **kwargs ):
    """ Method Description

    Packs the argv parameters into kwarg format provided it is given in a manner
    making it possible, and the parameter names are in the options list. e.g.:

        python app.py name1=.16 name1=apple name1 24

            will be parced to:

        kwargs[name1] = float( .16 )
        kwargs[name2] = 'apple'
        kwargs[name3] = int( 24 )

    If name is not in options it will not be processed.

    [OPTIONAL] The first index number of parameters will be ignored.
               (not counting argv[0])
    [OPTIONAL] required should be set to True in case it is a problem if no user
               supplied parameters were detected
    """

    index       = kwargs.get( "index", 1 )
    isRequired  = kwargs.get( "required", False )

    if len( sys.argv ) > ( index + 1 ):
        previous = ""

        kwargs = {}
        for current in sys.argv[( index + 1 ):]:

            if "=" in current:
                previous = ""
                temp = current.split("=")

                if temp[0] in options:
                    if temp[1] == "":
                        printw( "You have provided an empty string as parameter: " + temp[0] + "=" )

                    if "." in temp[1] and temp[1].replace( ".", "" ).isdigit():
                        kwargs[temp[0]] = float( temp[1] )
                    elif temp[1].isdigit():
                        kwargs[temp[0]] = int( temp[1] )
                    elif temp[1] == "False" or temp[1] == "false":
                        kwargs[temp[0]] = False
                    elif temp[1] == "True" or temp[1] == "true":
                        kwargs[temp[0]] = True
                    else:
                        kwargs[temp[0]] = temp[1]

                else:
                    printe( "Parameter option " + temp[0] + " is not recognized." )
                    print( "\t\tAvailable choices are: " + str( options ) )
                    sys.exit( 1 )

            else:
                if previous == "":
                    if current in options:
                        previous = current

                    else:
                        printe( "Parameter option " + current + " is not recognized." )
                        print( "\t\tAvailable choices are: " + str( options ) )
                        sys.exit( 1 )
                else:
                    kwargs[previous] = current
                    previous = ""

        return kwargs

    elif not isRequired:
        return {}

    else:
        printe( "The application is expecting arguments, but none were provided." )
        print( "\t\tAvailable choices are: " + str( options ) )
        sys.exit( 1 )
