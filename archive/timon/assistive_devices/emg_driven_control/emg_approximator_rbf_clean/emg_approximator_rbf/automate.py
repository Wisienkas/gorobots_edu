#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os, sys, subprocess, time
import multiprocessing

from progress_bar import ProgressBar

from colored_output import BColors as BC
printe = BC.printe
printw = BC.printw
printi = BC.printi
printok = BC.printok


previous        = 100000.0
previousPath    = ""
emgProcesses    = []

readErrors      = 0


def trainANN( **kwargs ):
    """ Method Description
    Call this to run the network in training mode.

    In trining mode 2 things will happen.
        1,  A new network will be instantiated with the parameters given, and
            trined on the first portion of the data selected through the
            parameter 'percentage_1'.
              e.g. if you have a data of 100 timesteps, and you specify
              percentage_1 to be 80.0, than the first 80 temsteps will be used
              for the learning process.
        2,  When the learning is done, the firt 'percentage_2' portion of the
            data not used during the learning process will be used to validate
            the success of the training done.
              e.g. remaining with out previous example, when the first 80
              percent of the data was used for training, if percentage_2 is set
              to 50.0 then the first 10 timesteps of the remaining 20 will be
              used for validation.

    Data format:
        Input data should be provided in a text file, where lines correspond to
        timesteps, and tab separated values in each line corespond to the
        different inputs of the network. If more data is provided than the
        number of input neurons specified, the additional information will be
        discarded.

        The network will output files similar in structure to the input files,
        but here, on the first line, a legend is inserted telling which column
        contains data on what.

    @param inputPath path to the file containing the input data
    @param targetPath path to the file containing the target data
    @param resultPath tells the system where to dump the results
    @param savePath tells the system where to save the network when the save
        method is invoked. (Note: the network will not be saved unless this
        method is manually called.)
    @param numberOfInputs self explanatory
    @param numberOfOutputs self explanatory
    @param numberOfHiddenUnits self explanatory
    @param learningRate self explanatory
    @param min defines the lower bound in one dimension of the interval on
        which the hidden unit's centroids will be placed. (Note: this is used
        collectively for all dimentions.)
    @param max see parameter min above
    @param randomizePosition set to True if the hidden unit's centroids are
        to be placed randomly. False will distribute them evenly.
    @param randomizeSTD Set to True if random STD values are to be used
    @param std if randomizeSTD is set to False an STD value can be given here
        that is used collectively for instancitaing all Gaussians
    @param std_min if randomizeSTD is set to True, with this value, a lower
        bound can be given, which is used during the STD generation process
    @param std_max see parameter std_min above
    @param percentage_1 see training mode description above
    @param percentage_2 see training mode description above
    @param iterationLimit the maximum number of iterations to be allowed during
        training
    """

    argv_1  = kwargs["inputPath"]
    argv_2  = kwargs["targetPath"]
    argv_3  = kwargs["resultPath"]
    argv_4  = kwargs["savePath"]
    argv_5  = str( kwargs.get( "numberOfInputs", 1 ) )
    argv_6  = str( kwargs.get( "numberOfOutputs", 1 ) )
    argv_7  = str( kwargs.get( "numberOfHiddenUnits", 10 ) )
    argv_8  = str( kwargs.get( "learningRate", 0.5 ) )
    argv_9  = str( kwargs.get( "min", -1.2 ) )
    argv_10 = str( kwargs.get( "max", 1.2 ) )
    argv_11 = str( 1 if kwargs.get( "randomizePosition", True ) else 0 )
    argv_12 = str( 1 if kwargs.get( "randomizeSTD", True ) else 0 )
    argv_13 = str( kwargs.get( "std", 0.5 ) )
    argv_14 = str( kwargs.get( "std_min", 0.3 ) )
    argv_15 = str( kwargs.get( "std_max", 0.8 ) )
    argv_16 = str( kwargs.get( "percentage_1", 80.0 ) )
    argv_17 = str( kwargs.get( "percentage_2", 50.0 ) )
    argv_18 = str( kwargs.get( "iterationLimit", 10 ) )

    args = ( "./start", \
             argv_1, argv_2, argv_3, argv_4, argv_5, argv_6, argv_7, argv_8, \
             argv_9, argv_10, argv_11, argv_12, argv_13, argv_14, argv_15, \
             argv_16, argv_17, argv_18 )

    popen = subprocess.Popen( args, stdout = subprocess.PIPE )
    popen.wait()
    output = popen.stdout.read()

    print( str( output, "utf-8" ) )

    sys.stdout.flush()


def testANN( **kwargs ):
    """ Method Description
    Call this to run the network in testing mode.

    In testing mode 2 things will happen:
        1,  Saved network data will be loaded from the file specified in the
            'loadPath' parameter.
        2,  The last porion of the input data specified by the two percentage
            parameters will be used to test the network. A similar output will
            be produces as in the validation case.
              e.g. if the input consists of 100 timesteps, and parameter_1 and
              barameter_2 are both set to 50.0, than the last 25 timesteps will
              be used for testing.

    Other parameters are the same as in the trainANN method.
    """

    argv_1  = kwargs["inputPath"]
    argv_2  = kwargs["targetPath"]
    argv_3  = kwargs["resultPath"]
    argv_4  = kwargs["loadPath"]
    argv_5  = str( kwargs.get( "percentage_1", 80.0 ) )
    argv_6  = str( kwargs.get( "percentage_2", 50.0 ) )

    args    = ( "./start", \
             argv_1, argv_2, argv_3, argv_4, argv_5, argv_6 )

    popen   = subprocess.Popen( args, stdout = subprocess.PIPE )
    popen.wait()
    output  = popen.stdout.read()

    print( str( output, "utf-8" ) )

    sys.stdout.flush()


def extractMSE( path ):
    """ Method Description

    Extracts the Mean Square Error (MSE) value from the end of a file. It uses
    the program 'tail' - available in all Linux distributions - for reading
    from the end of the result files.
    """

    args = ( "tail", path, "-n1" )
    popen = subprocess.Popen( args, stdout = subprocess.PIPE )
    popen.wait()
    output = popen.stdout.read()
    # print( "MSE:", float( str( output, "utf-8" ).split("\t")[1] ) )
    return float( str( output, "utf-8" ).split("\t")[1] )


def selection( currentPath ):
    """ Method Description

    Reads the MSE value from a file, and stores it. When next invoked, doas the
    same for the new file, and compares the MSE value to the stored one, then
    erases the file having the higher error value.
    """

    global previous, previousPath

    current = extractMSE( currentPath )

    # print previous, current
    if current < previous:
        try:
            os.remove( previousPath )
        except:
            pass
        previous        = current
        previousPath    = currentPath

    else:
        os.remove( currentPath )


def selection2( pathPrefix, n ):
    """ Method Description

    Reads the MSE value from n fles selected by the path prefix provided, and
    erases all except the one having the lowest value.
      Note: files should be named following the naming convention used by this
      script (result_code_#.txt, where # denotes a number),
    """

    previousPath    = pathPrefix + str( 1 ) + ".txt"
    previousMSE     = extractMSE( previousPath )

    for i in range( 2, ( n + 1 ) ):
        currentPath = pathPrefix + str( i ) + ".txt"
        currentMSE  = extractMSE( currentPath )

        if currentMSE < previousMSE:
            try:
                os.remove( previousPath )
            except:
                pass
            previousMSE     = currentMSE
            previousPath    = currentPath

        else:
            os.remove( currentPath )


def cleanUp():
    """ Method Description

    Terminates all runnig processes invoked by this script.
    """

    for p in emgProcesses:
        p.terminate()

    del emgProcesses[:]


def getCode( nhu, lr, s ):
    """ Method Description

    Generates a code based on which the used parameters can later be identified
    from the file name.
    """

    numberOfHiddenUnits = [ 50, 200, 800, 1500 ]
    learningRate        = [ 0.2, 0.5, 0.9 ]
    std                 = [ 0.1, 0.3, 0.5, 0.8 ]

    # Get indexes
    n_nhu   = numberOfHiddenUnits.index( nhu )
    n_lr    = learningRate.index( lr )
    n_s     = std.index( s )

    return ( 100 * n_nhu ) + ( 10 * n_lr ) + ( 1 * n_s )


def interpretCode( c ):
    """ Method Description

    Translates the codes on the result_files into network configuration parameters.
    """

    code = str( c )

    numberOfHiddenUnits = [ 50, 200, 800, 1500 ]
    learningRate        = [ 0.2, 0.5, 0.9 ]
    std                 = [ 0.1, 0.3, 0.5, 0.8 ]

    noh = 50
    lr  = 0.2
    s   = 0.1

    if len( code ) > 2 and code[-3] == "1": noh = 200
    elif len( code ) > 2 and code[-3] == "2": noh = 800
    elif len( code ) > 2 and code[-3] == "2": noh = 1500

    if len( code ) > 1 and code[-2] == "1": lr = 0.5
    elif len( code ) > 1 and code[-2] == "2": lr = 0.9

    if code[-1] == "1": s = 0.3
    elif code[-1] == "2": s = 0.5
    elif code[-1] == "3": s = 0.8

    return {
        "numberOfHiddenUnits": noh,
        "learningRate": lr,
        "std": s
    }


if __name__ == "__main__":

    # Measuring execution time
    t_0 = time.time()

    # Creating a progress bar (number of steps corresponding to 100 percent, the title to use on the created window, resolution of the updata)
    pb = ProgressBar( 2*3*2, title = "", stepsInPercentage = 1.0 )
    counter         = 0

    # Defining the parameters to be tested
    numberOfHiddenUnits = [ 50, 200 ]
    learningRate        = [ 0.2, 0.5, 0.9 ]
    std                 = [ 0.1, 0.3 ]

    # Defining inputs to use
    inputs        = [ "1" ]

    inputPrefix = "resources/input_"
    targetPath  = "resources/target.txt"
    savePath    = "save_files/def.txt"

    # Applying different parameter sets
    for t in inputs:
        inputPath   = inputPrefix + t + ".txt"
        code        = 0

        for n_nhu, nhu in enumerate( numberOfHiddenUnits ):

            for n_lr, lr in enumerate( learningRate ):
                threads = []

                for n_s, s in enumerate( std ):
                    code = getCode( nhu, lr, s ) #( 100 * n_nhu ) + ( 10 * n_lr ) + ( 1 * n_s )
                    resultPrefix  = "results/validation/result_" + str( code ) + "_"

                    cleanUp()

                    # Instantiating 10 different networks to minimize the chances of getting false results by hitting a local minimum
                    for k in range( 1, 11 ):
                        resultPath  = resultPrefix + str( k ) + ".txt"

                        # Outsourcing the work to different processes
                        p = multiprocessing.Process(
                            target = trainANN,
                            kwargs = {
                                "inputPath": inputPath,
                                "targetPath": targetPath,
                                "resultPath": resultPath,
                                "savePath": savePath,
                                "numberOfHiddenUnits": nhu,
                                "learningRate": lr,
                                "min": -0.2,
                                "max": 1.2,
                                "randomizePosition": False,
                                "randomizeSTD": False,
                                "std": s,
                                "percentage_1": 33.33,
                                "percentage_2": 50.0,
                                "numberOfInputs": 1
                                })
                        p.start()
                        emgProcesses.append( p )

                    for p in emgProcesses:
                        p.join()

                    counter += 1
                    pb.updateProgress( counter )
                    selection2( resultPrefix, 10 )

    printi( "readErrors: " + str( readErrors ) )
    printi( "Execution time: ~" + str( time.time() - t_0 ) )


    printi( "Done." )
