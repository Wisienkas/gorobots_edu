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

        Note: beware, that in case pre-filtering is applied to the input data,
        numbers belonging to the input columns in the files outputted by the
        network will belong to the filtered inputs not the originals.

    Pre-filtering:
        The wrapper class can apply a rolling average filter to the input it
        receives, conforming to the equation (in TeX syntax):

            y_n = (1 - \lambda) \sum_{i = 0}^{n} \lambda^{n-i}x_i

        where 'y' is the output of the filter, 'x' is the input, and \lambda is
        a leakage/discount factor.

    @param inputPath path to the file containing the input data
    @param targetPath path to the file containing the target data
    @param resultPath tells the system where to dump the results
    @param saveDir tells the system which directory to choose for generating the
        save files when backing up the network when the save method is invoked.
        (Note: the network will not be saved unless this method is manually
        called.)
    @param saveNum each time the network is saved multiple files are generated.
        This number is appended to the end of these files to prevent unvanted
        overriding from occurring and provide a mean by which they can be
        identified when multiple networks are saved in the same directory.
    @param numberOfInputs self explanatory
    @param numberOfOutputs self explanatory
    @param numberOfHiddenUnits self explanatory
    @param learningMode switches between RLS and LMS learning.
    @param internalNonlinearity the activation function to be used by the hidden
        units can be set to 'linear', 'sigmoid' (logistic) or 'tanh'
    @param outputNonlinearity the activation function to be used by the output
        units can be set to 'linear', 'sigmoid' (logistic) or 'tanh'
    @param inputSparsity self explanatory
    @param internalSparsity self explanatory
    @param learningRate self explanatory
    @param leak self explanatory
    @param preFiltLeakage the \lambda parameter used in the rolling average
        filter introduced above. If set to negative - e.g. -1 - no filtering is
        done
    @param percentage_1 see training mode description above
    @param percentage_2 see training mode description above
    @param repetition the input provided for learning can be reused multiple
        times for a more extensive learning, the number of repetitions is set
        here
    @param iterationLimit the maximum number of iterations to be allowed during
        training
    """

    argv_1  = kwargs["inputPath"]
    argv_2  = kwargs["targetPath"]
    argv_3  = kwargs["resultPath"]
    argv_4  = kwargs.get( "saveDir", "save_files/" )
    argv_5  = str( kwargs.get( "saveNum", 1 ) )
    argv_6  = str( kwargs.get( "numberOfInputs", 1 ) )
    argv_7  = str( kwargs.get( "numberOfOutputs", 1 ) )
    argv_8  = str( kwargs.get( "numberOfHiddenUnits", 32 ) )
    argv_9  = "1" if str( kwargs.get( "learningMode", "RLS" ) ) == "RLS" else 2
    argv_10 = "0"
    if str( kwargs.get( "internalNonlinearity", "linear" ) ) == "sigmoid":
        argv_10 = "1"
    else:
        argv_10 = "2"
    argv_11 = "0"
    if str( kwargs.get( "outputNonlinearity", "linear" ) ) == "sigmoid":
        argv_11 = "1"
    else:
        argv_11 = "2"
    argv_12 = str( kwargs.get( "inputSparsity", 50.0 ) )
    argv_13 = str( kwargs.get( "internalSparsity", 50.0 ) )
    argv_14 = str( kwargs.get( "learningRate", 0.99 ) )
    argv_15 = str( kwargs.get( "leak", 0.33 ) )
    argv_16 = str( kwargs.get( "preFiltLeakage", -1.0 ) )
    argv_17 = str( kwargs.get( "percentage_1", 80.0 ) )
    argv_18 = str( kwargs.get( "percentage_2", 50.0 ) )
    argv_19 = str( kwargs.get( "repetition", 1 ) )
    argv_20 = str( kwargs.get( "iterationLimit", 4000 ) )

    args = ( "./start", \
             argv_1, argv_2, argv_3, argv_4, argv_5, argv_6, argv_7, argv_8, \
             argv_9, argv_10, argv_11, argv_12, argv_13, argv_14, argv_15, \
             argv_16, argv_17, argv_18, argv_19, argv_20 )

    popen = subprocess.Popen( args, stdout = subprocess.PIPE )
    popen.wait()
    output = popen.stdout.read()

    if len( output ) > 0: print( "trainANN:", str( output, "utf-8" ) )

    sys.stdout.flush()


def testANN( **kwargs ):
    """ Method Description
    Call this to run the network in testing mode.

    In testing mode 2 things will happen:
        1,  Saved network data will be loaded from the file specified by the
            'saveDir' and 'saveNum' parameter cpmbination.
        2,  The last porion of the input data specified by the two percentage
            parameters will be used to test the network. A similar output will
            be produces as in the validation case.
              e.g. if the input consists of 100 timesteps, and parameter_1 and
              barameter_2 are both set to 50.0, than the last 25 timesteps will
              be used for testing.

    Other parameters are the same as in the trainANN method.
    """

    argv_1 = kwargs["inputPath"]
    argv_2 = kwargs["targetPath"]
    argv_3 = kwargs["resultPath"]
    argv_4 = kwargs["saveDir"]
    argv_5 = str( kwargs["saveNum"] )
    argv_6 = str( kwargs.get( "preFiltLeakage", -1.0 ) )
    argv_7 = str( kwargs.get( "percentage_1", 80.0 ) )
    argv_8 = str( kwargs.get( "percentage_2", 50.0 ) )

    args = ( "./start", \
             argv_1, argv_2, argv_3, argv_4, argv_5, argv_6, argv_7, argv_8 )

    popen = subprocess.Popen( args, stdout = subprocess.PIPE )
    popen.wait()
    output = popen.stdout.read()

    if len( output ) > 0: print( "testANN:", str( output, "utf-8" ) )

    sys.stdout.flush()


def extractMSE( path ):
    """ Method Description

    Extracts the Mean Square Error (MSE) value from the end of a file. It uses
    the program 'tail' - available in all Linux distributions - for reading
    from the end of the result files.
    """

    global readErrors

    if os.path.isfile( path ):
        args = ( "tail", path, "-n1" )
        popen = subprocess.Popen( args, stdout = subprocess.PIPE )
        popen.wait()
        output = popen.stdout.read()
        # print( "MSE:", float( str( output, "utf-8" ).split("\t")[1] ), path )

        values = str( output, "utf-8" ).split("\t")

        return float( values[1] )
    else:
        readErrors += 1
        return 1000000.0


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

        # print previous, current
        if currentMSE < previousMSE:
            try:
                os.remove( previousPath )
            except:
                pass
            previousMSE     = currentMSE
            previousPath    = currentPath

        else:
            try:
                os.remove( currentPath )
            except:
                pass

    # print( previousMSE )


def cleanUp():
    """ Method Description

    Terminates all runnig processes invoked by this script.
    """

    for p in emgProcesses:
        p.terminate()

    del emgProcesses[:]


def getCode( nhu, l, isp, itp, lr ):
    """ Method Description

    Generates a code based on which the used parameters can later be identified
    from the file name.
    """

    numberOfHiddenUnits = [ 30, 65, 100, 150, 200 ]
    leak                = [ 0.3, 0.4, 0.5, 0.6 ]
    inputSparsity       = [ 50.0, 55.0, 60.0, 65.0, 70.0 ]
    internalSparsity    = [ 50.0, 55.0, 60.0, 65.0, 70.0 ]
    learningRate        = [ 0.92, 0.94, 0.96, 0.98, 0.99 ]

    # Get indexes
    n_nhu   = numberOfHiddenUnits.index( nhu )
    n_l     = leak.index( l )
    n_isp   = inputSparsity.index( isp )
    n_itp   = internalSparsity.index( itp )
    n_lr    = learningRate.index( lr )

    return ( 10000 * n_nhu ) + ( 1000 * n_l ) + ( 100 * n_isp ) + ( 10 * n_itp ) + ( 1 * n_lr )


def interpretCode( c ):
    """ Method Description

    Translates the codes on the result_files into network configuration
    parameters.
    """

    code = str( c )

    numberOfHiddenUnits = [ 30, 65, 100, 150, 200 ]
    leak                = [ 0.3, 0.4, 0.5, 0.6 ]
    inputSparsity       = [ 50.0, 55.0, 60.0, 65.0, 70.0 ]
    internalSparsity    = [ 50.0, 55.0, 60.0, 65.0, 70.0 ]
    learningRate        = [ 0.92, 0.94, 0.96, 0.98, 0.99 ]

    noh = 30
    l  = 0.3
    ips = 50.0
    its = 50.0
    lr = 0.92

    if len( code ) > 4 and code[-5] == "1": noh = 65
    elif len( code ) > 4 and code[-5] == "2": noh = 100
    elif len( code ) > 4 and code[-5] == "3": noh = 150
    elif len( code ) > 4 and code[-5] == "4": noh = 200

    if len( code ) > 3 and code[-4] == "1": l = 0.4
    elif len( code ) > 3 and code[-4] == "2": l = 0.5
    elif len( code ) > 3 and code[-4] == "3": l = 0.6

    if len( code ) > 2 and code[-3] == "1": ips = 55.0
    elif len( code ) > 2 and code[-3] == "2": ips = 60.0
    elif len( code ) > 2 and code[-3] == "3": ips = 65.0
    elif len( code ) > 2 and code[-3] == "4": ips = 70.0
    elif len( code ) > 2 and code[-3] == "5": ips = 75.0
    elif len( code ) > 2 and code[-3] == "6": ips = 80.0
    elif len( code ) > 2 and code[-3] == "7": ips = 85.0
    elif len( code ) > 2 and code[-3] == "8": ips = 90.0

    if len( code ) > 1 and code[-2] == "1": its = 55.0
    elif len( code ) > 1 and code[-2] == "2": its = 60.0
    elif len( code ) > 1 and code[-2] == "3": its = 65.0
    elif len( code ) > 1 and code[-2] == "4": its = 70.0

    if code[-1] == "1": lr = 0.94
    elif code[-1] == "2": lr = 0.96
    elif code[-1] == "3": lr = 0.98
    elif code[-1] == "4": lr = 0.99

    return {
        "numberOfHiddenUnits": noh,
        "leak": l,
        "inputSparsity": ips,
        "internalSparsity": its,
        "learningRate": lr
    }



if __name__ == "__main__":

    # Measuring execution time
    t_0 = time.time()

    # Creating a progress bar (number of steps corresponding to 100 percent, the title to use on the created window, resolution of the updata)
    pb      = ProgressBar( float( 1*1*1*1*1 ), title = "", stepsInPercentage = 1.0, callback = cleanUp )
    counter = 0

    # Defining the parameters to be tested
    numberOfHiddenUnits = [ 65 ]
    leak                = [ 0.3 ]
    inputSparsity       = [ 50.0 ]
    internalSparsity    = [ 70.0 ]
    learningRate        = [ 0.99 ]

    # Defining inputs to use
    inputs              = [ "1" ]

    inputPrefix = "resources/input_"
    targetPath  = "resources/target.txt"
    saveDir    = "save_files/"

    # Applying different parameter sets
    for t in inputs:
        inputPath   = inputPrefix + t + ".txt"
        code        = 0

        for n_nhu, nhu in enumerate( numberOfHiddenUnits ):

            for n_l, l in enumerate( leak ):

                for n_isp, isp in enumerate( inputSparsity ):

                    for n_itp, itp in enumerate( internalSparsity ):
                        threads = []

                        for n_lr, lr in enumerate( learningRate ):

                            code = getCode( nhu, l, isp, itp, lr )

                            resultPrefix  = "results/validation/result_" + str( code ) + "_"

                            cleanUp()

                            for k in range( 1, 11 ):
                                resultPath  = resultPrefix + str( k ) + ".txt"

                                p = multiprocessing.Process(
                                    target = trainANN,
                                    kwargs = {
                                        "inputPath": inputPath,
                                        "targetPath": targetPath,
                                        "resultPath": resultPath,
                                        "numberOfHiddenUnits": nhu,
                                        "leak": l,
                                        "preFiltLeakage": -1.0,
                                        "inputSparsity": isp,
                                        "internalSparsity": itp,
                                        "learningRate": lr,
                                        "iterationLimit": 10000000,
                                        "percentage_1": 80.0,
                                        "percentage_2": 50.0,
                                        "saveDir": saveDir,
                                        "saveNum": code,
                                        "repetition": 1
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
