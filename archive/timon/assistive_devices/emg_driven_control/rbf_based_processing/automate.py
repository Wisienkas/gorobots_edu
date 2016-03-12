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

    args = ( "/home/ttimon7/Work/c++_workspace/emg_approximator_rbf/start", \
             argv_1, argv_2, argv_3, argv_4, argv_5, argv_6, argv_7, argv_8, \
             argv_9, argv_10, argv_11, argv_12, argv_13, argv_14, argv_15, \
             argv_16, argv_17, argv_18 )

    popen = subprocess.Popen( args, stdout = subprocess.PIPE )
    popen.wait()
    output = popen.stdout.read()

    print( str( output, "utf-8" ) )

def testANN( **kwargs ):
    argv_1  = kwargs["inputPath"]
    argv_2  = kwargs["targetPath"]
    argv_3  = kwargs["resultPath"]
    argv_4  = kwargs["loadPath"]
    argv_5  = str( kwargs.get( "percentage_1", 80.0 ) )
    argv_6  = str( kwargs.get( "percentage_2", 50.0 ) )

    args    = ( "/home/ttimon7/Work/c++_workspace/emg_approximator_rbf/start", \
             argv_1, argv_2, argv_3, argv_4, argv_5, argv_6 )

    popen   = subprocess.Popen( args, stdout = subprocess.PIPE )
    popen.wait()
    output  = popen.stdout.read()

    # print( str( output, "utf-8" ) )

def extractMSE( path ):
    """ Method Description

    Extracts the corrected Mean Square Error (MSE), which is the raw MSE
    measured during testing upscaled to the number of samples being used
    altogether.
    """
    args = ( "tail", path, "-n1" )
    popen = subprocess.Popen( args, stdout = subprocess.PIPE )
    popen.wait()
    output = popen.stdout.read()
    # print( "MSE:", float( str( output, "utf-8" ).split("\t")[1] ) )
    return float( str( output, "utf-8" ).split("\t")[1] )

def selection( currentPath ):
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

    Reads the mse value from n fles selected by the path
    prefix provided, and erases all except the one having
    the lowest value.
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
            os.remove( currentPath )

    # print( previousMSE )

def worker( inputPath, targetPath, resultPath, savePath, nhu, lr, std ):

    trainANN( inputPath = inputPath, targetPath = targetPath, resultPath = resultPath, savePath = savePath,
        numberOfHiddenUnits = nhu, learningRate = lr, min = -0.2, max = 1.2, randomizePosition = False,
        randomizeSTD = False, std = std )

    sys.stdout.flush()

def cleanUp():
    for p in emgProcesses:
        p.terminate()

    del emgProcesses[:]

if __name__ == "__main__":

    pb = ProgressBar( 3*4*3*4*10, title = "", stepsInPercentage = 1.0 )
    counter         = 0

    numberOfHiddenUnits = [ 50, 200, 800, 1500 ]
    learningRate        = [ 0.2, 0.5, 0.9 ]
    std                 = [ 0.1, 0.3, 0.5, 0.8 ]

    temp        = [ "gap_1", "gap_2", "gap_3" ]

    inputPrefix = "resources/synthetic/gap/synthetic_emg_"
    targetPath  = "resources/synthetic/synthetic_emg.txt"
    savePath    = "/home/ttimon7/Work/c++_workspace/emg_approximator_rbf/save_files/def.txt"

    for t in temp:
        inputPath   = inputPrefix + t + ".txt"
        code        = 0

        for n_nhu, nhu in enumerate( numberOfHiddenUnits ):

            for n_lr, lr in enumerate( learningRate ):
                threads = []

                for n_s, s in enumerate( std ):
                    code = ( 100 * n_nhu ) + ( 10 * n_lr ) + ( 1 * n_s )
                    resultPrefix  = "results/validation/synthetic_emg/" + t + "/result_" + str( code ) + "_"

                    cleanUp()

                    for k in range( 1, 11 ):
                        resultPath  = resultPrefix + str( k ) + ".txt"

                        p = multiprocessing.Process(
                            target = worker,
                            args = (
                                inputPath,
                                targetPath,
                                resultPath,
                                savePath,
                                nhu,
                                lr,
                                s )
                            )
                        p.start()
                        emgProcesses.append( p )

                    for p in emgProcesses:
                        p.join()

                    counter += 10
                    pb.updateProgress( counter )
                    selection2( resultPrefix, 10 )

    printi( "readErrors: " + str( readErrors ) )
    printi( "Execution time: ~" + str( time.time() - t_0 ) )


    # Sin - cross-validation
    #
    # gsx = [ "_g_", "_s_", "_x_" ]
    # for i in gsx:
    #
    #     par = [ "0.05_0-4", "0.05_0-6" ]
    #     if i == "_x_": par = [ "6_0.25", "8_0.25" ]
    #
    #     for j in par:
    #         for k in range( 1, 11 ):
    #
    #             inputPath   = "/home/ttimon7/Work/c++_workspace/emg_approximator_esn/resources/synthetic/sin" + i + j + ".txt"
    #             targetPath  = "/home/ttimon7/Work/c++_workspace/emg_approximator_esn/resources/synthetic/sin.txt"
    #             resultPath  = "/home/ttimon7/Work/c++_workspace/emg_approximator_esn/results/sin/" + i + "/result_" + j + "_" + str( k ) + ".txt"
    #
    #             callANN( inputPath = inputPath, targetPath = targetPath, resultPath = resultPath,
    #                 learningRate = 0.995, iterationLimit = 5000, percentage = 30.0 )
    #
    #             selection( resultPath )
    #             sys.stdout.flush()
    #
    #             counter += 1
    #             pb.updateProgress( counter )

    # Sin - testing
    # testANN(
    #     inputPath = "resources/synthetic/sin.txt",
    #     targetPath = "resources/synthetic/sin.txt",
    #     resultPath = "results/testing/def.txt",
    #     loadPath = "save_files/def.txt" )

    printi( "Done." )
