#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys, os, time
from threading import Thread, Lock

import tkinter as tk
from tkinter.ttk import *

from colored_output import BColors as BC
printe = BC.printe
printw = BC.printw
printi = BC.printi
printok = BC.printok

class ProgressBar( object ):

    def __init__( self, maximum, **kwargs ):
        self.title              = kwargs.get( "title", "Task" )
        self.maximum            = maximum
        self.progress           = 0
        self.stepsInPercentage  = kwargs.get( "stepsInPercentage", 5.0 )
        self.step               = maximum * ( self.stepsInPercentage / 100.0 )
        self.milestones         = []

        self.console            = kwargs.get( "console", False )
        self.stopFlag           = False
        self.exitOnStop         = kwargs.get( "exitOnStop", False )

        self.callback           = kwargs.get( "callback", False )
        self.callbackArgs       = kwargs.get( "callbackArgs", False )

        s = self.step
        while s < self.maximum:
            self.milestones.append( s )
            s += self.step

        if self.console:
            print( "Progress: [", end = "" )

        self.t                  = Thread( target = self.showProgressBar )
        self.t.demon            = True
        self.t.start()

        # We must wait for the thread to start up, and instantiate the GUI.
        time.sleep( 0.5 )

    def getProgress( self, actual ):
        new_progress = self.progress

        for i in range( self.progress, len( self.milestones ) ):
            if actual >= self.milestones[i]:
                new_progress += 1

            else:
                break

        return new_progress

    def printProgress( self, new_progress ):
        for i in range( self.progress, new_progress ):
            print( "-", end = "" )

            if new_progress > ( len( self.milestones ) - 1 ):
                print( "]" )
                self.stop()

            else:
                self.progress = new_progress

        sys.stdout.flush()

    def showProgressBar( self ):

        self.root = tk.Tk()

        self.root.title( self.title )

        self.root.resizable( width = tk.FALSE, height = tk.FALSE )
        size = ( 400.0, 193.0 )
        w = self.root.winfo_screenwidth()
        h = self.root.winfo_screenheight()
        x = ( w / 2 ) - ( size[0] / 2 )
        y = ( h / 2 ) - ( size[1] / 2 )
        self.root.geometry( "%dx%d+%d+%d" % ( size + ( x, y ) ) )

        separator1 = tk.Frame( height = 50, bd = 1, relief = tk.SUNKEN )
        separator1.pack( fill = tk.X, padx = 5, pady = 5 )

        self.label = tk.Label(
            separator1,
            text    = "Progress: ",
            font    = ( "Helvetica", "12", "bold " ),
            fg      = "grey",
            height  = 2,
            padx    = 1,
            pady    = 1 )
        self.label.pack()

        self.progressbar = tk.ttk.Progressbar( orient = tk.HORIZONTAL, length = 390, mode = "determinate" )
        self.progressbar.pack()

        separator2 = tk.Frame( height = 3, bd = 1, relief = tk.SUNKEN )
        separator2.pack( fill = tk.X, padx = 5, pady = 5 )

        self.b = tk.Button(
            self.root,
            text    = "Cancel",
            font    = ( "Helvetica", "6", "bold " ),
            command = lambda: self.stop( True ),
            width   = 20,
            height  = 1,
            padx    = 1,
            pady    = 5 )
        self.b.pack()

        self.root.mainloop()

    def updateProgress( self, actual ):
        if not self.stopFlag:
            new_progress = self.getProgress( actual )

            difference = new_progress - self.progress
            if ( difference ) > 0:
                if self.console:
                    self.printProgress( new_progress )

                self.progressbar.step( self.stepsInPercentage * difference )
                self.label["text"]              = "Progress: " + (  "%3.1f" % ( new_progress * self.stepsInPercentage ) ) + "%"

                if new_progress >= ( len( self.milestones ) - 1 ):
                    self.label["fg"]            = "green"
                    self.label["text"]          = "COMPLETE"
                    self.progressbar["value"]   = 100.0
                    self.stop()

                else:
                    self.progress = new_progress

            time.sleep( .1 )

    def stop( self, abort = False ):
        self.stopFlag = True

        if self.callback != False:
            if self.callbackArgs != False:
                self.callback( self.callbackArgs )
            else:
                self.callback()

        if self.exitOnStop or abort:
            os._exit( 1 )

if __name__ == "__main__":

    pb = ProgressBar( 400.0, stepsInPercentage = 2.5, exitOnStop = True )

    for a in range(0, 401, 40):
        pb.updateProgress( a )

        sys.stdout.flush()

    printi( "Done." )
