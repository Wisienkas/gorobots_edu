#!/usr/bin/python3

class BColors:
    """ Class Decription

    Container for ANSI Escape Characters
    """

    # '\033[1;30mGray like Ghost\033[1;m'
    # '\033[1;31mRed like Radish\033[1;m'
    # '\033[1;32mGreen like Grass\033[1;m'
    # '\033[1;33mYellow like Yolk\033[1;m'
    # '\033[1;34mBlue like Blood\033[1;m'
    # '\033[1;35mMagenta like Mimosa\033[1;m'
    # '\033[1;36mCyan like Caribbean\033[1;m'
    # '\033[1;37mWhite like Whipped Cream\033[1;m'
    # '\033[1;38mCrimson like Chianti\033[1;m'
    # '\033[1;41mHighlighted Red like Radish\033[1;m'
    # '\033[1;42mHighlighted Green like Grass\033[1;m'
    # '\033[1;43mHighlighted Brown like Bear\033[1;m'
    # '\033[1;44mHighlighted Blue like Blood\033[1;m'
    # '\033[1;45mHighlighted Magenta like Mimosa\033[1;m'
    # '\033[1;46mHighlighted Cyan like Caribbean\033[1;m'
    # '\033[1;47mHighlighted Gray like Ghost\033[1;m'
    # '\033[1;48mHighlighted Crimson like Chianti\033[1;m'
    HEADER      = '\033[95m'
    WARNING     = '\033[93m'
    ERROR       = '\033[91m'
    OKBLUE      = '\033[94m'
    OKGREEN     = '\033[92m'
    FAIL        = '\033[91m'
    BOLD        = '\033[1m'
    UNDERLINE   = '\033[4m'
    ENDC        = '\033[0m'

    @staticmethod
    def colorText( text, **kwargs ):
        color = kwargs.get( "color", "" )
        bold  = BColors.BOLD if kwargs.get( "bold", False ) else ""

        return bold + color + text + BColors.ENDC

    @staticmethod
    def printc( text, **kwargs ):
        color = kwargs.get( "color", "" )
        bold  = BColors.BOLD if kwargs.get( "bold", False ) else ""

        print( bold + color + text + BColors.ENDC )

    @staticmethod
    def printok( text ):
        OK_HEADER     = "[" + BColors.BOLD + BColors.OKGREEN + "OK" + BColors.ENDC + "]      "

        print( OK_HEADER + text )

    @staticmethod
    def printi( text ):
        INFO_HEADER     = "[" + BColors.BOLD + BColors.OKBLUE + "INFO" + BColors.ENDC + "]    "

        print( INFO_HEADER + text )

    @staticmethod
    def printw( text ):
        WARNING_HEADER  = "[" + BColors.BOLD + BColors.WARNING + "WARNING" + BColors.ENDC + "] "

        print( WARNING_HEADER + text )

    @staticmethod
    def printe( text ):
        ERROR_HEADER    = "[" + BColors.BOLD + BColors.ERROR + "ERROR" + BColors.ENDC + "]   "

        print( ERROR_HEADER + text )
