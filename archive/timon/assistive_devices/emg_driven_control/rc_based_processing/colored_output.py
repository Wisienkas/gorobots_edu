#!/usr/bin/python3

class BColors:
    """ Class Decription

    Provides colored console output facilities.
    """

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
