################################################################################
####                                README                                  ####
################################################################################

EMGapproxRBF is a wrapper class, designed to provide ease of use and automation
capabilities - when invoked through the automate.py script - for the lower level
rbf_network implementation.

External dependencies (provided in the includes folder):
    1,  rbf_network.h, rbf_network.cpp

Setup:
    1,  Modify "Makefile.conf" to point to your sources (if necessary)
    2,  Build main.cpp in the src directory to get an executable called "start"
    3,  Modify "automate.py" to fit your needs
    4,  Run the script

Outputs will be named following a naming convention such as: result_code_#.txt,
where "code" will correspond to the network parameters being used when producing
the file, while  "#" will be number having no special meaning at all, it is a
byproduct of the multiprocessing portion of the script.

Example code and further instructions for setting network parameters are
provided in "automate.py".

Note: The script uses a Linux specific program called "tail" to extract error
values from the files generated. Routes in the example code use the format
commonly used by Unix systems.
