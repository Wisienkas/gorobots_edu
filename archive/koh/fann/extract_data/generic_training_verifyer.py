#!/usr/bin/python
# -*- coding: utf-8 -*-

""" Use training data and trained fann to plot training data for comparison
    with nn output.
"""

## New style (python-3 like) print function
from __future__ import print_function
from __future__ import division

from numpy import zeros, fromiter

import pylab
from pylab import plot

import sys

import pyfann


#######################################################         Read in data

print("Reading data")
# Expecting training data in a file called "training_data"
training = open("training", 'r')

# First line is no. of samples, no. input and no. output neurons
num_samples, num_in, num_out = fromiter(training.readline().split(), dtype=int)

inputs  = zeros([num_in,  num_samples])
outputs = zeros([num_out, num_samples])

for i in xrange(num_samples):
    inputs[ :, i] = fromiter(training.readline().split(), dtype=float)
    outputs[:, i] = fromiter(training.readline().split(), dtype=float)


#########################################################         Create NN nnet

print("Applying fann nn")
ann = pyfann.libfann.neural_net()
if not ann.create_from_file('training.net'):
    print ( "failed to create neural network" )
    sys.exit(1)

nn_out  = zeros([num_out, num_samples])

################################################################         Save data

f = open("Input_TrainOutput_Outputnet.csv", "w")
for i in xrange( num_samples ):
    nn_out[:, i]  = ann.run(inputs[:, i])
    for each in inputs[:, i]:
    	print( each, "\t", end='', file=f )
    for each in outputs[:, i]:
    	print( each, "\t", end='', file=f )
    for each in nn_out[:, i]:
    	print( each, "\t", end='', file=f )
    print( "", file=f )
f.close()

################################################################         Plot

print("plotting")
figure = pylab.figure()

for i in xrange(num_in):
    plot(inputs[i, :], label="input " + str(i))

for i in xrange(num_out):
    plot(outputs[i, :], label="output " + str(i))
    plot(nn_out[ i, :], label="nn_out " + str(i))


pylab.title("Trained network over training data")
pylab.xlabel("samples")
pylab.ylabel("[a.u.]")
pylab.legend(loc="upper left", bbox_to_anchor=(0.94, 1.0))

pylab.show()
