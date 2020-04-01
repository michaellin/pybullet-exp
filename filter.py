import numpy as np
import scipy.signal

class lowPassFilter(object):

  def __init__(self, norder, fc, fs):
    self.b, self.a = scipy.signal.iirfilter(norder, fc,
                                  btype='lowpass',ftype='butter',
                                  fs=fs)

    self.input = np.zeros(norder + 1)
    self.output = np.zeros(norder + 1)
    print("b {}, a {}".format(self.b, self.a))

  def filter(self, x):
    # shift input by one
    self.input = np.pad(self.input, (1,0))[:-1]
    # place new input as first element
    self.input[0] = x

    # shift ouput by one
    self.output = np.pad(self.output, (1,0))[:-1]
    # compute the new output item with IIR filter equation
    self.output[0] = self.b.dot(self.input) - self.a[1:].dot(self.output[1:]) 

    return self.output[0]
