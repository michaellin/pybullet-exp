import numpy as np
import scipy.signal

class lowPassFilter(object):

  def __init__(self, norder, fc, fs, vecSize=1):
    self.b, self.a = scipy.signal.iirfilter(norder, fc,
                                  btype='lowpass',ftype='butter',
                                  fs=fs)
    self.b = np.tile(self.b, (vecSize, 1))
    self.a = np.tile(self.a, (vecSize, 1))

    self.vecSize = vecSize

    self.input = np.zeros((vecSize, norder + 1))
    self.output = np.zeros((vecSize, norder + 1))

  def filter(self, x):
    if (type(x) == np.ndarray):
      if (x.shape[0] != self.vecSize):
        raise TypeError
    else:
      if (self.vecSize != 1):
        raise TypeError
    # shift input by one
    self.input = np.pad(self.input, ((0,0), (1,0)))[:,:-1]
    # place new input as first element
    self.input[:,0] = x

    # shift ouput by one
    self.output = np.pad(self.output, ((0,0),(1,0)))[:,:-1]
    # compute the new output item with IIR filter equation
    self.output[:,0] = self.b.dot(self.input.T) - self.a[:,1:].dot(self.output[:,1:].T) 

    return self.output[0]
