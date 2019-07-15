import numpy as np
import matplotlib.pyplot as plt
from sd_recieve import read_mag_file
from mpl_toolkits.mplot3d import Axes3D
from ellipsoid_fit import ellipsoid_fit as ellipsoid_fit, data_regularize


def calibration_ellipsoid(file='magcalib.txt'):

    data = read_mag_file(file)
    data = data_regularize(data)

    center, evecs, radii = ellipsoid_fit(data)

    a, b, c = radii
   #r = (a * b * c) ** (1. / 3.)
    D = np.array([[1/a, 0., 0.], [0., 1/b, 0.], [0., 0., 1/c]])
  
    transformation = evecs@D@evecs.T

    return center.T,transformation

