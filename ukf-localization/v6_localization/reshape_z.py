# -*- coding: utf-8 -*-

"""
Copyright 2015 Roger R Labbe Jr.
FilterPy library.
http://github.com/rlabbe/filterpy

This is licensed under an MIT license.
"""

import numpy as np

def reshape_z(z, dim_z, ndim):
    """
    Ensure that the measurement vector `z` is reshaped to the correct dimensions.

    Parameters:
    -----------
    z : array-like
        The measurement vector to be reshaped. It can have varying dimensions and will
        be reshaped to match (dim_z, 1).

    dim_z : int
        The expected number of rows in the measurement vector.

    ndim : int
        The desired output dimensionality:
        - 1: Returns a 1D array.
        - 0: Returns a scalar.
        - Other values leave the shape as (dim_z, 1).

    Returns:
    --------
    z : ndarray
        The reshaped measurement vector.

    Raises:
    -------
    ValueError
        If the measurement vector `z` cannot be reshaped to the specified dimensions.
    """

    # Convert z to a 2D array
    z = np.atleast_2d(z)

    # If the number of columns in z equals the expected number of rows, transpose it
    if z.shape[1] == dim_z:
        z = z.T

    # Check if the reshaped z matches the expected dimensions
    if z.shape != (dim_z, 1):
        raise ValueError('z (shape {}) must be convertible to shape ({}, 1)'.format(z.shape, dim_z))

    # If ndim is 1, array to a 1D array
    if ndim == 1:
        z = z[:, 0]

    # If ndim is 0, return a scalar
    if ndim == 0:
        z = z[0, 0]
        
    return z
