"""@package Trilaterator

This module provides a class for trilateration from n position and range measurements.
"""

import numpy as np
from scipy import optimize
from math import sqrt

class Trilaterator():
    def trilaterate(self, beacons, distances, estimate):
        """
        This function takes some beacon position and ranges which are used to
        estimate the position of the ranged entity.

        Args:
            beacons: List of beacon positions, itself lists
            distances: List of distance measurements to beacons
            estimate: Initial position estimate

        Returns:
            Estimated position
        """
        return optimize.least_squares(self.residuals,
                                        estimate,
                                        args=(np.array(distances),
                                              np.array(beacons))).x

    def residuals(self, estimate, distances, beacons):
        """
        Residual function for trilaterate()

        Args:
            estimate: Current position estimate
            distances: List of distances
            beacons: List of beacons
        """
        s = np.empty(distances.shape[0])
        for i in range(0, distances.shape[0]):
            s[i] = distances[i] - np.linalg.norm(estimate - beacons[i])
        return s

if __name__ == "__main__":
    tri = Trilaterator()
    beacons = [[0.0, 0.0, 0.0],
               [1.0, 0.0, 0.0],
               [0.0, 1.0, 0.0],
               [1.0, 1.0, 0.0]]

    distances = [0.7, 0.7, 0.7, 0.7]
    estimate = beacons[0]
    estimate = tri.trilaterate(beacons, distances, estimate)
    print(estimate)
