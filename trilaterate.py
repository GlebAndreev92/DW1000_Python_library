import numpy as np
from scipy import optimize
from math import sqrt

class Trilaterator():
    def trilaterate(self, beacons, distances, estimate):
        return optimize.least_squares(self.residuals,
                                        estimate,
                                        args=(np.array(distances),
                                              np.array(beacons))).x

    def residuals(self, estimate, distances, beacons):
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