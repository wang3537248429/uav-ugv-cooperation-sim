"""Estimator/flight-controller frame and uncertainty boundaries."""

import numpy as np


def map_flu_to_ned(pose):
    flip = np.diag([1., -1., -1.])
    ned = np.eye(4)
    ned[:3, 3] = flip @ pose[:3, 3]
    ned[:3, :3] = flip @ pose[:3, :3] @ flip
    return ned


def navigation_covariance(rmse):
    covariance = np.zeros(36)
    covariance[[0, 7, 14]] = max(0.0025, float(rmse) ** 2)
    # ArduCopter 4.5.7 combines all three angular diagonal entries into
    # one angErr for ExternalNav yaw fusion. Huge RP entries also suppress
    # yaw; do not reuse the truth baseline's 'ignore RP' covariance here.
    covariance[[21, 28, 35]] = 0.0025
    return covariance.tolist()
