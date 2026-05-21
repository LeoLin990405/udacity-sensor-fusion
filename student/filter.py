# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        self.dim_state = params.dim_state

    def F(self):
        F = np.matrix(np.identity(params.dim_state))
        F[0, 3] = params.dt
        F[1, 4] = params.dt
        F[2, 5] = params.dt
        return F

    def Q(self):
        dt = params.dt
        q = params.q
        q1 = q * dt**4 / 4
        q2 = q * dt**3 / 2
        q3 = q * dt**2
        return np.matrix([
            [q1, 0, 0, q2, 0, 0],
            [0, q1, 0, 0, q2, 0],
            [0, 0, q1, 0, 0, q2],
            [q2, 0, 0, q3, 0, 0],
            [0, q2, 0, 0, q3, 0],
            [0, 0, q2, 0, 0, q3],
        ])

    def predict(self, track):
        F = self.F()
        track.set_x(F * track.x)
        track.set_P(F * track.P * F.transpose() + self.Q())

    def update(self, track, meas):
        H = meas.sensor.get_H(track.x)
        gamma = self.gamma(track, meas)
        S = self.S(track, meas, H)
        K = track.P * H.transpose() * np.linalg.inv(S)
        I = np.identity(params.dim_state)
        track.set_x(track.x + K * gamma)
        track.set_P((I - K * H) * track.P)
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        return meas.z - meas.sensor.get_hx(track.x)

    def S(self, track, meas, H):
        return H * track.P * H.transpose() + meas.R
