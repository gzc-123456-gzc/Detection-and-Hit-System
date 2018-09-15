#!/usr/bin/env python
# -------------------
import numpy as np


class CustomKalmanFilter:
    """
    Class used to implement a KalmanFilter.
    """
    def __init__(self, init_state, transition_matrix, cov_matrix, measure_state_matrix, measure_noise_cov):
        """
        Initialize variables.

        N = nr. of states
        M = nr. of outputs

        x = state vector
        z = output vector

        :param transition_matrix: NxN matrix used for updating the state
        :param init_state: Nx1 state vector
        :param cov_matrix: NxN matrix representing E{xx'}, where x is the a priori estimation
        :param measure_state_matrix: MxN matrix used to select the output from the state vector
        :param measure_noise_cov: MxM matrix representing E{zz'}
        """
        # current iteration data
        self.state = init_state
        self.transition_matrix = transition_matrix
        self.cov_matrix = cov_matrix
        self.measure_state_matrix = measure_state_matrix
        self.measure_noise_cov = measure_noise_cov
        self.initialized = False

    def predict(self):
        """
        Based on past state vector and the transition matrix, predict current state vector and covariance matrix of the
        state.
        """
        # predict current state
        self.state = self.transition_matrix.dot(self.state)
        # predict current covariance matrix
        self.cov_matrix = self.transition_matrix.dot(self.cov_matrix).dot(self.transition_matrix.T)

    def correct(self, measurement):
        """
        Based on the prediction and actual data measured, correct the current state.
        When the measurement is noisy, the current state will be closer to the predicted state.

        :param measurement: vector containing the measurements
        :return:
        """
        kalman_gain = self.__get_kalman_gain()
        # update current state
        self.state = self.state + kalman_gain.dot(measurement - self.measure_state_matrix.dot(self.state))
        # update covariance matrix
        self.cov_matrix = self.cov_matrix - kalman_gain.dot(self.measure_state_matrix).dot(self.cov_matrix)

    def __get_kalman_gain(self):
        """
        Kalman gain is determined by minimizing the expected value E{xx'}, where x is the current state vector.

        :return: Kalman gain used in the correction step
        """
        kalman_gain = self.cov_matrix.dot(self.measure_state_matrix.T)
        # determine covariance matrix of measurement
        measure_cov = self.measure_state_matrix.dot(self.cov_matrix).dot(self.measure_state_matrix.T) \
                      + self.measure_noise_cov
        kalman_gain = kalman_gain.dot(np.linalg.inv(measure_cov))
        return kalman_gain
