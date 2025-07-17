class KalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimated_value = 0.0
        self.estimated_error = 1.0

    def clear(self):
        self.estimated_value = 0.0
        self.estimated_error = 1.0

    def update(self, measurement):
        # Kalman Gain
        kalman_gain = self.estimated_error / (self.estimated_error + self.measurement_variance)
        # Update the estimated value
        self.estimated_value = self.estimated_value + kalman_gain * (measurement - self.estimated_value)
        # Update the error covariance
        self.estimated_error = (1 - kalman_gain) * self.estimated_error + self.process_variance
        return self.estimated_value