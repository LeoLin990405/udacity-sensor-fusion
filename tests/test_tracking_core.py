import unittest

import numpy as np

from student.association import Association
from student.filter import Filter
from student.measurements import Measurement, Sensor
from student.trackmanagement import Trackmanagement


class _Transform:
    transform = np.identity(4).reshape(-1).tolist()


class _CameraCalibration:
    extrinsic = _Transform()
    intrinsic = [1000.0, 1000.0, 960.0, 640.0]


def _lidar_measurement(frame, x, y=0.0, z=0.0):
    sensor = Sensor('lidar', None)
    values = [x, y, z, 1.5, 2.0, 4.0, 0.0]
    return Measurement(frame, values, sensor)


class TrackingCoreTests(unittest.TestCase):
    def test_constant_velocity_prediction_matrices(self):
        kf = Filter()
        transition = kf.F()
        process_noise = kf.Q()

        self.assertEqual(transition.shape, (6, 6))
        self.assertEqual(process_noise.shape, (6, 6))
        np.testing.assert_allclose(process_noise, process_noise.T)
        self.assertGreaterEqual(np.linalg.eigvalsh(process_noise).min(), -1e-12)

    def test_lidar_update_reduces_position_residual(self):
        manager = Trackmanagement()
        manager.init_track(_lidar_measurement(1, 10.0))
        track = manager.track_list[0]
        track.x[0] = 8.0
        measurement = _lidar_measurement(2, 10.0)
        kf = Filter()

        before = abs(float(kf.gamma(track, measurement)[0]))
        kf.update(track, measurement)
        after = abs(float(kf.gamma(track, measurement)[0]))

        self.assertLess(after, before)

    def test_nearest_neighbor_assignment_is_one_to_one(self):
        manager = Trackmanagement()
        manager.init_track(_lidar_measurement(1, 10.0))
        manager.init_track(_lidar_measurement(1, 20.0))
        measurements = [_lidar_measurement(2, 20.1), _lidar_measurement(2, 10.1)]
        association = Association()
        association.associate(manager.track_list, measurements, Filter())

        pairs = [association.get_closest_track_and_meas() for _ in range(2)]

        self.assertEqual(set(pairs), {(0, 1), (1, 0)})
        self.assertEqual(association.unassigned_tracks, [])
        self.assertEqual(association.unassigned_meas, [])

    def test_track_manager_deletes_missed_unconfirmed_track(self):
        manager = Trackmanagement()
        measurement = _lidar_measurement(1, 10.0)
        manager.init_track(measurement)

        manager.manage_tracks([0], [], [measurement])

        self.assertEqual(manager.N, 0)
        self.assertEqual(manager.track_list, [])

    def test_camera_projection_and_field_of_view(self):
        camera = Sensor('camera', _CameraCalibration())
        state = np.matrix([[10.0], [1.0], [2.0], [0.0], [0.0], [0.0]])

        projection = camera.get_hx(state)
        jacobian = camera.get_H(state)

        self.assertTrue(camera.in_fov(state))
        np.testing.assert_allclose(projection, [[860.0], [440.0]])
        self.assertEqual(jacobian.shape, (2, 6))


if __name__ == '__main__':
    unittest.main()
