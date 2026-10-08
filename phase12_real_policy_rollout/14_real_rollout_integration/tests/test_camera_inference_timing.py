import unittest
from unittest.mock import patch,MagicMock
from camera_inference_timing import inference_timing


class CameraTimingTests(unittest.TestCase):
    def test_fresh_start_slow_inference(self):
        value=inference_timing(10.,10.03,10.83)
        self.assertLess(value['camera_age_at_inference_start'],.5)
        self.assertGreater(value['camera_age_at_inference_end'],.5)

    def test_stale_start_and_boundary_reject(self):
        for start in (10.5,10.6):
            self.assertFalse(inference_timing(10.,start,11.)['camera_age_at_inference_start']<.5)

    def test_source_clock(self):
        value=inference_timing(10.,10.03,10.83,source_stamp=100.,ros_start=100.1,ros_end=100.9)
        self.assertAlmostEqual(value['source_age_at_inference_start'],.1)
        self.assertAlmostEqual(value['source_age_at_inference_end'],.9)

    def test_bad_timing(self):
        for args in ((11.,10.,12.),(10.,12.,11.),(float('nan'),11.,12.)):
            with self.assertRaises(ValueError):inference_timing(*args)

    def test_live_predict_handoff_uses_start(self):
        from live_observation import LiveObservation
        observer=LiveObservation.__new__(LiveObservation);observer.url='http://unused'
        response=MagicMock();response.__enter__.return_value=response
        with patch('live_observation.time.monotonic',side_effect=[10.03,10.83]),patch('live_observation.urlopen',return_value=response),patch('live_observation.json.load',return_value={'action':[0.]*7}):
            observer.predict(b'image','openvla',selected_frame_receive=10.)
        self.assertAlmostEqual(observer.last_prediction['selected_prediction_frame_age'],.03)
        self.assertAlmostEqual(observer.last_prediction['camera_age_at_inference_end'],.83)

    def test_live_predict_stale_start_no_request(self):
        from live_observation import LiveObservation
        observer=LiveObservation.__new__(LiveObservation);observer.url='http://unused'
        with patch('live_observation.time.monotonic',return_value=10.5),patch('live_observation.urlopen') as request:
            with self.assertRaises(ValueError):observer.predict(b'image','openvla',selected_frame_receive=10.)
            request.assert_not_called()
