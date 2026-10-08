import unittest
from validate_clean_readiness import sample_hardware_ready

class SinglePredictionPreflightTests(unittest.TestCase):
    def hardware(self):
        return {k:{'response':{field:value,'success':True}}
                for k,field,value in [('mode','robot_mode',1),('state','robot_state',1),('system','robot_system',0)]}
    def test_ready(self):
        self.assertTrue(sample_hardware_ready(self.hardware()))
    def test_safe_off_blocks(self):
        h=self.hardware();h['state']['response']['robot_state']=3
        self.assertFalse(sample_hardware_ready(h))
    def test_failed_response_blocks(self):
        h=self.hardware();h['state']['response']['success']=False
        self.assertFalse(sample_hardware_ready(h))
    def test_virtual_system_blocks(self):
        h=self.hardware();h['system']['response']['robot_system']=1
        self.assertFalse(sample_hardware_ready(h))
    def test_missing_is_not_ready(self):
        self.assertFalse(sample_hardware_ready({}))
