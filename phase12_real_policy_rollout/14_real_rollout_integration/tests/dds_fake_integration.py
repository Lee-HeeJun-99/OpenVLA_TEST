"""Run separately in an isolated local ROS domain; never launch robot driver."""
import os
if os.environ.get('ROS_DOMAIN_ID')!='231' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise RuntimeError('isolated_domain_required')
import sys,time,unittest,threading
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rclpy
from rclpy.executors import MultiThreadedExecutor
from dsr_msgs2.srv import MoveLine,MoveStop,SetToolDigitalOutput
from real_sink import RosServiceTransport

class DDSIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init();cls.server=rclpy.create_node('isolated_fake_doosan');cls.client=rclpy.create_node('isolated_test_client')
        if any(name=='/dsr01/dsr_controller2' for name in [('/'+ns.strip('/')+'/'+n).replace('//','/') for n,ns in cls.client.get_node_names_and_namespaces()]):
            raise RuntimeError('driver_present_in_test_domain')
        cls.mode='success';cls.received=[];cls.services=[]
        def callback(req,res):
            cls.received.append(req)
            if cls.mode=='delay':time.sleep(.2)
            res.success=cls.mode!='failure';return res
        for typ,name in ((MoveLine,'/dsr01/motion/move_line'),(MoveStop,'/dsr01/motion/move_stop'),(SetToolDigitalOutput,'/dsr01/io/set_tool_digital_output')):
            cls.services.append(cls.server.create_service(typ,name,callback))
        cls.executor=MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.server);cls.executor.add_node(cls.client)
        cls.thread=threading.Thread(target=cls.executor.spin);cls.thread.start();cls.transport=RosServiceTransport(cls.client)
    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown();cls.thread.join();cls.client.destroy_node();cls.server.destroy_node();rclpy.shutdown()
    def test_interfaces(self):
        self.__class__.mode='success'
        for name,typ,values in (('/dsr01/motion/move_line','MoveLine',dict(pos=[400.,0.,500.,0.,0.,0.],vel=[20.,20.],acc=[20.,20.],sync_type=0)),
                ('/dsr01/motion/move_stop','MoveStop',dict(stop_mode=3)),
                ('/dsr01/io/set_tool_digital_output','SetToolDigitalOutput',dict(index=1,value=0))):
            self.assertTrue(self.transport.wait(self.transport.send(name,typ,values),2)['success'])
    def test_timeout_and_failure(self):
        self.__class__.mode='delay'
        with self.assertRaises(TimeoutError):self.transport.wait(self.transport.send('/dsr01/motion/move_stop','MoveStop',{'stop_mode':3}),.02)
        time.sleep(.25);self.__class__.mode='failure'
        self.assertFalse(self.transport.wait(self.transport.send('/dsr01/motion/move_stop','MoveStop',{'stop_mode':3}),2)['success'])
    def test_z_server_disappearance(self):
        for service in self.services:self.server.destroy_service(service)
        time.sleep(.1)
        with self.assertRaises(TimeoutError):
            future=self.transport.send('/dsr01/motion/move_stop','MoveStop',{'stop_mode':3})
            self.transport.wait(future,.1)

if __name__=='__main__':unittest.main()
