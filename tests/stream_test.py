#!/usr/bin/env python
import select
import sys
import threading
import time
import traceback
import unittest

import rosbag
import rospy
import rostest
import sensor_msgs.msg
import v4l2capture


PKG = 'virtual_cam'
NAME = 'stream_test'


class TestStream(unittest.TestCase):
    
    def setUp(self):
        super(TestStream, self).setUp()
        
        self.bag = rosbag.Bag(rospy.get_param('bag_file'))
        self.addCleanup(self.bag.close)
        self.frame_count = sum(1 for _ in self.bag.read_messages())
        
        self.stop = threading.Event()
        self.addCleanup(self.stop.set)

        self.play_count = 0
        self.play_thd = threading.Thread(target=self._play)
        self.play_thd.daemon = True
        
        self.capture_count = 0
        self.capture_thd = threading.Thread(target=self._capture)
        self.capture_thd.daemon = True
    
    def _play(self):
        try:
            pub = rospy.Publisher(rospy.get_param('topic'), sensor_msgs.msg.Image)
            bag = rosbag.Bag(rospy.get_param('bag_file'))
            for topic, message, ts in bag.read_messages():
                rec_ts, cur_ts = ts, time.time()
                pub.publish(message)
                self.play_count += 1
                if self.stop.is_set():
                    break
            bag.close()
        except:
            traceback.print_exc()
            raise
    
    def _capture(self):
        video = v4l2capture.Video_device('/dev/video1')
        try:
            video.set_format(640, 480)
            video.create_buffers(60)
            video.queue_all_buffers()
            video.start()
            while not self.stop.is_set():
                ready, _, _ = select.select((video,), (), (), 1.0)
                if ready:
                    video.read_and_queue()
                    self.capture_count += 1
                    continue
                if self.play_count >= self.frame_count:
                    break
        except:
            traceback.print_exc()
            raise
        finally:
            video.close()
    
    def test_stream(self):
        for thd in [self.capture_thd, self.play_thd]:
            thd.start()
        for thd in [self.capture_thd, self.play_thd]:
            thd.join()
        self.assertEqual(self.play_count, self.frame_count)
        self.assertGreater(self.capture_count, int(self.frame_count * 0.75))


if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestStream)
