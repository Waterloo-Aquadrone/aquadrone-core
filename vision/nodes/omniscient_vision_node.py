#!/usr/bin/env python3
import rospy
from vision.omniscient_vision import OmniscientVision


if __name__ == "__main__":
    rospy.init_node("omniscient_vision_node")

    omniscient_vision_node = OmniscientVision()
    omniscient_vision_node.run()
