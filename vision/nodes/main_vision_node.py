import rospy
import sensor_msgs.Image as Image


class VisionNode:
    def __init__(self):
        rospy.Subscriber("/zed/zed_node/rgb_raw/image_raw_color", Image, self.process_image, queue_size=1)

    def process_image(self, image):
        print(image.data[0])

    @staticmethod
    def run():
        rospy.spin()


if __name__ == '__main__':
    vision_node = VisionNode()
    vision_node.run()
    rospy.spin()
