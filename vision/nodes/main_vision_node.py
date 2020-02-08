import rospy
import sensor_msgs.Image as Image

class VisionNode:
	def init(self):
		rospy.Subscriber("/zed/zed_node/rgb_raw/image_raw_color", Image, process_image, queue_size=1)

	def process_image(self, image):
		print(image.data[0])

if __name__ == '__main__':
	vision_node = VisionNode()
	vision_node.run()
	rospy.spin()
