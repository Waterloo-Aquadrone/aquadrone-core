# Vision Pipeline
[UML diagram of the vision system pipeline](https://github.com/Waterloo-Aquadrone/aquadrone-core/tree/tv/vision/vision-uml.png)
## Camera Input
The camera frames are published to the `/zed/zed_node/rgb/image_rect_color` topic by the Zed ROS wrapper node.
For instructions on how to install the node, visit [here](https://github.com/stereolabs/zed-ros-wrapper).
## Image Preprocessing
The image preprocessing node is responsible for taking the Image from the camera and preprocessing it for the object detection detection algorithm.

Currently, the node only detects the degree of blur in the frame and publishes the frame for the object detection node if it is below the `blur_threshold`.

Message type published by this node: <a target='_blank' href='https://github.com/Waterloo-Aquadrone/aquadrone-core/blob/dev/aquadrone_msgs/msg/Frame.msg'>Frame</a>
## Object Detection
The object detection runs the trained YOLO neural network on the preprocessed image and outputs an array of `BoundingBoxes`.

Each bounding box stores five attributes about the detected object:
* `class_id` of the detected object (based on the `.names` file)
* `x_center` of the bounding box on the x axis
* `y_center` of the bounding box on the y axis
* `width` of the bounding box
* `height` of the bounding box

Message type published by this node: <a target='_blank' href='https://github.com/Waterloo-Aquadrone/aquadrone-core/blob/dev/aquadrone_msgs/msg/BoundingBoxes.msg'>BoundingBoxes</a>, <a target='_blank' href='https://github.com/Waterloo-Aquadrone/aquadrone-core/blob/dev/aquadrone_msgs/msg/BoundingBox.msg'>BoundingBoxes</a>

**NOTE**: The code works if the weights and the config file of the neural network are placed in `vision/nodes/testdata` subdirectory. This can be changed in lines 20 & 21 in `object_detection_node.py` (and updating launch files so that the value of the `test_data_path` parameter points to your desired location).
## Image Postprocessing
The image post processing node uses the bounding boxes and the camera frame to calculate the center of the detected objects by:
* Cropping the original frame to the bounding box for each object
* Using a center strategy to calculate the center
### Center Strategies
The center strategy classes can be found in `src/vision` directory. Concrete strategy classes are derived from a parent center strategy class.

Currently, only two concrete strategy classes have been implemented: `ColourCenterStrategyClass` and `ContourCenterStrategyClass`. It is expected that more concrete strategy classes tailored to each class will be added.

**NOTE**: All concrete strategy class must be imported in `vision/nodes/center_imports.py` for use in the image post processing node.

Message type published by this node: <a target='_blank' href='https://github.com/Waterloo-Aquadrone/aquadrone-core/blob/dev/aquadrone_msgs/msg/Centers.msg'>Centers</a>, <a target='_blank' href='https://github.com/Waterloo-Aquadrone/aquadrone-core/blob/dev/aquadrone_msgs/msg/Center.msg'>Center</a>
## Depth Sensing
The depth sensing node is yet to be implemented (must use physical Zed camera to test). It is responsible for using the calculated center coordinates to calculate the distance of the detected centers from the submarine (camera).

Message type published by this node: <a target='_blank' href='#'>Depths (to be implemented)</a>, <a target='_blank' href='#'>Depth (to be implemented)</a>
## Testing
There are two launch file that can be used to test the overall pipeline: `test_object_detection_node.launch` and `test_pipeline_with_sim.launch`.

The `test_object_detection_node.launch` file displays the output of the object detection node on the images (bounding boxes). Because the model hasn't been trained yet, the bounding boxes are wrong. But once we have the weights we can replace the existing weights file and check to see how the model is doing using this launch file.

The `test_pipeline_with_sim.launch` file runs the image pre processing and image post processing nodes. The bounding boxes were manually calculated and are sent to image post processing by the object detection simulator node. The test input and output nodes are responsible for reading input and displaying out as obvious from their name.
### Cleaning up after testing
There are comments that indicate which lines to remove and which lines to uncomment once the code can be tested with the actual camera:
* The `test_input_node.py` and `test_output_node.py` will not be needed.
* Once the neural network is trained, the `object_detection_simulator_node.py` is also unneccessary and the `testdata` subdirectory and its content can also be removed. Make sure to include alternative weights and config files!
*  The `frame` message type was created for the sake of `test_input_node.py`. Once it is no longer needed, it can be replaced by the `sensor_msgs/Image` message type.