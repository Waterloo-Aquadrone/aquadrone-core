geometry_msgs/Point position
geometry_msgs/Vector3 pos_variance

geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 vel_variance

# Directly from IMU
geometry_msgs/Quaternion orientation_quat

# Unsure of good meaning for the variance
geometry_msgs/Quaternion orientation_quat_variance

# Calculated from quaternion
geometry_msgs/Vector3 orientation_RPY

# Propogated from quaternion variance
geometry_msgs/Vector3 orientation_RPY_variance

