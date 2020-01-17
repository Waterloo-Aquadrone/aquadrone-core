geometry_msgs/Point position
geometry_msgs/Vector3 pos_variance

geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 vel_variance


# Quaternion measured directly from IMU
# Unsure of good meaning for the variance of a quaternion
geometry_msgs/Quaternion orientation_quat
geometry_msgs/Quaternion orientation_quat_variance

# RPY values and variance calculated from quaternion
geometry_msgs/Vector3 orientation_RPY
geometry_msgs/Vector3 orientation_RPY_variance

geometry_msgs/Vector3 ang_vel
geometry_msgs/Vector3 ang_vel_variance

