# Keyframe message - bundles a point cloud with its pose
# Used to communicate keyframes between odometry estimator and local map manager

sensor_msgs/PointCloud2 cloud    # The point cloud data
geometry_msgs/Pose pose           # The pose when this keyframe was created
