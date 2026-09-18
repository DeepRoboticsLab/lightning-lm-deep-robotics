# Versioned, synchronized SLAM input, after sensor point filtering and before
# deskewing. Arrays preserve the estimator's exact float/double values and order.
# One message per consumed scan, including IMU initialization and skipped LiDAR
# corrections. No raw rejected points, images, TF, maps or visualization data.
uint32 version
uint64 sequence
float64 lidar_begin_time
float64 lidar_end_time
# Four float32 values per point: x, y, z, intensity (LiDAR frame).
float32[] xyzi
# One float64 per point: offset from scan begin in milliseconds.
float64[] point_time_ms
# Seven float64 per sample: timestamp seconds, ax, ay, az, gx, gy, gz.
float64[] imu
