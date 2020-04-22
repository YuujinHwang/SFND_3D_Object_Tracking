

# Track an Object in 3D Space

## FP.0 - 4
- implemented in repository

## FP.5 Performance Evaluation 1

### Find lidar-based TTC detection fault

1. Lidar pointscloud classification

<img src = "./build/lidar_3d.png" width = "400">
<img src = "./build/lidar_detect.png" width = "400">
Only clustering lidar points cloud with ROI leads to outlier points into 
TTC calculation. The proper clustering algorithm is needed.

2. Lidar sensing fault

<img src = "./build/lidar_fault.png" width = "400">
<img src = "./build/lidar_fault_ttc.png" width = "400">
Filtered lidar points with ROI only contains outliers, although all the inlier points are disappeared.
That makes TTC very small value, eventhough the car is stopped in traffic.


## FP.6 Performance Evaluation 2

### Find combination of det/desc combination, and find camera-based TTC fault