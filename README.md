# Gyroflow to CSV

Convert your [Gyroflow](https://gyroflow.xyz) stabilization to a CSV file.

Make sure to choose "including processed gyro data" when saving your project in Gyroflow. By default, Gyroflow saves out the rotations as Euler rotation (ZYX). It also saves out the data in your footages' native frame rate.

## CLI Script

```
positional arguments:
  gyroflow_path         The path to your Gyroflow file

optional arguments:
  -h, --help            show this help message and exit
  -j, --json_export     Output a JSON file instead of a CSV
  -s, --smoothed        Calculate from smoothed quaternions
  -q, --quaternions     Save data as quaternions
  -a, --all_timestamps  Save all timestamps instead of FPS converted
```

You can later use the [Vonk](https://docs.google.com/document/d/1U9WfdHlE1AZHdU6_ZQCB1I2nSa5I7TyHG2vKMi2I7v8/edit?usp=sharing) data nodes inside of Fusion (you find it in Reactor) to read the CSV and drive a 3D camera with it. Download the example project if you want to see how it's done.
