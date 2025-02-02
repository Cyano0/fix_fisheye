# README: Fix Fisheye Camera Timestamps in ROS2 Bags

When recording data, some topics (especially images) might have a `header.stamp` value of `0`, making it difficult to synchronize with other sensor data. The script fixes this by setting `header.stamp` to the **bag-recorded timestamp**.

Note:
- Only the `header.stamp` field is modified.
- The original metadata structure is preserved.
  
## Features

- [x] Reads a ROS2 bag directory (**not a ****`.db3`**** file**)
- [x] Updates timestamps for specified image topics
- [x] Preserves metadata integrity
- [x] Overwrites the original bag while keeping the structure intact

## Installation & Dependencies

This script requires Python 3 and the ROS2 Python API (`rosbag2_py`).

### Install Dependencies

```sh
pip install rclpy sensor_msgs builtin_interfaces
```

Ensure you have ROS2 installed and sourced in your environment.

## Usage

### 1. Run the Script

In the terminal:
```
python timestamp_fix.py --directory /path/to/rosbags
```

### 2. Select the Rosbag
To select rosbag, add the directory of rosbags in the command line: 
```
python timestamp_fix.py --directory /path/to/rosbags
```


### 3. Select Topics Being Fixed

By default, the script updates timestamps for these fisheye camera topics:

- `/fisheye_image_SN00012`
- `/fisheye_image_SN00013`
- `/fisheye_image_SN00014`

#### To Modify the Topics

Edit this line in the script:

```python
topics_to_fix = ["/fisheye_image_SN00012", "/fisheye_image_SN00013", "/fisheye_image_SN00014"]
```

Change or add new topics as needed.

#### How the Fix Works

1. Reads each message from the bag.
2. Extracts the **bag-recorded timestamp (****`t`****)**.
3. Updates `msg.header.stamp` with `t`:
   ```python
   sec = t // 1_000_000_000  # Convert nanoseconds to seconds
   nsec = t % 1_000_000_000  # Get nanoseconds
   msg.header.stamp = Time(sec=sec, nanosec=nsec)
   ```
4. Writes the corrected messages back to a **temporary bag**.
5. Replaces the **original bag** with the fixed version.

## Troubleshooting

**Error: Bag Not Found**\
Make sure the provided bag **directory** exists and contains ROS2 bag files.

**Error: Serialization Issues**\
If a message cannot be deserialized, a warning will be printed, and the script will continue processing.


