#!/usr/bin/python3
import os
import shutil
import rosbag2_py  # Python API for handling ROS2 bags
import rclpy.serialization
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time  # ROS2 message for timestamping

def fix_fisheye_with_bag_time(bag_path):
    """
    Fixes the timestamps of fisheye camera topics in a ROS2 bag by replacing 
    the message's header.stamp with the bag-recorded timestamp.

    Args:
        bag_path (str): Path to the input ROS2 bag directory.
    """
    topics_to_fix = ["/fisheye_image_SN00012", "/fisheye_image_SN00013", "/fisheye_image_SN00014"]

    # Ensure the input bag is a directory, not a .db3 file
    if not os.path.isdir(bag_path):
        print(f"Error: {bag_path} is not a valid ROS2 bag directory.")
        return

    temp_bag = bag_path + "_temp"  # Temporary bag directory

    # Set up storage options for reading and writing bags
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")

    # Initialize reader and writer
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=temp_bag, storage_id="sqlite3"), converter_options)

    # Create the same topics in the output bag
    for topic_metadata in reader.get_all_topics_and_types():
        writer.create_topic(topic_metadata)

    print(f"Processing bag: {bag_path}")

    try:
        # Process each message in the input bag
        while reader.has_next():
            topic, data, t = reader.read_next()  # Read next message

            # If topic needs fixing, modify its timestamp
            if topic in topics_to_fix:
                try:
                    # Deserialize the message
                    msg = rclpy.serialization.deserialize_message(data, Image)
                    
                    # Convert t -> builtin_interfaces/Time and update header.stamp
                    sec = t // 1_000_000_000  # Convert nanoseconds to seconds
                    nsec = t % 1_000_000_000  # Remainder as nanoseconds
                    msg.header.stamp = Time(sec=sec, nanosec=nsec)

                    # Re-serialize the message
                    data = rclpy.serialization.serialize_message(msg)

                except Exception as e:
                    print(f"Warning: Failed to deserialize/serialize message on topic {topic}: {e}")

            # Write the (possibly modified) message to the temp bag
            writer.write(topic, data, t)

    finally:
        # Ensure reader and writer are cleaned up properly
        del writer
        del reader

    print(f"Finished processing. Replacing original bag with fixed version...")

    # Replace the original bag with the fixed version
    try:
        shutil.rmtree(bag_path)  # Remove the original bag directory
        shutil.move(temp_bag, bag_path)  # Move fixed bag in place
        print(f"Bag successfully updated: {bag_path}")
    except Exception as e:
        print(f"Error replacing original bag: {e}")

# Example usage (provide the ROS bag directory, not .db3 file)
fix_fisheye_with_bag_time("/media/prabuddhi/Crucial X9/Human_sensing_bags/Day2/1/1_0")
