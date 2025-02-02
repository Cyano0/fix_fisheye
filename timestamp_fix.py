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
        print(f"Skipping {bag_path}: Not a valid ROS2 bag directory.")
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


def process_directory(directory="."):
    """
    Processes all ROS2 bag directories in the given directory.

    Args:
        directory (str): Path to the directory containing ROS2 bags. Defaults to the current directory.
    """
    print(f"Searching for ROS2 bags in: {directory}")

    # Ensure directory exists
    if not os.path.exists(directory):
        print(f"Error: Specified directory '{directory}' does not exist.")
        return

    # Find all directories that contain ROS2 bag data (ignoring _temp folders)
    for item in os.listdir(directory):
        item_path = os.path.join(directory, item)
        if os.path.isdir(item_path) and not item.endswith("_temp"):
            # Check if this directory is a valid ROS2 bag
            if any(f.endswith(".db3") for f in os.listdir(item_path)):  # ROS2 bags have .db3 files inside
                fix_fisheye_with_bag_time(item_path)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Fix fisheye timestamps in all ROS2 bags in a directory.")
    parser.add_argument(
        "-d", "--directory", type=str, default=".", help="Path to the directory containing ROS2 bags (default: current directory)."
    )
    args = parser.parse_args()

    process_directory(args.directory)
