import rosbag2_py # Python API for handling ROS2 bags
import rclpy.serialization
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time # ROS2 message for timestamping 

def fix_fisheye_with_bag_time(input_bag, output_bag):
    # Set up storage options for reading and writing bags
    storage_options = rosbag2_py.StorageOptions(uri=input_bag, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    
    # Initialize a sequential reader to read from the input bag
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

     # Initialize a sequential writer to write to the output bag
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=output_bag, storage_id='sqlite3'), converter_options)
    
    # Create the same topics in the output bag as in the input bag
    for topic_metadata in reader.get_all_topics_and_types():
        writer.create_topic(topic_metadata)
    
    # Process each message in the input bag
    while reader.has_next():
        topic, data, t = reader.read_next() # Read next message
        # t is the bag-recorded timestamp (nanoseconds)
        if topic in ["/fisheye_image_SN00012", "/fisheye_image_SN00013", "/fisheye_image_SN00014"]: # If the message is from one of the specified fisheye camera topics
            # Deserialize
            msg = rclpy.serialization.deserialize_message(data, Image)
            
            # Convert t -> builtin_interfaces/Time   # Convert the bag-recorded time to Time format and set as new timestamp
            sec = t // 1_000_000_000 # Convert nanoseconds to seconds
            nsec = t % 1_000_000_000  # Remainder as nanoseconds
            msg.header.stamp = Time(sec=sec, nanosec=nsec)
            
            # Re-serialize
            data = rclpy.serialization.serialize_message(msg)
        
        # Write the message to the output bag
        writer.write(topic, data, t)
    
    # Clean up by deleting writer and reader objects
    del writer
    del reader

# Example usage to fix timestamps in a specific bag file
fix_fisheye_with_bag_time("/media/prabuddhi/Crucial X9/Human_sensing_bags/Day2/1/1_0.db3", "new.db3")

