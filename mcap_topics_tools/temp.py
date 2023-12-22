"""script that reads ROS2 messages from an MCAP bag using the rosbag2_py API."""
import argparse
import os
import shutil
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import ctypes
import struct
# from ros2_ars540_msgs.msg import DetectionSegmentArray, DetectionSegment

INPUT_PATH = "/media/chris/IACSSD4TBT2/dec_13_23_run_2/rosbag2_2023_12_13-15_48_09/"
OUTPUT_PATH = "/media/chris/IACSSD4TBT2/dec_13_23_run_2/radar_ptc/rosbag2_2023_12_13-15_48_09/"


# If the output folder already exists, ask the user if they want to delete it
if os.path.exists(OUTPUT_PATH):
    print ("OUTPUT_PATH: ", OUTPUT_PATH, "\n")
    answer = input("The OUTPUT_PATH already exists. Do you want to delete it? (y/n) ")
    if answer.lower() == "y":
        shutil.rmtree(OUTPUT_PATH)
        print("Folder deleted.")
    else:
        print("Deletion cancelled. Re-run the script with a different OUTPUT_PATH.")
        exit()
        
# Check if the extension is a db3 or MCAP
store_type = None
files = os.listdir(INPUT_PATH)
for file in files:
    if file.endswith(".db3"):
        store_type = "sqlite3"
        print("Detected Input bag is a db3 file.")
    elif file.endswith(".mcap"):
        store_type = "mcap"
        print("Detected Input bag is a mcap file.")
if not store_type:
    print("Error: Input bag is not a db3 or mcap file.")
    exit()

reader = rosbag2_py.SequentialReader()
writer = rosbag2_py.SequentialWriter()

# Opens the bag files and sets the converter options
try:
    reader.open(
        rosbag2_py.StorageOptions(uri=INPUT_PATH, storage_id=store_type),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    writer.open(
        rosbag2_py.StorageOptions(uri=OUTPUT_PATH, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

except Exception as e:
    print(e)

# Get all topics and types
TOPIC_TYPES = reader.get_all_topics_and_types()
TYPE_MAP = {TOPIC_TYPES[i].name: TOPIC_TYPES[i].type for i in range(len(TOPIC_TYPES))}

print ("\n\n All topics and types in the input bag: \n")
for i in TYPE_MAP:
    print(i, "  |  ",  TYPE_MAP[i])

TOPICS_TO_EXCLUDE = {
    # Camera Topics
    "/vimba_front_left_center/image",
    "/vimba_front_left_center/image/compressed",
    "/vimba_front_right_center/image",
    "/vimba_front_right_center/image/compressed",
    "/vimba_front_left/image",
    "/vimba_front_left/image/compressed",
    "/vimba_front_right/image",
    "/vimba_front_right/image/compressed",

    # We don't have these:
    "/luminar_front/status",
    "/luminar_left/status",
    "/luminar_right/status",
    "/luminar_rear/status"
}

radar_topicname="/ars548_process/detections"
radar_ptc_topicname="/ars548_process/detections/pointcloud"


##########################################################################################################3

def process_radar(radar_msg):
    # Assuming radar_msg is a DetectionSegmentArray message

    # Create a PointCloud2 message
    cloud_msg = PointCloud2()
    cloud_msg.header = std_msgs.msg.Header()
    cloud_msg.header.stamp = radar_msg.header.stamp
    cloud_msg.header.frame_id = "radar_front"#radar_msg.header.frame_id # Set the frame ID appropriately
    # print(radar_msg.header.frame_id )

    # Define the fields in PointCloud2
    fields = [  
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        # Add more fields if needed, like intensity, etc.
    ]
    cloud_msg.fields = fields
    cloud_msg.is_bigendian = False
    cloud_msg.point_step = 12  # If only x, y, z
    cloud_msg.is_dense = True  # If there are no invalid points

    # Process each detection and convert to point cloud data
    points = []
    for detection in radar_msg.detections:
        # Convert detection data (range, azimuth, elevation) to x, y, z
        # This conversion depends on the radar's coordinate system and units
        # Example conversion (assuming spherical coordinates and meter units):
        r = detection.range  # range
        azimuth = np.deg2rad(detection.azimuth_angle)  # Convert to radians
        elevation = np.deg2rad(detection.elevation_angle)  # Convert to radians
        x = r * np.cos(elevation) * np.sin(azimuth)
        y = r * np.cos(elevation) * np.cos(azimuth)
        z = r * np.sin(elevation)

        # Pack the point data to fit the PointCloud2 format
        packed_data = struct.pack('fff', x, y, z)
        points.append(packed_data)

    # Set the data and dimensions in PointCloud2
    cloud_msg.data = b''.join(points)
    cloud_msg.height = 1
    cloud_msg.width = len(radar_msg.detections)
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width

    return cloud_msg

###############################################################################################################33333

'''
This while loop does the following:
1. Read the next message
2. Check if the topic is in the set of topics to extract
'''
counter = 0
timestamp_buffer = 0
print ("\n***MODE: Filtering the whole bag***")
print ("\nIterating through the input bag...")

radar_msg_buf = None

while reader.has_next():
    
    # Read the next message
    topic_name, data, timestamp = reader.read_next()

    # Keep a map of the topics that are in the output bag
    out_bag_topics = set()

    # If the topics are in TOPICS_TO_EXCLUDE
    if topic_name not in TOPICS_TO_EXCLUDE:

        # Create the topic if it doesn't exist
        if topic_name not in out_bag_topics:
            topic = rosbag2_py.TopicMetadata(name=topic_name, type=TYPE_MAP[topic_name], \
                    serialization_format='cdr')
            writer.create_topic(topic)
            out_bag_topics.add(topic_name)
        
        # If is a radar topic, we process into pointcloud
        if topic_name == radar_topicname:
            # Add new PointCloud2
            if radar_ptc_topicname not in out_bag_topics:
                topic = rosbag2_py.TopicMetadata(name=radar_ptc_topicname, type='sensor_msgs/msg/PointCloud2', \
                        serialization_format='cdr')
                writer.create_topic(topic)
                out_bag_topics.add(radar_ptc_topicname)

            # Process the incoming data
            msg_type = get_message(TYPE_MAP[topic_name])
            msg = deserialize_message(data, msg_type)
            radar_msg_buf = msg

            # print("Processing Radar Message!")
            ptc_radar = serialize_message(process_radar(msg))

            # Write the radar pointcloud to the output bag
            writer.write(radar_ptc_topicname, ptc_radar, timestamp)
    
        # Write the message to the output bag
        writer.write(topic_name, data, timestamp)

    # Print the current timestamp
    # if counter % 50000 == 0:
    if counter % 500000 == 0 and counter > 1:

        print("Currently: ", timestamp/1e9)
        break

    counter += 1

# Properly close the reader and writer
# reader.close()
# writer.close()
# print("Rosbag Closed Properly")

# Close the bag file
del reader
del writer