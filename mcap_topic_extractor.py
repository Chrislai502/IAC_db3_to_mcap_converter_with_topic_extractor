"""script that reads ROS2 messages from an MCAP bag using the rosbag2_py API."""
import argparse

from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
import os
import shutil
import rosbag2_py

# ---------------------------------------------------------------------------- #
#                                  PARAMETERS                                  #
# ---------------------------------------------------------------------------- #

# Desired topics to extract from the bag
TOPICS_TO_EXTRACT = ["/luminar_front_points", "/luminar_left_points", "/luminar_right_points", \
                     "/radar_front/radar_visz_static", "/radar_front/radar_visz_static_array",\
                     "/radar_front/radar_visz_moving", "/radar_front/radar_visz_moving_array",\
                      "/perception/lvms_inside_vis", "/perception/lvms_outside_vis", \
                      '/radar_right/marker', '/radar_front/from_can_bus',  \
                      '/radar_front/to_can_bus'
,'/radar_front/esr_status1'\
,'/radar_front/esr_status2'\
,'/radar_front/esr_status3'\
,'/radar_front/esr_status4'\
,'/radar_front/esr_status5'\
,'/radar_front/esr_status6'\
,'/radar_front/esr_status7'\
,'/radar_front/esr_status8'\
,'/radar_front/esr_status9'\
,'/radar_front/esr_track'\
,'/radar_front/esr_valid1'\
,'/radar_front/esr_valid2'\
,'/radar_front/esr_vehicle1'\
,'/radar_front/esr_vehicle2'\
,'/radar_front/esr_vehicle3'\
,'/radar_front/esr_vehicle4'\
,'/radar_front/esr_vehicle5'\
,'/radar_left/detection'\
,'/radar_left/header_information_detections'\
,'/radar_left/marker'\
,'/radar_left/vehicle_state'\
,'/radar_liadar_detected_objects'\
,'/radar_right/detection'\
,'/radar_right/header_information_detections'\
,'/radar_right/marker'\
,'/radar_right/marker_array'\
,'/radar_right/vehicle_state'\

# Novatel
,'/novatel_bottom/bestgnsspos'\
,'/novatel_bottom/bestgnssvel'\
,'/novatel_bottom/bestpos'\
,'/novatel_bottom/bestutm'\
,'/novatel_bottom/bestvel'\
,'/novatel_bottom/corrimu'\
,'/novatel_bottom/fix'\
,'/novatel_bottom/gps'\
,'/novatel_bottom/heading2'\
,'/novatel_bottom/imu/data'\
,'/novatel_bottom/insconfig'\
,'/novatel_bottom/inspva'\
,'/novatel_bottom/inspvax'\
,'/novatel_bottom/insstdev'\
,'/novatel_bottom/odom'\
,'/novatel_bottom/oem7raw'\
,'/novatel_bottom/rawimu'\
,'/novatel_bottom/rxstatus'\
,'/novatel_bottom/time'\
,'/novatel_top/bestgnsspos'\
,'/novatel_top/bestgnssvel'\
,'/novatel_top/bestpos'\
,'/novatel_top/bestutm'\
,'/novatel_top/bestvel'\
,'/novatel_top/corrimu'\
,'/novatel_top/fix'\
,'/novatel_top/gps'\
,'/novatel_top/heading2'\
,'/novatel_top/imu/data'\
,'/novatel_top/insconfig'\
,'/novatel_top/inspva'\
,'/novatel_top/inspvax'\
,'/novatel_top/insstdev'\
,'/novatel_top/odom'\
,'/novatel_top/oem7raw'\
,'/novatel_top/rawimu'\
,'/novatel_top/rxstatus'\
,'/novatel_top/time']

# Output path for the new bag
OUTPUT_PATH = "/media/roar/2a177b93-e672-418b-8c28-b075e87fcbc7/Chris_short_bags/Rosbags/radar_only/wheel_spoke_and_mirror/"
# ---------------------------------------------------------------------------- #




# ---------------------------------------------------------------------------- #
#                            READER FOR EXISTING BAG                           #
# ---------------------------------------------------------------------------- #
# Get the message type for a topic
def typename(topic_name, topic_types):
    for topic_type in topic_types:
        if topic_type.name == topic_name:
            return topic_type.type
    raise ValueError(f"topic {topic_name} not in bag")

# Global topic_types variable when the bag is opened
TOPIC_TYPES = None
TYPE_MAP = None

def read_messages(input_bag: str, topics_to_extract: list = None):
    reader = rosbag2_py.SequentialReader()
    
    # Opens the bag file and sets the converter options
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    # Get all topics and types
    '''
    Reader has 4 functions:
    - get_all_topics_and_types
    - has_next
    - read_next
    - reset_filter
    - set_filter
    # Set filter for topic of string type
    source: https://github.com/ros2/rosbag2/blob/c7c7954d4d9944c160d7b3d716d1cb95d34e37e4/rosbag2_py/test/test_sequential_reader.py
    storage_filter = rosbag2_py.StorageFilter(topics=['/topic'])
    reader.set_filter(storage_filter)

    '''
    global TOPIC_TYPES
    global TYPE_MAP
    TOPIC_TYPES = reader.get_all_topics_and_types()
    TYPE_MAP = {TOPIC_TYPES[i].name: TOPIC_TYPES[i].type for i in range(len(TOPIC_TYPES))}
    for i in TYPE_MAP:
        print(i, "  |  ",  TYPE_MAP[i])

    # Sequentially read messages from the bag, one by one. 
    while reader.has_next():
        
        # Read the next message
        topic_name, data, timestamp = reader.read_next()

        # Check if the topic is in the set of topics to extract.
        if topic_name in set(topics_to_extract):
          topic_type = TYPE_MAP[topic_name]
          msg_type = get_message(topic_type)
          msg = deserialize_message(data, msg_type)
          yield topic_name, msg, timestamp

        # Implicit: else: continue

    # Close the bag file
    del reader
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
#                              WRITER FOR NEW BAG                              #
# ---------------------------------------------------------------------------- #
def create_topic(writer, topic_name, serialization_format='cdr'):
    """
    Create a new topic.
    :param writer: writer instance
    :param topic_name:
    :param topic_type:
    :param serialization_format:
    :return:
    """
    global TYPE_MAP
    topic = rosbag2_py.TopicMetadata(name=topic_name, type=TYPE_MAP[topic_name], \
                                     serialization_format=serialization_format)
    writer.create_topic(topic)

def write_to(topic_names: list, messages: list, timestamps: list, output_path: str = "output/"):
    
    writer = rosbag2_py.SequentialWriter()

    # If the folder already exists, delete it
    if os.path.exists(output_path):
        shutil.rmtree(output_path)

    # Opens the bag file and sets the converter options
    writer.open(
        rosbag2_py.StorageOptions(uri=output_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    # Create topics
    global TOPICS_TO_EXTRACT
    for topic_name in TOPICS_TO_EXTRACT:
        create_topic(writer, topic_name)

    # Write messages to the output bag
    for i in range(len(topic_names)):
        writer.write(topic_names[i], serialize_message(messages[i]), timestamps[i])
    
    # Close the bag file
    del writer
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
#                                     MAIN                                     #
# ---------------------------------------------------------------------------- #
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", help="input bag path (folder or filepath) to read from"
    )

    args = parser.parse_args()

    # Initalizing list to store the messages
    messages = []
    topic_names = []
    timestamps = []

    # Check if Parameters are given.
    global TOPICS_TO_EXTRACT
    if TOPICS_TO_EXTRACT == []:
        TOPICS_TO_EXTRACT = None

    print("Reading from Rosbag: ", args.input, "\n")
    print("Reading messages from bag...\n")

    # TODO: Use tqdm to show progress bar (Not possible because of the generator)
    count = 0

    # Iterate through all the messages in the bag
    for topic_name, msg, timestamp in read_messages(args.input, TOPICS_TO_EXTRACT):
        topic_names.append(topic_name)
        messages.append(msg)
        timestamps.append(timestamp)
        count += 1

    # Make sure that all the lists are of same length
    assert len(topic_names) == len(messages) == len(timestamps)
    print("Passed the assertion check.\n")

    print("Type Map: ", TYPE_MAP, "\n")

    # Writing the messages to a new bag
    print("Writing messages to bag...\n")
    write_to(topic_names, messages, timestamps, OUTPUT_PATH)

if __name__ == "__main__":
    main()