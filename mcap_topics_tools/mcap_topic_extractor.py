"""script that reads ROS2 messages from an MCAP bag using the rosbag2_py API."""
import argparse
import os
import shutil
import rosbag2_py
import yaml


# ---------------------------------------------------------------------------- #
#                                  PARAMETERS                                  #
# ---------------------------------------------------------------------------- #
# Load global constants from YAML file
with open('config.yaml', 'r') as file:
    config = yaml.safe_load(file)

TOPICS_TO_EXCLUDE = set(config.get('TOPICS_TO_EXCLUDE', []))
INCLUDE_TOPICS_MODE = config.get('INCLUDE_TOPICS_MODE', False)
TOPICS_TO_INCLUDE = set(config.get('TOPICS_TO_INCLUDE', []))
FILTER_BY_TIMESTAMP = config.get('FILTER_BY_TIMESTAMP', False)
FROM_TIMESTAMP = config.get('FROM_TIMESTAMP', 0)
TO_TIMESTAMP = config.get('TO_TIMESTAMP', float("inf"))
INPUT_PATH = config.get('INPUT_PATH', '')
OUTPUT_PATH = config.get('OUTPUT_PATH', '')

# ----------------------------- Timestamp Filter ----------------------------- #
# The Parameters are the start and end timestamps to filter the bag, all TOPICS_TO_INCLUDE
# will be included in the output bag if they have a message within the range:
#
# FROM_TIMESTAMP_TOPIC.FROM_TIMESTAMP < TO_TIMESTAMP_TOPIC.timestamp < TO_TIMESTAMP_TOPIC.TO_TIMESTAMP
#
# Example FROM_TIMESTAMP and TO_TIMESTAMP formats:
# sec: 1673037998
# nanosec: 455526000
# FROM_TIMESTAMP = 1673037998455526000
# FROM_TIMESTAMP = 1673037998455526000

# Set to TRUE if using timestamp filter. Else, it will filter the whole rosbag.
FILTER_BY_TIMESTAMP = True

FROM_TIMESTAMP = 1673037588612982487
FROM_TIMESTAMP = -float("inf")
FROM_TIMESTAMP_TOPIC = "/luminar_front_points" # topic to check timestamp from

# sec: 1673038010
# nanosec: 792381600
# ExampleTO_TIMESTAMP = 1673038010792381600
# TO_TIMESTAMP = 1673037592257594379
TO_TIMESTAMP = float("inf")
TO_TIMESTAMP_TOPIC = FROM_TIMESTAMP_TOPIC

# ------------------------ Input and Output Paths ----------------------- #
# Input path is the path to the bag to be filtered.
# Output path is the path to the new bag to be created.
# Input bag can be in sqlite3 or mcap format, Output by default is mcap format uunless you change the code.
# INPUT_PATH = "/media/Public/ROSBAG_BACKUPS/VEGAS_CES_2022/Jan6th_PTP_synced_cam_Lidar/rosbag2_2023_01_06-15_39_44/"
# INPUT_PATH = "/home/zhihao/rosbags/merged_rosbag/"
INPUT_PATH = "/media/Public/ROSBAG_BACKUPS/VEGAS_CES_2022/Jan6th_PTP_synced_cam_Lidar/rosbag2_2023_01_06-15_39_44/"
INPUT_PATH = "/home/zhihao/rosbags/merged_rosbag/"
# INPUT_PATH = "/home/zhihao/chris/testing_bag/"

OUTPUT_PATH = "/media/roar/2a177b93-e672-418b-8c28-b075e87fcbc7/Chris_short_bags/Rosbags/radar_only/timestamp_test/"
OUTPUT_PATH = "/home/zhihao/rosbags/filtered_exclude_merged_rosbag/"
# OUTPUT_PATH = "/home/zhihao/chris/testing_bag_mcap/"

# ---------------------- Topics to extract from the bag ---------------------- #

# Using the topics to extract list or Topics to exclude
INCLUDE_TOPICS_MODE = False 

# ---------------------------------------------------------------------------- #

# ---------------------------------------------------------------------------- #
#                                MESSAGE_FILTER                                #
# ---------------------------------------------------------------------------- #
def message_filter(input_bag_folder: str):
    
    global TOPIC_TYPES
    global TYPE_MAP
    global TOPICS_TO_INCLUDE
    global FROM_TIMESTAMP
    global TO_TIMESTAMP
    global OUTPUT_PATH
    global FROM_TIMESTAMP_TOPIC
    global TO_TIMESTAMP_TOPIC
    global FILTER_BY_TIMESTAMP
    global INCLUDE_TOPICS_MODE
    global TOPICS_TO_EXCLUDE

    reader = rosbag2_py.SequentialReader()
    writer = rosbag2_py.SequentialWriter()

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
    files = os.listdir(input_bag_folder)
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

    # Opens the bag files and sets the converter options
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=input_bag_folder, storage_id=store_type),
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
        return
    

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

    TOPIC_TYPES = reader.get_all_topics_and_types()
    TYPE_MAP = {TOPIC_TYPES[i].name: TOPIC_TYPES[i].type for i in range(len(TOPIC_TYPES))}
    
    print ("\n\n All topics and types in the input bag: \n")
    for i in TYPE_MAP:
        print(i, "  |  ",  TYPE_MAP[i])
        
    # Check if the topics to extract are in the input bag
    if TOPICS_TO_INCLUDE and INCLUDE_TOPICS_MODE:
        for topic in TOPICS_TO_INCLUDE:
            if topic not in TYPE_MAP:
                print("ERROR: The topic ", topic, " is not in the input bag.")
                exit()
            print("Topic ", topic, " found in the input bag.")
    else:
        print("FATAL ERROR: TOPICS_TO_INCLUDE not defined.")
        

    # Check if FILTER_BY_TIMESTAMP is True, Else Run through the whole bag
    if FILTER_BY_TIMESTAMP:

        '''
        This while loop does the following:
        1. Read the next message
        2. Check if the topic is in the set of topics to extract
        '''
        
        # Check if FROM_TIMESTAMP_TOPIC and TO_TIMESTAMP_TOPIC are in the input bag
        if FROM_TIMESTAMP_TOPIC not in TYPE_MAP or TO_TIMESTAMP_TOPIC not in TYPE_MAP:
            print("ERROR: The topics for the timestamp range are not in the input bag.")
            return
        
        counter = 0
        timestamp_buffer = 0
        print ("\n***MODE: Filtering by Timestamp***")
        print ("\nIterating through the input bag...")
        while reader.has_next():
            
            # Read the next message
            topic_name, data, timestamp = reader.read_next()

            if topic_name == FROM_TIMESTAMP_TOPIC and \
                FROM_TIMESTAMP <= timestamp <= TO_TIMESTAMP:

                print("Found the first timestamp range!")
                print("Timestamp: ", timestamp)

                # Keep a map of the topics that are in the output bag
                out_bag_topics = set()

                curr_timestamp = timestamp
                while (curr_timestamp <= TO_TIMESTAMP and reader.has_next()):
                    
                    # Read the next message
                    topic_name, data, timestamp = reader.read_next()
                    
                    # Check if the topic is in the set of topics to extract or to exclude.
                    if INCLUDE_TOPICS_MODE:
                        
                        # Check if the topic is in the set of topics to extract.
                        if topic_name in TOPICS_TO_INCLUDE:
                        
                            # Create the topic if it doesn't exist
                            if topic_name not in out_bag_topics:
                                topic = rosbag2_py.TopicMetadata(name=topic_name, type=TYPE_MAP[topic_name], \
                                        serialization_format='cdr')
                                writer.create_topic(topic)
                                out_bag_topics.add(topic_name)

                            # Write the message to the output bag
                            writer.write(topic_name, data, timestamp)
                    else :
                        if topic_name not in TOPICS_TO_EXCLUDE:
                            
                            # Create the topic if it doesn't exist
                            if topic_name not in out_bag_topics:
                                topic = rosbag2_py.TopicMetadata(name=topic_name, type=TYPE_MAP[topic_name], \
                                        serialization_format='cdr')
                                writer.create_topic(topic)
                                out_bag_topics.add(topic_name)
                            
                            # Write the message to the output bag
                            writer.write(topic_name, data, timestamp)

                    # Update the current timestamp
                    if topic_name == TO_TIMESTAMP_TOPIC:
                        curr_timestamp = timestamp

                    # Print the current timestamp
                    if counter % 50000 == 0:
                        print("Currently: ", curr_timestamp, " | Target: ", \
                            TO_TIMESTAMP, " | Diff: ", (TO_TIMESTAMP - curr_timestamp)/1e9, " secs")

                    counter += 1

                print("Found the last timestamp range!")

                # Break after the range is reached
                break
            
            # Print the current timestamp
            if (topic_name == FROM_TIMESTAMP_TOPIC):
                timestamp_buffer = timestamp
            if counter % 50000 == 0:
                print("Currently: ", timestamp_buffer, " | Target: ", \
                        FROM_TIMESTAMP, " | Diff: ", (FROM_TIMESTAMP - timestamp_buffer)/1e9, " secs")
            counter += 1
            
    else:
        
        '''
        This while loop does the following:
        1. Read the next message
        2. Check if the topic is in the set of topics to extract
        '''
        counter = 0
        timestamp_buffer = 0
        print ("\n***MODE: Filtering the whole bag***")
        print ("\nIterating through the input bag...")
        while reader.has_next():
            
            # Read the next message
            topic_name, data, timestamp = reader.read_next()

            # Keep a map of the topics that are in the output bag
            out_bag_topics = set()

            # Check if the topic is in the set of topics to extract or to exclude.
            if INCLUDE_TOPICS_MODE:

                # Check if the topic is in the set of topics to extract.
                if topic_name in TOPICS_TO_INCLUDE:
                
                    # Create the topic if it doesn't exist
                    if topic_name not in out_bag_topics:
                        topic = rosbag2_py.TopicMetadata(name=topic_name, type=TYPE_MAP[topic_name], \
                                serialization_format='cdr')
                        writer.create_topic(topic)
                        out_bag_topics.add(topic_name)

                    # Write the message to the output bag
                    writer.write(topic_name, data, timestamp)
                
            else :
                if topic_name not in TOPICS_TO_EXCLUDE:
                    
                    # Create the topic if it doesn't exist
                    if topic_name not in out_bag_topics:
                        topic = rosbag2_py.TopicMetadata(name=topic_name, type=TYPE_MAP[topic_name], \
                                serialization_format='cdr')
                        writer.create_topic(topic)
                        out_bag_topics.add(topic_name)
                    
                    # Write the message to the output bag
                    writer.write(topic_name, data, timestamp)

            # Print the current timestamp
            if counter % 50000 == 0:
                print("Currently: ", timestamp, " | Target: NO TIMESTAMP", " | Diff: ", (TO_TIMESTAMP - timestamp)/1e9, " secs")

            counter += 1

    # Close the bag file
    del reader
    del writer

# ---------------------------------------------------------------------------- #
#                                     MAIN                                     #
# ---------------------------------------------------------------------------- #
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input", help="input bag path (folder or filepath) to read from", default=None
    )

    args = parser.parse_args()
    input_path = None

    # Check if Parameters are given.
    global TOPICS_TO_INCLUDE
    if TOPICS_TO_INCLUDE == []:
        TOPICS_TO_INCLUDE = None

    if args.input is None:
        if INPUT_PATH is None:
            print("No input path given. Exiting...")
            return
        else:
            input_path = INPUT_PATH

    print("Reading from Rosbag: ", input_path, "\n")
    print("Reading messages from bag...\n")

    # Filter and write the messages to a new bag
    message_filter(input_path)
    
    print("Done! Output bag in :", OUTPUT_PATH)
    print("Enjoy your new bag file! :)")

if __name__ == "__main__":
    main()
