import argparse
from pathlib import Path
from typing import Union
import multiprocessing
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import os
# from rclpy.serialization import serialize_message
import shutil

# ---------------------------------------------------------------------------- #
#                                  PARAMETERS                                  #
# ---------------------------------------------------------------------------- #

# Output path for the new bag
OUTPUT_PATH = "/media/roar/2a177b93-e672-418b-8c28-b075e87fcbc7/Chris_short_bags/Rosbags/mittPittBag/"
# ---------------------------------------------------------------------------- #


class MessageIterator:
    def __init__(self, cur, message_type: str, buffer_size: int = 1000) -> None:
        self.cur = cur
        self.len = self.cur.rowcount
        self.message_type = message_type
        self.buffer_size = buffer_size

    def __iter__(self):
        while True:
            buffer = self.__fetch_some()
            if not buffer:
                break
            for timestamp, message in buffer:
                yield timestamp, message

    def __deserialize(self, row):
        timestamp, data = row
        return timestamp, deserialize_message(data, self.message_type)

    def __fetch_some(self):
        pass

        rows = self.cur.fetchmany(self.buffer_size)

        # with multiprocessing.Pool() as p:
        #     result = p.map(self.__deserialize, rows)
        result = [self.__deserialize(row) for row in rows]
        return result

    def __len__(self):
        return self.len

class SqlBagFileParser:
    """
    Class to parse ROS2 bag files in SQLite3 format and extract messages from specified topics.
    https://answers.ros.org/question/358686/how-to-read-a-bag-file-in-ros2/?answer=377686#post-id-377686
    """

    def __init__(self, bag_file: Union[str, Path]):
        import sqlite3

        # Connect to the SQLite3 database
        self.conn = sqlite3.connect(str(bag_file))
        self.cursor = self.conn.cursor()

        # create a message type map
        # Query the topics table to get topic IDs, names, and types
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()

        # Create dictionaries for topic types, IDs, and message types
        self.topic_type = {name_of: type_of for id_of, name_of, type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
        self.topic_msg_message = {name_of: type_of for id_of, name_of, type_of in topics_data}

    def __del__(self):
        # Close the SQLite3 connection when the object is deleted
        if self.conn:
            self.conn.close()

    def get_messages(self, topic_name:str, lazy=False):
        """get converted messages from ROS2 bag
        Args:
            topic_name (str): name of the topic to extract.
            lazy (bool, optional): If true, return a MessageIterator.
                                   If false, return a list of all the messages.
                                   Defaults to False.
        Returns:
            [type]: [description]
        """

        if not topic_name in self.topic_msg_message:
            return None
        topic_id = self.topic_id[topic_name]

        message_type = get_message(self.topic_msg_message[topic_name])
        if not lazy:
            # Get from the db
            rows = self.cursor.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id = {} ORDER BY timestamp".format(topic_id)
            ).fetchall()
            # Deserialise all and timestamp them
            return [
                (timestamp, deserialize_message(data, message_type)) for timestamp, data in rows
            ]
        else:
            rows_cursor = self.cursor.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id = {} ORDER BY timestamp".format(topic_id)
            )
            return MessageIterator(rows_cursor, message_type)

def create_topic(writer, topic_name, typemap, serialization_format='cdr'):
    """
    Create a new topic.
    :param writer: writer instance
    :param topic_name:
    :param topic_type:
    :param serialization_format:
    :return:
    """
    topic = rosbag2_py.TopicMetadata(name=topic_name, type=typemap[topic_name], \
                                     serialization_format=serialization_format)
    writer.create_topic(topic)

# ---------------------------------------------------------------------------- #
#                                     MAIN                                     #
# ---------------------------------------------------------------------------- #
def main():
    print("Starting db3_to_mcap.py")
    # Getting Arguments Parsed from the Command Line
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", help="input bag path (folder or filepath) to read from"
    )

    args = parser.parse_args()

    bag_file_path = Path(args.input)
    print(f"Bag File Path: {bag_file_path}")

    # Initialize the SqlBagFileParser with the bag file path
    parser = SqlBagFileParser(bag_file_path)

    # Get the list of available topics in the bag file
    available_topics = list(parser.topic_msg_message.keys())
    print(f"Available Topics: {len(available_topics)}")

    message_iterators = []

    # Create a MessageIterator for each topic
    print ("Creating Message Iterators...")
    counter = 0
    for topic in available_topics:
        # Create an array of (earliest_timestamp, earliest_msg, topic, message_type, message_iterator)
        message_iterator = parser.get_messages(topic, lazy=True)
        message_iterator = message_iterator.__iter__()
        message_type = parser.topic_type[topic]
        try:
            earliest_timestamp, earliest_msg= next(message_iterator)
        except StopIteration:
            print(f"Topic {topic} has no messages")
            available_topics.remove(topic)
            continue
        message_iterators.append([earliest_timestamp, earliest_msg, topic, message_type, message_iterator]) 
        counter += 1
    print(f"{counter} Message Iterators Created out of {len(available_topics)} available!")

    # ---------------------------------------------------------------------------- #
    #                            Writing into output bag                           #
    # ---------------------------------------------------------------------------- #
    writer = rosbag2_py.SequentialWriter()


    # If the folder already exists, delete it
    if os.path.exists(OUTPUT_PATH):
        shutil.rmtree(OUTPUT_PATH)

    # Opens the bag file and sets the converter options
    writer.open(
        rosbag2_py.StorageOptions(uri=OUTPUT_PATH, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    # Creating topics 
    for topic_name in available_topics:
        create_topic(writer, topic_name, parser.topic_type)
    
    print("Writing Messages to Output Bag")
    counter = 0

    # Write messages to the output bag
    while message_iterators != []:
        iterator_arr = min(message_iterators, key=lambda x: x[0]) # Get the iterator with the earliest message
        writer.write(iterator_arr[2], serialize_message(iterator_arr[1]), iterator_arr[0])
        
        # Update the iterator array with the next message
        try:
            iterator_arr[0], iterator_arr[1] = next(iterator_arr[4])
        except StopIteration:
            message_iterators.remove(iterator_arr)

        counter += 1
        # if counter % 1000 == 0:
        print(f"Messages Written: {counter}")
    
    # Close the bag file
    del writer

if __name__ == "__main__":
    main()