import argparse
from pathlib import Path
from typing import Union
# import multiprocessing
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import os
# from rclpy.serialization import serialize_message
import shutil

class MessageIterator:
    def __init__(self, cur, buffer_size: int = 1000) -> None:
        self.cur = cur
        self.len = self.cur.rowcount
        self.buffer_size = buffer_size

        # Buffer to store the results of the fetchmany call
        self._buffer_index = 0
        self._buffer = []

    def __iter__(self):
        return self

    def __next__(self):
        if self._buffer_index >= len(self._buffer):
            self.__fetch_some()
            self._buffer_index = 0

            # If there are no more messages, we are done
            if not self._buffer:
                raise StopIteration
        
        item = self._buffer[self._buffer_index]
        self._buffer_index += 1
        return item

    def __fetch_some(self):
        # Fetch the next batch of messages
        rows = self.cur.fetchmany(self.buffer_size)
        result = [row for row in rows]
        self._buffer = result

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
        # self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
        self.topic_id = {id_of: name_of for id_of, name_of, type_of in topics_data}
        self.topic_msg_message = {name_of: type_of for id_of, name_of, type_of in topics_data}

    def __del__(self):
        # Close the SQLite3 connection when the object is deleted
        if self.conn:
            self.conn.close()

    def get_messages(self):
        """get converted messages from ROS2 bag
        Args:
            topic_name (str): name of the topic to extract.
            lazy (bool, optional): If true, return a MessageIterator.
                                   If false, return a list of all the messages.
                                   Defaults to False.
        Returns:
            [type]: [description]
        """

        # Creating the Iterator
        rows_cursor = self.cursor.execute(
            "SELECT timestamp, data, topic_id FROM messages"
        )
        return MessageIterator(rows_cursor)

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
    print("\nConverter Starting!\n")
    # Getting Arguments Parsed from the Command Line
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", help="input bag path (folder or filepath) to read from"
        
    )
    parser.add_argument(
        "output", help="output bag path (folder or filepath) to write to"
    )

    args = parser.parse_args()

    bag_file_path = Path(args.input)
    print(f"Bag File Path: {bag_file_path} \n")

    # Initialize the SqlBagFileParser with the bag file path
    parser = SqlBagFileParser(bag_file_path)

    # Get the list of available topics in the bag file
    available_topics = list(parser.topic_msg_message.keys())
    message_iterator = parser.get_messages()


    # ---------------------------------------------------------------------------- #
    #                            Writing into output mcap bag                      #
    # ---------------------------------------------------------------------------- #
    writer = rosbag2_py.SequentialWriter()

    # If the folder already exists, delete it
    if os.path.exists(args.output):
        shutil.rmtree(args.output)

    # Opens the bag file and sets the converter options
    writer.open(
        rosbag2_py.StorageOptions(uri=args.output, storage_id="mcap"),
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
    for timestamp, data, topic_id in message_iterator:

        topic_name = parser.topic_id[topic_id]
        
        writer.write(topic_name, data, timestamp)

        counter += 1
        if counter % 10000 == 0:
            print(f"Messages Written: {counter}")
    
    # Close the bag file
    del writer

    print("Finished Writing Messages to Output Bag! Enjoy your new bag file :)")

if __name__ == "__main__":
    main()