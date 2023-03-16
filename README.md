# ROS2 mcap Rosbag Topic Extractor

## Description
This script extracts specific topics from an MCAP bag and creates a new bag with only those topics. The script reads the input bag file and writes the desired topics into a new output bag file. The user can specify the topics to be extracted and the output path for the new bag. 

## Usage
To use the script, navigate to the directory where the script is located and run the following command in the terminal: 

```
python3 script.py input_bag
```

Where `input_bag` is the path to the input bag directory.

## Parameters
The script uses two parameters you can change in main.py: 

- `TOPICS_TO_EXTRACT`: Desired topics to extract from the bag.
- `OUTPUT_PATH`: Output path for the new bag.

## Dependencies
The script requires the following dependencies: 
- `argparse`
- `rclpy`
- `rosidl_runtime_py`
- `rosbag2_py`

## Execution
When executed, the script will read all the messages in the input bag file and create a new bag file with only the desired topics. 

## Note
This script assumes that the input bag has messages in the CDR serialization format. If the input bag has messages in a different serialization format, please modify the `input_serialization_format` and `output_serialization_format` options in the `ConverterOptions` class.

## Sources
https://github.com/foxglove/mcap/blob/main/python/examples/ros2/py_mcap_demo/py_mcap_demo/writer.py
https://github.com/foxglove/mcap/blob/main/python/examples/ros2/py_mcap_demo/py_mcap_demo/reader.py
