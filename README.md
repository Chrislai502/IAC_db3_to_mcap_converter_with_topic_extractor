# ROS2 bag mcap Topic Extractor / db3 to mcap converter

## Description
This repository contains 2 scripts, an db3_to_mcap converter and mcap_topic_extractor.

## Usage
Navigate to the directory where the script is located and run the following command in the terminal.
To use the mcap_topic_extractor:

```
python3 mcap_topic_extractor.py /path/to/input_bag/
```

To use the db3_to_mcap, run:

```
python3 db3_to_mcap.py /input/bag/bag.db3 /output/path/ 
```

## Parameters For mcap_topic_extractor
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
