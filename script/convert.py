"""Copyright (c) 2025 Dapeng Feng All rights reserved."""

# !/usr/bin/python3

import argparse
from typing import Union
from rclpy.time import Time
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import (
    PointCloud2,
    PointField,
    Imu,
    CompressedImage,
    Image,
)
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
from livox_ros_driver.msg import CustomMsg as LivoxCustomMsg
from luminous_msgs.msg import Header as LuminousHeader
from luminous_msgs.msg import CameraInfo as LuminousCameraInfo
from luminous_msgs.msg import CompressedImage as LuminousCompressedImage
from luminous_msgs.msg import PointCloud2 as LuminousPointCloud2
from luminous_msgs.msg import Imu as LuminousImu
import rosbag2_py

def camera_info_to_luminous_camera_info(
    camera_info: LuminousCameraInfo, frame_id: str = "camera"
) -> LuminousCameraInfo:
    """Convert a LuminousCameraInfo message to a LuminousCameraInfo with a new frame_id."""
    header = LuminousHeader(stamp=camera_info.header.stamp, frame_id=frame_id)

    return LuminousCameraInfo(
        header=header,
        height=camera_info.height,
        width=camera_info.width,
        distortion_model=camera_info.distortion_model,
        D=camera_info.D,
        K=camera_info.K,
        R=camera_info.R,
        P=camera_info.P,
        binning_x=camera_info.binning_x,
        binning_y=camera_info.binning_y,
        roi=camera_info.roi,
    )

def image_to_luminous_compressed_image(
    image: Union[Image, CompressedImage], frame_id: str = "camera"
) -> LuminousCompressedImage:
    header = LuminousHeader(stamp=image.header.stamp, frame_id=frame_id)

    if isinstance(image, Image):
        # Convert Image to CompressedImage
        bridge = CvBridge()
        compressed_image = bridge.cv2_to_compressed_imgmsg(
            bridge.imgmsg_to_cv2(image)
        )
        compressed_image.header = image.header
        image = compressed_image

    return LuminousCompressedImage(
        header=header, format=image.format, data=image.data
    )


def imu_to_luminous_imu(imu: Imu, frame_id: str = "imu") -> LuminousImu:
    header = LuminousHeader(stamp=imu.header.stamp, frame_id=frame_id)

    return LuminousImu(
        header=header,
        orientation=imu.orientation,
        angular_velocity=imu.angular_velocity,
        linear_acceleration=imu.linear_acceleration,
    )


livox_fields = [
    PointField(
        name="offset_time", offset=0, datatype=PointField.UINT32, count=1
    ),
    PointField(name="x", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(
        name="reflectivity", offset=16, datatype=PointField.UINT8, count=1
    ),
    PointField(name="tag", offset=17, datatype=PointField.UINT8, count=1),
    PointField(name="line", offset=18, datatype=PointField.UINT8, count=1),
]


def livox_to_luminous_pointcloud2(
    livox: LivoxCustomMsg, frame_id: str = "livox"
) -> LuminousPointCloud2:
    points = []
    for point in livox.points:
        points.append(
            [
                point.offset_time,
                point.x,
                point.y,
                point.z,
                point.reflectivity,
                point.tag,
                point.line,
            ]
        )

    pc2 = point_cloud2.create_cloud(
        header=livox.header,
        fields=livox_fields,
        points=points,
    )

    header = LuminousHeader(stamp=pc2.header.stamp, frame_id=frame_id)

    return LuminousPointCloud2(
        header=header,
        height=pc2.height,
        width=pc2.width,
        fields=pc2.fields,
        is_bigendian=pc2.is_bigendian,
        point_step=pc2.point_step,
        row_step=pc2.row_step,
        data=pc2.data,
        is_dense=pc2.is_dense,
    )


def pointcloud2_to_luminous_pointcloud2(
    pc2: point_cloud2.PointCloud2, frame_id: str = "velodyne"
) -> LuminousPointCloud2:
    header = LuminousHeader(stamp=pc2.header.stamp, frame_id=frame_id)

    return LuminousPointCloud2(
        header=header,
        height=pc2.height,
        width=pc2.width,
        fields=pc2.fields,
        is_bigendian=pc2.is_bigendian,
        point_step=pc2.point_step,
        row_step=pc2.row_step,
        data=pc2.data,
        is_dense=pc2.is_dense,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", help="input bag path (folder or filepath) to read from"
    )
    parser.add_argument(
        "output", help="output bag path (folder or filepath) to write to"
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="overwrite the output bag if it already exists",
    )

    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.input, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    if args.overwrite:
        import shutil
        import os

        if os.path.exists(args.output):
            shutil.rmtree(args.output)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.output, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    for topic_type in topic_types:
        if topic_type.type == "sensor_msgs/msg/CameraInfo":
            msg_type = "luminous_msgs/msg/CameraInfo"
        if topic_type.type == "sensor_msgs/msg/CompressedImage":
            msg_type = "luminous_msgs/msg/CompressedImage"
        elif topic_type.type == "sensor_msgs/msg/Image":
            msg_type = "luminous_msgs/msg/Image"
        elif topic_type.type == "sensor_msgs/msg/Imu":
            msg_type = "luminous_msgs/msg/Imu"
        elif topic_type.type == "livox_ros_driver/msg/CustomMsg":
            msg_type = "luminous_msgs/msg/PointCloud2"
        elif topic_type.type == "sensor_msgs/msg/PointCloud2":
            msg_type = "luminous_msgs/msg/PointCloud2"
        else:
            msg_type = topic_type.type
        try:
            get_message(msg_type)
        except Exception as e:
            print(f"Skipping topic {topic_type.name}: {e}")
            continue
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=topic_type.name, type=msg_type, serialization_format="cdr"
            )
        )

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, _ = reader.read_next()
        try:
            msg_type = get_message(typename(topic))
            msg = deserialize_message(data, msg_type)
        except Exception as e:
            print(f"Error processing message from {topic}: {e}")
            continue
        if isinstance(msg, LuminousCameraInfo):
            msg = camera_info_to_luminous_camera_info(msg)
        elif isinstance(msg, Image) or isinstance(msg, CompressedImage):
            msg = image_to_luminous_compressed_image(msg)
        elif isinstance(msg, Imu):
            msg = imu_to_luminous_imu(msg)
        elif isinstance(msg, LivoxCustomMsg):
            msg = livox_to_luminous_pointcloud2(msg)
        elif isinstance(msg, PointCloud2):
            msg = pointcloud2_to_luminous_pointcloud2(msg)

        writer.write(
            topic,
            serialize_message(msg),
            Time.from_msg(msg.header.stamp).nanoseconds,
        )


if __name__ == "__main__":
    main()
