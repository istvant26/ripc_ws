import os
import csv
import argparse
import rosbag2_py
import rclpy.serialization
from rosidl_runtime_py.utilities import get_message


def flatten_msg(msg):
    """Convert ROS message to a flat dict."""
    out = {}

    def recurse(obj, prefix=""):
        if hasattr(obj, "__slots__"):
            for slot in obj.__slots__:
                val = getattr(obj, slot)
                recurse(val, f"{prefix}{slot}.")
        elif isinstance(obj, (list, tuple)):
            for i, v in enumerate(obj):
                recurse(v, f"{prefix}{i}.")
        else:
            out[prefix[:-1]] = obj

    recurse(msg)
    return out


def export_topic(reader, topic_name, type_name, out_csv):
    MsgType = get_message(type_name)

    with open(out_csv, "w", newline="") as f:
        writer = None

        while reader.has_next():
            topic, data, t = reader.read_next()

            if topic != topic_name:
                continue

            msg = rclpy.serialization.deserialize_message(data, MsgType)
            row = flatten_msg(msg)
            row["timestamp"] = t

            if writer is None:
                writer = csv.DictWriter(f, fieldnames=row.keys())
                writer.writeheader()

            writer.writerow(row)

    print(f"✔ Wrote {out_csv}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", help="Path to ros2 bag folder")
    parser.add_argument("--out", default="csv_export", help="Output directory")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=args.bag_dir, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")

    reader.open(storage_options, converter_options)
    topics = reader.get_all_topics_and_types()

    for t in topics:
        safe_name = t.name.replace("/", "_")[1:]
        csv_path = os.path.join(args.out, f"{safe_name}.csv")

        export_topic(reader, t.name, t.type, csv_path)

        # rewind reader for next topic
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)


if __name__ == "__main__":
    main()
