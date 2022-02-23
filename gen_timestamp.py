import argparse
from glob import glob
import os

def main(dataset_path):
    depth_file_list = glob(os.path.join(dataset_path, "depth/*.png"))
    with open(os.path.join(dataset_path, "timestamps.txt"), "a") as ts_f:
        for idx, depth_fname in enumerate(depth_file_list):
            depth_ts = depth_fname.split("/")[-1].replace(".png", "")
            index_str = "%06d" % idx
            line_txt = index_str + " " + depth_ts + "\n"
            ts_f.write(line_txt)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate timestamps for dataset.")
    parser.add_argument("--dataset", help="Path to the dataset to index timestamps.")
    args = parser.parse_args()
    main(args.dataset)
