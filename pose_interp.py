'''
Created: 2022/02/11
Maintainer: Ran Cheng <ran.cheng2@mail.mcgill.ca>
adapted from https://github.com/ethz-asl/robotcar_tools/blob/master/python/interpolate_poses.py
LICENSE: MIT
'''

import transformations as tfs
import numpy as np
import argparse
import bisect
import transformations as tfs


def main(kf_pose_fname, timestamps_fname):
    kf_pose = np.genfromtxt(kf_pose_fname)
    timestamps = np.genfromtxt(timestamps_fname)
    timestamps = np.squeeze(timestamps[:, 1]) * 1e6
    pose_timestamps = kf_pose[:, 0] * 1e6
    abs_poses = kf_pose[:, 1:]
    interpolated_pose = interpolate_poses(pose_timestamps.astype(np.int64).tolist(), abs_poses,
                                          timestamps.astype(np.int64).tolist(), int(timestamps[0]))
    np.savetxt(kf_pose_fname.replace(".txt", "_interp.txt"), np.array(interpolated_pose), fmt="%1.6f")


def interpolate_poses(pose_timestamps, abs_poses, requested_timestamps, origin_timestamp):
    """Interpolate between absolute poses.
    Args:
        pose_timestamps (list[int]): Timestamps of supplied poses. Must be in ascending order.
        abs_poses (list[numpy.matrixlib.defmatrix.matrix]): SE3 matrices representing poses at the timestamps specified.
        requested_timestamps (list[int]): Timestamps for which interpolated timestamps are required.
        origin_timestamp (int): UNIX timestamp of origin frame. Poses will be reported relative to this frame.
    Returns:
        list[numpy.matrixlib.defmatrix.matrix]: SE3 matrix representing interpolated pose for each requested timestamp.
    Raises:
        ValueError: if pose_timestamps and abs_poses are not the same length
        ValueError: if pose_timestamps is not in ascending order
    """
    requested_timestamps.insert(0, origin_timestamp)
    requested_timestamps = np.array(requested_timestamps)
    pose_timestamps = np.array(pose_timestamps)
    if len(pose_timestamps) != len(abs_poses):
        raise ValueError('Must supply same number of timestamps as poses')

    abs_quaternions = np.zeros((4, len(abs_poses)))
    abs_positions = np.zeros((3, len(abs_poses)))
    for i, pose in enumerate(abs_poses):
        if i > 0 and pose_timestamps[i - 1] >= pose_timestamps[i]:
            raise ValueError('Pose timestamps must be in ascending order')

        abs_quaternions[:, i] = pose[
                                3:]  # np.roll(pose[3:], -1) uncomment this if the quaternion is saved as [w, x, y, z]
        abs_positions[:, i] = pose[:3]

    upper_indices = [bisect.bisect(pose_timestamps, pt) for pt in requested_timestamps]
    lower_indices = [u - 1 for u in upper_indices]

    if max(upper_indices) >= len(pose_timestamps):
        upper_indices = [min(i, len(pose_timestamps) - 1) for i in upper_indices]

    fractions = (requested_timestamps - pose_timestamps[lower_indices]) // \
                (pose_timestamps[upper_indices] - pose_timestamps[lower_indices])

    quaternions_lower = abs_quaternions[:, lower_indices]
    quaternions_upper = abs_quaternions[:, upper_indices]

    d_array = (quaternions_lower * quaternions_upper).sum(0)

    linear_interp_indices = np.nonzero(d_array >= 1)
    sin_interp_indices = np.nonzero(d_array < 1)

    scale0_array = np.zeros(d_array.shape)
    scale1_array = np.zeros(d_array.shape)

    scale0_array[linear_interp_indices] = 1 - fractions[linear_interp_indices]
    scale1_array[linear_interp_indices] = fractions[linear_interp_indices]

    theta_array = np.arccos(np.abs(d_array[sin_interp_indices]))

    scale0_array[sin_interp_indices] = \
        np.sin((1 - fractions[sin_interp_indices]) * theta_array) / np.sin(theta_array)
    scale1_array[sin_interp_indices] = \
        np.sin(fractions[sin_interp_indices] * theta_array) / np.sin(theta_array)

    negative_d_indices = np.nonzero(d_array < 0)
    scale1_array[negative_d_indices] = -scale1_array[negative_d_indices]

    quaternions_interp = np.tile(scale0_array, (4, 1)) * quaternions_lower \
                         + np.tile(scale1_array, (4, 1)) * quaternions_upper

    positions_lower = abs_positions[:, lower_indices]
    positions_upper = abs_positions[:, upper_indices]

    positions_interp = np.multiply(np.tile((1 - fractions), (3, 1)), positions_lower) \
                       + np.multiply(np.tile(fractions, (3, 1)), positions_upper)

    poses_mat = np.zeros((4, 4 * len(requested_timestamps)))

    poses_mat[0, 0::4] = 1 - 2 * np.square(quaternions_interp[2, :]) - \
                         2 * np.square(quaternions_interp[3, :])
    poses_mat[0, 1::4] = 2 * np.multiply(quaternions_interp[1, :], quaternions_interp[2, :]) - \
                         2 * np.multiply(quaternions_interp[3, :], quaternions_interp[0, :])
    poses_mat[0, 2::4] = 2 * np.multiply(quaternions_interp[1, :], quaternions_interp[3, :]) + \
                         2 * np.multiply(quaternions_interp[2, :], quaternions_interp[0, :])

    poses_mat[1, 0::4] = 2 * np.multiply(quaternions_interp[1, :], quaternions_interp[2, :]) \
                         + 2 * np.multiply(quaternions_interp[3, :], quaternions_interp[0, :])
    poses_mat[1, 1::4] = 1 - 2 * np.square(quaternions_interp[1, :]) \
                         - 2 * np.square(quaternions_interp[3, :])
    poses_mat[1, 2::4] = 2 * np.multiply(quaternions_interp[2, :], quaternions_interp[3, :]) - \
                         2 * np.multiply(quaternions_interp[1, :], quaternions_interp[0, :])

    poses_mat[2, 0::4] = 2 * np.multiply(quaternions_interp[1, :], quaternions_interp[3, :]) - \
                         2 * np.multiply(quaternions_interp[2, :], quaternions_interp[0, :])
    poses_mat[2, 1::4] = 2 * np.multiply(quaternions_interp[2, :], quaternions_interp[3, :]) + \
                         2 * np.multiply(quaternions_interp[1, :], quaternions_interp[0, :])
    poses_mat[2, 2::4] = 1 - 2 * np.square(quaternions_interp[1, :]) - \
                         2 * np.square(quaternions_interp[2, :])

    poses_mat[0:3, 3::4] = positions_interp
    poses_mat[3, 3::4] = 1

    poses_mat = np.linalg.solve(poses_mat[0:4, 0:4], poses_mat)

    poses_out = [0] * (len(requested_timestamps) - 1)
    for i in range(1, len(requested_timestamps)):
        pose_mat = poses_mat[0:4, i * 4:(i + 1) * 4]
        pose_rot = pose_mat.copy()
        pose_rot[:3, -1] = 0
        pose_rot[-1, :3] = 0
        pose_position = pose_mat[:3, -1]
        pose_quaternion = tfs.quaternion_from_matrix(pose_rot, isprecise=True)  # [w x y z]
        poses_out[i - 1] = [requested_timestamps[i] / 1e6, -pose_position[0], -pose_position[1], pose_position[2],
                            -pose_quaternion[3], -pose_quaternion[2], pose_quaternion[1], pose_quaternion[0]]
        # poses_out[i - 1] = poses_mat[0:4, i * 4:(i + 1) * 4]

    return poses_out


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interpolate SE3 Pose given timestamp information.")
    parser.add_argument("--kf_pose", help="key frame pose in TUM file format.")
    parser.add_argument("--timestamps", help="timestamps file with index and timestamps for original rgb sequences.")
    args = parser.parse_args()
    main(args.kf_pose, args.timestamps)
