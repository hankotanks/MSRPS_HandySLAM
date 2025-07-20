import os, sys, csv, json

import numpy as np
import matplotlib.pyplot as plt

from scipy.spatial.transform import Rotation as R

def find_closest_pose(poses, target_timestamp):
    return min(poses, key=lambda p: abs(p[0] - target_timestamp))

def rotation(deg, axis = 'x'):
    axes = {
        'x': [1, 0, 0],
        'y': [0, 1, 0],
        'z': [0, 0, 1]
    }

    rot = R.from_rotvec(np.deg2rad(deg) * np.array(axes[axis]))
    rot_mat_3x3 = rot.as_matrix()

    t = np.eye(4)
    t[:3, :3] = rot_mat_3x3

    return t

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print(f'USAGE: {sys.argv[0]} <scene-path> <out-path> <name>')
        sys.exit(1)

    name = sys.argv[3]

    path_gt = os.path.join(sys.argv[1], 'iphone', 'pose_intrinsic_imu.json')
    if not os.path.exists(path_gt):
        print(f'ERROR: {path_gt} does not exist.')
        sys.exit(1)

    path_times = os.path.join(sys.argv[2], 'stamps.csv')
    path_poses = os.path.join(sys.argv[2], 'poses.csv')

    if not os.path.exists(path_times):
        print(f'ERROR: {path_times} does not exist.')
        sys.exit(1)

    if not os.path.exists(path_poses):
        print(f'ERROR: {path_poses} does not exist.')
        sys.exit(1)

    poses_gt = []
    with open(path_gt, 'r') as f:
        data = json.load(f)
        for frame_key in sorted(data.keys()):
            frame = data[frame_key]

            pose = np.array(frame['aligned_pose'], dtype=np.float64)
            poses_gt.append((frame['timestamp'], pose))

    times_map = {}
    with open(path_times, newline = '') as f:
        reader_times = csv.reader(f)

        for row in reader_times:
            times_map[int(row[0])] = float(row[1])

    poses_unsorted = []
    with open(path_poses, newline = '') as f:
        _, base_gt = poses_gt[0]
        base_gt = base_gt @ rotation(90.0, 'z') @ rotation(-90.0, 'y')

        reader_poses = csv.reader(f)
        for row in reader_poses:
            row = [float(x.strip()) for x in row]

            index = int(row[0])
            tx, ty, tz = row[1:4]
            qx, qy, qz, qw = row[4:8]
            
            rot = R.from_quat([qx, qy, qz, qw]).as_matrix()
            pose = np.eye(4)
            pose[:3, :3] = rot
            pose[:3, 3] = [tx, ty, tz]

            poses_unsorted.append((times_map[index], base_gt @ pose))

    poses = sorted(poses_unsorted, key = lambda pose: pose[0])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    poses_pos = np.array([pose[1][:3, 3] for pose in poses])
    poses_pos_gt = np.array([pose[1][:3, 3] for pose in poses_gt])

    ax.plot(poses_pos[:, 0], poses_pos[:, 1], poses_pos[:, 2], 'r-', label=f'{name}')
    ax.plot(poses_pos_gt[:, 0], poses_pos_gt[:, 1], poses_pos_gt[:, 2], 'b-', label='ground truth')

    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.set_title(f'Trajectories [{name}]')
    ax.legend()
    plt.show()

    timestamps = []
    errors_x, errors_y, errors_z = [], [], []

    for timestamp_est, pose_est in poses:
        closest_timestamp, closest_pose_gt = find_closest_pose(poses_gt, timestamp_est)

        pos_est = pose_est[:3, 3]
        pos_gt = closest_pose_gt[:3, 3]
        error = pos_est - pos_gt

        timestamps.append(timestamp_est)
        errors_x.append(error[0])
        errors_y.append(error[1])
        errors_z.append(error[2])

    errors_x, errors_y, errors_z = np.array(errors_x), np.array(errors_y), np.array(errors_z)

    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    axs[0].plot(timestamps, errors_x, label='X', color='r')
    axs[0].set_ylabel('x [m]')
    axs[0].grid()

    axs[1].plot(timestamps, errors_y, label='Y', color='g')
    axs[1].set_ylabel('y [m]')
    axs[1].grid()

    axs[2].plot(timestamps, errors_z, label='Z', color='b')
    axs[2].set_ylabel('z [m]')
    axs[2].set_xlabel('Timestamp')
    axs[2].grid()

    plt.suptitle(f'Positional Drift [{name}]')
    plt.tight_layout()
    plt.show()


    print('avg [m]:', np.mean(np.sqrt(errors_x**2 + errors_y**2 + errors_z**2)))
    print('std [m]:', np.std(np.sqrt(errors_x**2 + errors_y**2 + errors_z**2)))
