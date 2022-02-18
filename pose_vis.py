from evo.tools import file_interface
from evo.tools import plot
import matplotlib.pyplot as plt
import argparse
# temporarily override some package settings
from evo.tools.settings import SETTINGS

SETTINGS.plot_usetex = False
SETTINGS.plot_axis_marker_scale = 0.1

DROP_FRAMES = True  # drop some keyframes to make the visualization more illusive


def main(kf_pose, full_pose):
    traj_kf = file_interface.read_tum_trajectory_file(kf_pose)
    traj_full = file_interface.read_tum_trajectory_file(full_pose)
    fig = plt.figure()
    ax = plot.prepare_axis(fig, plot.PlotMode.xyz)
    traj_by_label = {
        "keyframe pose": traj_kf,
        "full-seq pose": traj_full
    }
    plot.traj(ax, plot.PlotMode.xyz, traj_kf,
              style=SETTINGS.plot_reference_linestyle,
              color=SETTINGS.plot_reference_color, label='keyframe pose',
              alpha=SETTINGS.plot_reference_alpha)
    plot.traj(ax, plot.PlotMode.xyz, traj_full,
              style=SETTINGS.plot_trajectory_linestyle,
              color='green', label='full-seq pose',
              alpha=SETTINGS.plot_reference_alpha)
    plot.draw_coordinate_axes(ax, traj_kf, plot.PlotMode.xyz,
                              SETTINGS.plot_axis_marker_scale)
    plot.draw_coordinate_axes(ax, traj_full, plot.PlotMode.xyz,
                              SETTINGS.plot_axis_marker_scale * 0.1)
    fig.axes.append(ax)
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize SE3 pose.")
    parser.add_argument("--kf_pose", help="key frame pose in TUM file format.")
    parser.add_argument("--full_pose", help="all-frame pose file in TUM format.")
    args = parser.parse_args()
    main(args.kf_pose, args.full_pose)
