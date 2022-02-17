import numpy as np

def random_sample(input_arr, sample_size):
    assert len(input_arr) > sample_size + 1
    indice = np.sort(np.random.choice(list(np.arange(input_arr.shape[0])), sample_size, replace=False))
    np.insert(indice, 0, 0)
    np.insert(indice, -1, len(input_arr) - 1)
    indice = np.unique(indice)
    output_arr = input_arr[indice]
    return output_arr



if __name__ == "__main__":
    kf_pose_fname = "./data/kf_pose_result_tum.txt"
    kf_pose = np.genfromtxt(kf_pose_fname)
    kf_pose_out = random_sample(kf_pose, 256)
    kf_pose_out_fname = "./data/kf_pose_result_tum_vis.txt"
    np.savetxt(kf_pose_out_fname, kf_pose_out, fmt="%1.6f")
