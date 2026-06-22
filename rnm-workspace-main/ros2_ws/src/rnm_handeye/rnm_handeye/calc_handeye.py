import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import os

def main(args = None):

    package_path = get_package_share_directory("rnm_handeye")

    sample_file = os.path.join(package_path, "config", "handeye_samples.npz")
    result_file = os.path.join(package_path, "config", "handeye_result.yml")


    # Load samples
    if not os.path.exists(sample_file):
        print("No handeye samples found.")
        return
    
    data = np.load(sample_file, allow_pickle=True)

    gripper2base = data["gripper2base_poses"]
    target2cam = data["target2cam_poses"]

    print(f"Loaded {len(gripper2base)} samples")


    # Split transformations
    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []

    for T in gripper2base:
        R_gripper2base.append(T[:3,:3])
        t_gripper2base.append(T[:3,3])

    for T in target2cam:
        R_target2cam.append(T[:3,:3])
        t_target2cam.append(T[:3,3])


    # Hand-eye calibration
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method=cv2.CALIB_HAND_EYE_TSAI)
    T_cam2gripper = np.eye(4)
    T_cam2gripper[:3,:3] = R_cam2gripper
    T_cam2gripper[:3,3] = t_cam2gripper.flatten()

    # Save result
    result = {
        "handeye_calibration":
        {
            "transformation_matrix": T_cam2gripper.tolist()
        }
    }

    with open(result_file, "w") as f:
        yaml.dump(result, f)

    print("Calibration calculated")
    print("Saved:", result_file)

if __name__ == "__main__":
    main()