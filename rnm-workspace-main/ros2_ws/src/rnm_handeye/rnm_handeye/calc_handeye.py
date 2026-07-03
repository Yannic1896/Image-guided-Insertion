import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import os

def calculate_handeye():

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
    t_cam2gripper = t_cam2gripper.flatten()
    T_cam2gripper = np.eye(4)
    T_cam2gripper[:3,:3] = R_cam2gripper
    T_cam2gripper[:3,3] = t_cam2gripper

    # Error metrics
    rotation_errors = []
    translation_errors = []
    for i in range(len(gripper2base)-1):
        for j in range(i+1, len(gripper2base)):
            B = np.linalg.inv(gripper2base[i]) @ gripper2base[j]
            R_B = B[:3,:3]
            t_B = B[:3,3]
            A = target2cam[i] @ np.linalg.inv(target2cam[j])
            R_A = A[:3,:3]
            t_A = A[:3,3]

            # Rotation error [Error = (R_X*R_B).T * (R_A*R_X)]
            rotation_error = (R_cam2gripper @ R_B).T @ (R_A @ R_cam2gripper)
            angle = np.arccos(np.clip((np.trace(rotation_error)-1)/2, -1.0, 1.0))
            rotation_errors.append(np.degrees(angle))
        
            # Translation errors [Error = norm(R_A*t_x -t_x -R_X*t_B +t_A)]
            translation_error = np.linalg.norm((R_A @ t_cam2gripper) - t_cam2gripper
                - (R_cam2gripper @ t_B) + t_A)
            translation_errors.append(translation_error)
    
    rotation_mean = np.mean(rotation_errors)
    rotation_std = np.std(rotation_errors, ddof=1)
    translation_mean = np.mean(translation_errors)
    translation_std = np.std(translation_errors, ddof=1)

    # Save result
    result = {
        "handeye_calibration": {
            "transformation_matrix": T_cam2gripper.tolist()
        },
        "translation_error": {
            "mean_m": float(translation_mean),
            "std_m": float(translation_std)
        },
        "rotation_error": {
            "mean_deg": float(rotation_mean),
            "std_deg": float(rotation_std)
        }
    }

    with open(result_file, "w") as f:
        yaml.dump(result, f)

    print("Calibration calculated")
    print("Saved:", result_file)

def main():
    calculate_handeye()

if __name__ == "__main__":
    main()