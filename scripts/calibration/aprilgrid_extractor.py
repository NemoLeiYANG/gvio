#!/usr/bin/env python
import os
import re
import time
import argparse

import aslam_cv as acv
import aslam_cv_backend as acvb
import aslam_cameras_april as acv_april
import kalibr_common as kc

import cv2
import numpy as np

# make numpy print prettier
np.set_printoptions(suppress=True)


def atoi(text):
    return int(text) if text.isdigit() else text.lower()


def natural_keys(text):
    '''
    alist.sort(key=natural_keys) sorts in human order
    http://nedbatchelder.com/blog/200712/human_sorting.html
    (See Toothy's implementation in the comments)
    '''
    return [atoi(c) for c in re.split('(\d+)', text)]


def create_directory(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)


class CalibrationTargetDetector(object):
    def __init__(self, camera_config, target_config):
        target_params = target_config.getTargetParams()
        target_type = target_config.getTargetType()

        # Set up target
        if target_type == 'checkerboard':
            grid = acv.GridCalibrationTargetCheckerboard(
                target_params['targetRows'],
                target_params['targetCols'],
                target_params['rowSpacingMeters'],
                target_params['colSpacingMeters']
            )

        elif target_type == 'circlegrid':
            options = acv.CirclegridOptions()
            options.useAsymmetricCirclegrid = target_params['asymmetricGrid']

            grid = acv.GridCalibrationTargetCirclegrid(
                target_params['targetRows'],
                target_params['targetCols'],
                target_params['spacingMeters'],
                options
            )

        elif target_type == 'aprilgrid':
            grid = acv_april.GridCalibrationTargetAprilgrid(
                target_params['tagRows'],
                target_params['tagCols'],
                target_params['tagSize'],
                target_params['tagSpacing']
            )

        else:
            raise RuntimeError("Unknown calibration target.")

        # Setup detector
        options = acv.GridDetectorOptions()
        options.filterCornerOutliers = True
        self.detector = acv.GridDetector(camera_config.geometry, grid, options)


class MonoCameraValidator(object):
    def __init__(self, camera_config, target_config):
        print("initializing camera geometry")
        self.camera = kc.ConfigReader.AslamCamera.fromParameters(camera_config)
        self.target = CalibrationTargetDetector(self.camera, target_config)

        self.topic = camera_config.getRosTopic()
        self.windowName = "Camera: {0}".format(self.topic)
        cv2.namedWindow(self.windowName, 1)

        # Create image undistorter
        alpha = 1.0
        scale = 1.0
        self.undistorter = self.camera.undistorterType(self.camera.geometry,
                                                       cv2.INTER_LINEAR,
                                                       alpha, scale)
        if type(self.camera.geometry) == acv.DistortedOmniCameraGeometry:
            self.undist_camera = self.undistorter.getIdealPinholeGeometry()
        else:
            self.undist_camera = self.undistorter.getIdealGeometry()

        # Storage for reproj errors
        self.cornersImage = list()
        self.reprojectionErrors = list()

    def generateMonoview(self, np_image, obs, obs_valid):
        np_image = cv2.cvtColor(np_image, cv2.COLOR_GRAY2BGR)

        if obs_valid:
            # Calculate the reprojection error statistics
            cornersImage = obs.getCornersImageFrame()
            cornersReproj = obs.getCornerReprojection(self.camera.geometry)
            reprojectionErrors2 = cornersImage - cornersReproj
            reprojectionErrors = np.sum(np.abs(reprojectionErrors2)**2, axis=-1)**(1.0 / 2.0)

            # Save the errors for reprojection error map plotting
            self.cornersImage.append(cornersImage)
            self.reprojectionErrors.append(reprojectionErrors)

            outputList = [("mean_x:  ", np.mean(reprojectionErrors2[:,0])),
                          ("std_x:   ", np.std(reprojectionErrors2[:,0])),
                          ("max_y:   ", np.max(reprojectionErrors2[:,0])),
                          ("min_x:   ", np.min(reprojectionErrors2[:,0])),
                          ("", 0),
                          ("mean_y:  ", np.mean(reprojectionErrors2[:,1])),
                          ("std_y:   ", np.std(reprojectionErrors2[:,1])),
                          ("max_y:   ", np.max(reprojectionErrors2[:,1])),
                          ("min_y:   ", np.min(reprojectionErrors2[:,1])),
                          ("", 0),
                          ("mean_L2: ", np.mean(reprojectionErrors)),
                          ("std_L2:  ", np.std(reprojectionErrors)),
                          ("max_L2:  ", np.max(reprojectionErrors)),
                          ("min_L2:  ", np.min(reprojectionErrors))]
            meanReprojectionError = np.mean(reprojectionErrors)

            # Print the text
            x = 20
            y = 20
            for err_txt, err_val in outputList:
                fontScale = 0.75
                y += int(42*fontScale)
                if err_txt == "":
                    continue
                cv2.putText(np_image, err_txt, (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, fontScale=fontScale,
                            color=(0, 0, 255), thickness=2)
                cv2.putText(np_image, "{0: .4f}".format(err_val), (x + 100, y),
                            cv2.FONT_HERSHEY_SIMPLEX, fontScale=fontScale,
                            color=(0, 0, 255), thickness=2)

            # Draw reprojected corners
            for px, py in zip(cornersReproj[:,0],cornersReproj[:,1]):
                # Convert pixel to fixed point (opencv subpixel rendering...)
                shift = 4; radius = 0.5; thickness = 1
                px_fix =  int(px * 2**shift)
                py_fix =  int(py * 2**shift)
                radius_fix = int(radius * 2**shift)
                cv2.circle(np_image, (px_fix, py_fix), radius=radius_fix,
                           color=(255, 255, 0), thickness=thickness,
                           shift=shift)

        else:
            cv2.putText(np_image, "Detection failed...",
                        (np_image.shape[0] / 2, np_image.shape[1] / 5),
                        cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.5,
                        color=(0, 0, 255), thickness=2)
        cv2.imshow(self.windowName, np_image)
        cv2.waitKey(1)

        if obs_valid:
            return [cornersImage, cornersReproj, meanReprojectionError]


class AprilGridExtractor(object):
    def __init__(self, cam_path, cam_index, cam_chain, target, imu_angles_path):
        self.cam_path = cam_path
        self.camera_config = cam_chain.getCameraParameters(cam_index)
        self.cam_index = cam_index
        self.mono_validator = MonoCameraValidator(self.camera_config, target)
        self.imu_angles = np.loadtxt(imu_angles_path, delimiter=',', usecols=(0,1))

    def extract(self):
        # Load image paths
        print("Loading camera images from: " + self.cam_path)
        cam_files = [f for f in os.listdir(self.cam_path) if os.path.isfile(os.path.join(self.cam_path, f))]
        cam_files = sorted(cam_files, key=natural_keys)

        file_count = 0
        filename_num = file_count + 1
        for f in cam_files:
            image_path = os.path.join(self.cam_path, f)
            image = cv2.imread(image_path, 0)
            np_image = np.array(image)

            # Detect target for camera
            split_file = f.rsplit('.', 1)
            timestamp = acv.Time(0, 0)
            success, observation = self.mono_validator.target.detector.findTarget(timestamp, np_image)
            observation.clearImage()

            if success:
                # Display image
                print("Processing image" + ' ' + str(filename_num))

                # Get imu angle
                curr_imu_angle = self.imu_angles[file_count, 0:]
                [image_corners, reproj_corners, meanReprojectionError] = self.mono_validator.generateMonoview(
                    np_image,
                    observation,
                    success
                )
                target_corners = observation.getCornersTargetFrame()
                T_t_c = observation.T_t_c()
                T_c_t = np.linalg.inv(T_t_c.T())

                # Create output directory
                output_path = self.cam_path + "/points_data"
                create_directory(output_path)

                # Create output file
                file_name = str(filename_num) + ".txt"
                output_path = os.path.join(output_path, file_name)
                outfile = open(output_path,"w")
                # -- Output Gridpoints
                outfile.write("gridpoints:\n")
                for corner, pixel in zip(target_corners,image_corners):
                    line = "{:.5f}".format(corner[0]) + " "
                    line += "{:.5f}".format(corner[1]) + " "
                    line += "{:.5f}".format(corner[2]) + " "
                    line += "{:.5f}".format(pixel[0]) + " "
                    line += "{:.5f}".format(pixel[1]) + "\n"
                    outfile.write(line)
                # -- Ouput Transform
                outfile.write("tmatrix:\n")
                tmatrix = np.array(T_c_t)
                tmatrix_str = '\n'.join(" ".join('%0.5f' % (x) for x in y) for y in tmatrix)
                outfile.write(tmatrix_str)
                # -- Output Gimbal angles
                outfile.write("\ngimbalangles:\n")
                for i in range(len(curr_imu_angle)):
                    outfile.write(str(curr_imu_angle[i]) + ' ')
                outfile.write("\n")

                # outfile.write("reprojectionerror:\n")
                # outfile.write(str(np.mean(meanReprojectionError)))

                # Finish output
                outfile.write("end:")
                outfile.close()

            file_count += 1
            filename_num += 1


if __name__ == "__main__":
    # Parse CLI args
    parser = argparse.ArgumentParser(description='Extract AprilGrid data from camera images')
    parser.add_argument('--target', dest='target', help='Calibration target configuration as yaml file', required=True)
    parser.add_argument('--camchain', dest='camchain', help='Camera configuration as yaml file', required=True)
    parser.add_argument('--verbose', action='store_true', dest='verbose', help='Verbose output')
    parser.add_argument('--cam_num', dest='cam_num', help='The camera number from the intrinsic file', required=True)
    parser.add_argument('--cam_path', dest='cam_path', help='The location of the image files', required=True)
    parser.add_argument('--joint_path', dest='joint_path', help='Gimbal joint data', required=True)
    args = parser.parse_args()

    # Load calibration target
    print("Initializing calibration target config")
    target = kc.ConfigReader.CalibrationTargetParameters(args.target)

    # Load camera chain
    print("Initializing camchain config")
    camchain = kc.ConfigReader.CameraChainParameters(args.camchain)

    # Extract aprilgrid cornerss
    extractor = AprilGridExtractor(args.cam_path,
                                   int(args.cam_num),
                                   camchain,
                                   target,
                                   args.joint_path)
    extractor.extract()
