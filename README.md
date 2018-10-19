# A camera pose estimation demo.
In order to get camera pose, you need:
1. Calibrate you camera and get it's intrinsic and extrinsic parameters
2. Define 4 points on both real world and image (e.g. 4 corners of chessboard), which will be used to define image_points and object_points in program.
3. Use cv::solvePnP sovle the rotation and translation matrices. Note the tranlation and rotation values are based on camera system unit(Xc,Yc,Zc defined [here](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#void%20Rodrigues(InputArray%20src,%20OutputArray%20dst,%20OutputArray%20jacobian)))