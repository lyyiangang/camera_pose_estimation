/*
get camera pose from for points on ground. Note the final rotation angle and tranlation value is in camera coordinate system unit(Xc, Yc, Zc in below link)
https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#void%20Rodrigues(InputArray%20src,%20OutputArray%20dst,%20OutputArray%20jacobian)

*/
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
    
int main(int argc, char *argv[]) {
  int board_w = 315;
  int board_h = 315;
  cv::FileStorage fs("./cam_intrinsic.xml", cv::FileStorage::READ);
  cv::Mat intrinsic, distortion;

  fs["Intrinsics"] >> intrinsic;
  fs["Distortion"] >> distortion;

  if (!fs.isOpened() || intrinsic.empty() || distortion.empty()) {
    cout << "Error: Couldn't load intrinsic parameters"<< endl;
    return -1;
  }
  fs.release();

  cv::Mat rawImg = cv::imread("./glass_chessboard.png", 1);
  if (rawImg.empty()) {
    cout << "Error: Couldn't load image "  << endl;
    return -1;
  }

  vector<cv::Point2f> corners;
  corners.push_back({ 1146, 783});//left top
  corners.push_back({ 1264, 776});// right top
  corners.push_back({ 1175, 857});//left bottom
  corners.push_back({ 1307, 846});//right bottom

  // GET THE IMAGE AND OBJECT POINTS:
  // Object points are at (r,c):
  // (0,0), (board_w-1,0), (0,board_h-1), (board_w-1,board_h-1)
  // That means corners are at: corners[r*board_w + c]
  //
  cv::Point2f objPts[4], imgPts[4];
  //left top
  objPts[0].x = 0;
  objPts[0].y = 0;

  //right top
  objPts[1].x = board_w - 1;
  objPts[1].y = 0;

  //left bottom
  objPts[2].x = 0;
  objPts[2].y = board_h - 1;

  //right bottom
  objPts[3].x = board_w - 1;
  objPts[3].y = board_h - 1;

  imgPts[0] = corners[0];
  imgPts[1] = corners[1];
  imgPts[2] = corners[2];
  imgPts[3] = corners[3];

  // DRAW THE POINTS in order: B,G,R,YELLOW
  //
  cv::circle(rawImg, imgPts[0], 9, cv::Scalar(255, 0, 0), 3);
  cv::circle(rawImg, imgPts[1], 9, cv::Scalar(0, 255, 0), 3);
  cv::circle(rawImg, imgPts[2], 9, cv::Scalar(0, 0, 255), 3);
  cv::circle(rawImg, imgPts[3], 9, cv::Scalar(0, 255, 255), 3);

  vector<cv::Point2f> image_points;
  vector<cv::Point3f> object_points;
  for (int i = 0; i < 4; ++i) {
    image_points.push_back(imgPts[i]);
    object_points.push_back(cv::Point3f(objPts[i].x, objPts[i].y, 0));
  }
  cv::Mat  rmat;

  cv::Mat otherTVec, rvec;
  cv::solvePnP(object_points, 	// 3-d points in object coordinate
      image_points,  	// 2-d points in image coordinates
      intrinsic,     	// Our camera matrix
      distortion,     	// Since we corrected distortion in the
                      // beginning,now we have zero distortion
                      // coefficients
      rvec, 			// Output rotation *vector*.
      otherTVec  			// Output translation vector.
  );
  cv::Rodrigues(rvec, rmat);

  cout << "rotation matrix: " << rmat << endl;
  cout << "translation vector: " << otherTVec << endl;
  return 1;
}
