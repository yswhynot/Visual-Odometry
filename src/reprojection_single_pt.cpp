///*
// * reprojection_single_pt.cpp
// *
// *  Created on: Jul 1, 2016
// *      Author: yisha
// */
//
//#include <stdio.h>
//#include <iostream>
//#include <fstream>
//#include <time.h>
//#include <string>
//#include <math.h>
//#include <stdlib.h>
//
//#include <opencv2/sfm.hpp>
//#include "opencv2/sfm/robust.hpp"
//#include "opencv2/opencv.hpp"
//#include "opencv2/core.hpp"
//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/videoio.hpp"
//#include "opencv2/features2d.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/calib3d.hpp"
//#include "opencv2/xfeatures2d.hpp"
//#include "opencv2/viz.hpp"
//
//#include "cvsba/cvsba.h"
//
//using namespace cv;
//using namespace cv::sfm;
//using namespace std;
//
//int main(int argc, char** argv) {
//	Vec3f pt3d(100, 200, 50);
//	Mat pt2d_m1, pt2d_m2;
//	Mat R, R_tmp, t, t_tmp, R1, t1, R2, t2;
//	Mat pt3d_h, pt2d_h1, pt2d_h2;
//	Mat P1, P2, P;
//
//	vector<Point2d> pv1, pv2;
//	Mat result_p3;
//
//	// load camera intrinsic matrix
//	Mat cam_intrinsic, cam_distortion;
//	FileStorage fs("cam_info/201606071648.yml", FileStorage::READ);
//	fs["camera_matrix"] >> cam_intrinsic;
//	fs["distortion_coefficients"] >> cam_distortion;
//	fs.release();
//	cam_intrinsic.convertTo(cam_intrinsic, CV_64F);
//	cam_distortion.convertTo(cam_distortion, CV_64F);
//
//	R1 = Mat::eye(3, 3, CV_64F);
//	t1 = Mat::zeros(3, 1, CV_64F);
//	t = Mat::zeros(3, 1, CV_64F);
//
//	R_tmp = Mat::zeros(3, 1, CV_64F);
//	R_tmp.at<double> (0, 0) = 0.1;
//	R_tmp.at<double> (1, 0) = 0.2;
//	R_tmp.at<double> (2, 0) = 0.3;
//	Rodrigues(R_tmp, R);
//	t.at<double> (0, 0) = 0.5;
//	t.at<double> (1, 0) = 0.2;
//
//	R2 = R * R1;
//	t2 = R * t1 + t;
//
//	cout << "R: " << R << endl;
//	cout << "t: " << t << endl;
//
//	projectionFromKRt(cam_intrinsic, R1, t1, P1);
//	projectionFromKRt(cam_intrinsic, R2, t2, P2);
//	projectionFromKRt(cam_intrinsic, R, t, P);
//	cout << "P1: " << P1 << endl;
//	cout << "P2: " << P2 << endl;
//	cout << "P: " << P << endl;
//
//	euclideanToHomogeneous(pt3d, pt3d_h);
//	pt3d_h.convertTo(pt3d_h, CV_64F);
//	pt2d_h1 = P1 * pt3d_h;
//	pt2d_h2 = P2 * pt3d_h;
//	homogeneousToEuclidean(pt2d_h1, pt2d_m1);
//	homogeneousToEuclidean(pt2d_h2, pt2d_m2);
//	Point2d pt2d1(pt2d_m1.at<double>(0, 0), pt2d_m1.at<double>(0, 1));
//	Point2d pt2d2(pt2d_m2.at<double>(0, 0), pt2d_m2.at<double>(0, 1));
//	pv1.push_back(pt2d1);
//	pv2.push_back(pt2d2);
//
//	triangulatePoints(P1, P2, pv1, pv2, result_p3);
//
//	Mat result_h = result_p3.col(0);
//	Mat result_eu;
//	homogeneousToEuclidean(result_h, result_eu);
//
//	cout << "P origin:  " << pt3d << endl;
//	cout << "P measure: " << result_eu << endl;
//
//}
