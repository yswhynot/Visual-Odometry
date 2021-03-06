//#include <stdio.h>
//#include <iostream>
//#include <fstream>
//#include <time.h>
//#include <string>
//#include "opencv2/opencv.hpp"
//#include "opencv2/core.hpp"
//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/features2d.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/calib3d.hpp"
//#include "opencv2/xfeatures2d.hpp"
//#include <opencv2/viz.hpp>
//
//using namespace cv;
//using namespace std;
//
//const int IMG_WIDTH = 800;
//const int IMG_HEIGHT = 600;
//
//void readme();
//
//void constructProjectionMat(Mat& P1, Mat& P2, Mat& cam_intrinsic,
//		vector<Affine3d>& path, int img_i) {
//	for (int i = 0; i < img_i; i++) {
//		vector<Mat> pathi;
//		split(path[i].matrix, pathi);
//		P1 = pathi[0] * P1;
//		P2 = pathi[0] * P2;
//	}
//	P1.pop_back(1);
//	P1 = cam_intrinsic * P1;
//	vector<Mat> pathi;
//	split(path[img_i].matrix, pathi);
//	P2 = pathi[0] * P2;
//	P2.pop_back(1);
//	P2 = cam_intrinsic * P2;
//	cout << "P1: " << P1 << endl << "P2: " << P2 << endl;
//}
//
//int main(int argc, char** argv) {
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
//	Mat img_curr_origin, img_curr, img_prev_origin, img_prev;
//	img_prev_origin = imread("trans_img/s0.jpg", CV_BGR2GRAY);
//
//	// init the prev variables
//	vector<KeyPoint> keypoints_prev;
//	vector<Point2f> good_prev;
//	Mat descriptors_prev;
//
//	// init detection and extraction container
//	Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(50);
//	Ptr<xfeatures2d::SURF> extractor = xfeatures2d::SURF::create();
//	FlannBasedMatcher matcher;
//
//	// rotation & translation arrays
//	vector<Mat> R_vec, t_vec;
//
//	Mat R, t, P1, P2, R1, R2, Q;
//	vector<Vec3f> point_cloud_est;
//	vector<Affine3d> path;
//
//	clock_t c_begin = clock();
//
//	for (int img_i = 0; img_i < 21; img_i++) {
//		stringstream ss;
//		ss << "trans_img/s" << img_i << ".jpg";
//		cout << "current idx: " << ss.str() << endl;
//		img_curr_origin = imread(ss.str(), CV_BGR2GRAY);
//
//		// init the img_prev parameters
//		if (img_i == 0) {
//			undistort(img_prev_origin, img_prev, cam_intrinsic, cam_distortion);
//			detector->detect(img_prev, keypoints_prev);
//			extractor->compute(img_prev, keypoints_prev, descriptors_prev);
//			Mat img_keypoints_prev;
//			drawKeypoints(img_prev, keypoints_prev, img_keypoints_prev,
//					Scalar::all(-1), DrawMatchesFlags::DEFAULT);
//			//			imwrite("output/img/s0.jpg", img_keypoints_prev);
//
//			R = Mat::eye(3, 3, CV_64F);
//			t = Mat::zeros(3, 1, CV_64F);
//			R_vec.push_back(R);
//			t_vec.push_back(t);
//			path.push_back(Affine3d(R, t));
//			continue;
//		}
//
//		clock_t c_feature, c_extractor, c_match, c_homo;
//
//		// step 0: undistorted img
//		undistort(img_curr_origin, img_curr, cam_intrinsic, cam_distortion);
//
//		// step 1: detect features
//		c_feature = clock();
//		vector<KeyPoint> keypoints_curr;
//		detector->detect(img_curr, keypoints_curr);
//		printf("Feature Detection time: %f seconds\n",
//				(float) (clock() - c_feature) / CLOCKS_PER_SEC);
//
//		// step 2: descriptor
//		c_extractor = clock();
//		Mat descriptors_curr;
//		extractor->compute(img_curr, keypoints_curr, descriptors_curr);
//
//		printf("Descriptor Extraction time: %f seconds\n",
//				(float) (clock() - c_extractor) / CLOCKS_PER_SEC);
//
//		// step 3: FLANN matcher
//		c_match = clock();
//		vector<DMatch> matches;
//		matcher.match(descriptors_curr, descriptors_prev, matches);
//
//		printf("Match time: %f seconds\n",
//				(float) (clock() - c_match) / CLOCKS_PER_SEC);
//
//		c_homo = clock();
//		// key points distance
//		double min_dis = 100;
//		for (int i = 0; i < descriptors_curr.rows; i++) {
//			if (matches[i].distance < min_dis)
//				min_dis = matches[i].distance;
//		}
//
//		vector<DMatch> good_matches;
//		for (int i = 0; i < descriptors_curr.rows; i++) {
//			if (matches[i].distance < 3 * min_dis)
//				good_matches.push_back(matches[i]);
//		}
//
//		vector<Point2f> good_curr, good_prev;
//
//		for (size_t i = 0; i < good_matches.size(); i++) {
//			//-- Get the keypoints from the good matches
//			Point2f kp_curr = keypoints_curr[good_matches[i].queryIdx].pt;
//			Point2f kp_prev = keypoints_prev[good_matches[i].trainIdx].pt;
//			good_curr.push_back(kp_curr);
//			good_prev.push_back(kp_prev);
//		}
//
//		Mat img_keypoints_curr;
//		drawKeypoints(img_curr, keypoints_curr, img_keypoints_curr,
//				Scalar::all(-1), DrawMatchesFlags::DEFAULT);
//		imshow("Features", img_keypoints_curr);
//		string output = "output/";
//		output += ss.str();
//
//		try {
//			Mat E = findEssentialMat(
//					good_prev,
//					good_curr,
//					cam_intrinsic.at<double> (0, 0),
//					Point2f(cam_intrinsic.at<double> (0, 2),
//							cam_intrinsic.at<double> (1, 2)));
//
//			recoverPose(
//					E,
//					good_prev,
//					good_curr,
//					R,
//					t,
//					cam_intrinsic.at<double> (0, 0),
//					Point2f(cam_intrinsic.at<double> (0, 2),
//							cam_intrinsic.at<double> (1, 2)));
//
////			if(img_i > 15)
////				t = Mat::zeros(3, 1, CV_64F);
//
//			R_vec.push_back(R);
//			t_vec.push_back(t);
//
//			path.push_back(Affine3d(R, t));
//
//			Mat point4d_homo;
//			P1 = Mat::eye(4, 4, CV_64F);
//			P2 = Mat::eye(4, 4, CV_64F);
//			constructProjectionMat(P1, P2, cam_intrinsic, path, img_i);
//			triangulatePoints(P1, P2, good_prev, good_curr, point4d_homo);
//
//			for (int i = 0; i < point4d_homo.cols; i++) {
//				float z_index = point4d_homo.at<float> (3, i);
//				point_cloud_est.push_back(
//						Vec3f(point4d_homo.at<float> (0, i) / z_index,
//								point4d_homo.at<float> (1, i) / z_index,
//								point4d_homo.at<float> (2, i) / z_index));
//			}
//
//			cout << "R: " << R << endl;
//			cout << "t: " << t << endl;
//
//		} catch (...) {
//		}
//
//		printf("Total time: %f seconds\n",
//				(float) (clock() - c_feature) / CLOCKS_PER_SEC);
//
//		// update all prev parameters
//		keypoints_prev = keypoints_curr;
//		descriptors_curr.copyTo(descriptors_prev);
//
//		if (waitKey(30) >= 0)
//			break;
//	}
//
//	viz::Viz3d window("Coordinate Frame");
//	window.setWindowSize(Size(500, 500));
//	window.setWindowPosition(Point(150, 150));
//	window.setBackgroundColor(); // black by default
//
//	Matx33d K = Matx33d(cam_intrinsic.at<double> (0, 0), 0,
//			cam_intrinsic.at<double> (0, 2), 0,
//			cam_intrinsic.at<double> (0, 0), cam_intrinsic.at<double> (1, 2),
//			0, 0, 1);
//
//	viz::WCloud cloud_widget(point_cloud_est, viz::Color::green());
//	window.showWidget("point_cloud", cloud_widget);
//	window.showWidget(
//			"cameras_frames_and_lines",
//			viz::WTrajectory(path, viz::WTrajectory::BOTH, 0.1,
//					viz::Color::green()));
//	window.showWidget("cameras_frustums",
//			viz::WTrajectoryFrustums(path, K, 0.1, viz::Color::yellow()));
//	window.setViewerPose(path[0]);
//
//	cout << "Total: " << (float) (clock() - c_begin) / CLOCKS_PER_SEC << endl;
//
//	window.spin();
//
//	return 0;
//}
//
///** @function readme */
//void readme() {
//	cout << " Usage: ./SURF_detector <img1> <img2>" << endl;
//}
