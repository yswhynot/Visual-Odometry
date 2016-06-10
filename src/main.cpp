#include <stdio.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;

void readme();

//int main(int argc, char** argv) {
//
//	namedWindow("Features matches", 1);
////	namedWindow("Features current", 1);
//
//	// Camera intrinsic matrix
//	Mat cam_intrinsic, cam_distortion;
//
//	FileStorage fs("cam_info/201606071648.yml", FileStorage::READ);
//	fs["camera_matrix"] >> cam_intrinsic;
//	fs["distortion_coefficients"] >> cam_distortion;
//	fs.release();
//	cam_intrinsic.convertTo(cam_intrinsic, CV_64F);
//	cam_distortion.convertTo(cam_distortion, CV_64F);
//	std::cout << "cam_intrinsic: " << cam_intrinsic << std::endl;
//	std::cout << "cam_distortion: " << cam_distortion << std::endl;
//
//	Mat img_curr_origin, img_curr, img_prev_origin, img_prev;
//	img_prev_origin = imread("img/test1.jpg", CV_BGR2GRAY);
//	img_curr_origin = imread("img/test2.jpg", CV_BGR2GRAY);
//
//	clock_t c_feature, c_extractor, c_match, c_homo;
//
//	// step 0: undistorted img
//	undistort(img_prev_origin, img_prev, cam_intrinsic, cam_distortion);
//	undistort(img_curr_origin, img_curr, cam_intrinsic, cam_distortion);
//
//	// step 1
//	c_feature = clock();
//	FastFeatureDetector detector(15);
//	std::vector<KeyPoint> keypoints_curr, keypoints_prev;
//
//	detector.detect(img_prev, keypoints_prev);
//	detector.detect(img_curr, keypoints_curr);
//
//	printf("Feature Detection time: %f seconds\n",
//			(float) (clock() - c_feature) / CLOCKS_PER_SEC);
//
//	Mat img_keypoints_curr, img_keypoints_prev;
//
//	// step 2: descriptor
//	c_extractor = clock();
//	SurfDescriptorExtractor extractor;
//	Mat descriptors_curr, descriptors_prev;
//	extractor.compute(img_prev, keypoints_prev, descriptors_prev);
//	extractor.compute(img_curr, keypoints_curr, descriptors_curr);
//
//	printf("Descriptor Extraction time: %f seconds\n",
//			(float) (clock() - c_extractor) / CLOCKS_PER_SEC);
//
//	// step 3: FLANN matcher
//	c_match = clock();
//
//	FlannBasedMatcher matcher;
//	std::vector<DMatch> matches;
//	matcher.match(descriptors_curr, descriptors_prev, matches);
//
//	printf("Match time: %f seconds\n",
//			(float) (clock() - c_match) / CLOCKS_PER_SEC);
//
//	c_homo = clock();
//	// key points distance
//	double min_dis = 100;
//	for (int i = 0; i < descriptors_curr.rows; i++) {
//		if (matches[i].distance < min_dis)
//			min_dis = matches[i].distance;
//	}
//
//	std::vector<DMatch> good_matches;
//
//	for (int i = 0; i < descriptors_curr.rows; i++) {
//		if (matches[i].distance < 3 * min_dis)
//			good_matches.push_back(matches[i]);
//	}
//
//	std::vector<Point2f> good_curr;
//	std::vector<Point2f> good_prev;
//
//	for (int i = 0; i < good_matches.size(); i++) {
//		//-- Get the keypoints from the good matches
//		good_curr.push_back(keypoints_curr[good_matches[i].queryIdx].pt);
//		good_prev.push_back(keypoints_prev[good_matches[i].trainIdx].pt);
//	}
//
//	Mat img_matches;
//	drawMatches(img_prev, keypoints_prev, img_curr, keypoints_curr, good_matches,
//			img_matches, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);;
//	imshow("Features matches", img_matches);
////	imshow("Features current", img_keypoints_curr);
//
//	try {
//		// step 4: find fundamental Mat and rectification
//		Mat F = findFundamentalMat(good_prev, good_curr, CV_RANSAC);
//
//		Mat H = findHomography(good_prev, good_curr, CV_RANSAC);
//
//		std::cout << "good_curr: " << good_curr << std::endl;
//		std::cout << "good_prev: " << good_prev << std::endl;
//
//		std::cout << "H: " << H << std::endl;
//		std::cout << "F: " << F << std::endl;
//
//		printf("Fundamental Mat time: %f seconds\n",
//				(float) (clock() - c_homo) / CLOCKS_PER_SEC);
//
//		//			// step 5: correspondance
//		//			Mat disparity_map;
//		//			StereoBM(good_prev, good_curr, disparity_map);
//
//		// step 5: compute R, t - SVD
//		Mat E = cam_intrinsic.t() * F * cam_intrinsic;
//		std::cout << "E: " << E << std::endl;
//		Mat w_value, u, vt;
//		Mat w = Mat::zeros(3, 3, CV_64F);
//		SVD::compute(E, w_value, u, vt);
//		w.at<double> (0, 0) = w_value.at<double> (0, 0);
//		w.at<double> (1, 1) = w_value.at<double> (1, 0);
//		w.at<double> (2, 2) = w_value.at<double> (2, 0);
//		//				std::cout << "SVD: " << u * w * vt << std::endl;
//		Mat R1 = u * w * vt;
//		Mat R2 = u * w.t() * vt;
//		Mat t1 = u.col(2);
//		Mat t2 = -u.col(2);
//
//		std::cout << "R1: " << R1 << std::endl;
//		std::cout << "R2: " << R2 << std::endl;
//		std::cout << "t1: " << t1 << std::endl;
//		std::cout << "t2: " << t2 << std::endl;
//
//		Mat p(3, 1, CV_64F, Scalar(1));
//		printf("r1 t1:\n");
//		std::cout << R1 * p + t1 << std::endl;
//		printf("r1 t2:\n");
//		std::cout << R1 * p + t2 << std::endl;
//		//			printf("r2 t1:\n");
//		//			std::cout << R2 * p + t1 << std::endl;
//		//			printf("r2 t2:\n");
//		//			std::cout << R2 * p + t2 << std::endl;
//
//
//		// step 6: triangulate
//		//			Mat p3d;
//		//			reprojectImageTo3D(disparity_map, p3d)
//		//			std::cout << "Point 4d: " << p4d << std::endl;
//	} catch (...) {
//	}
//
//	printf("Total time: %f seconds\n",
//			(float) (clock() - c_feature) / CLOCKS_PER_SEC);
//
//	waitKey(0);
//
//	return 0;
//}

/** @function main for video */
int main(int argc, char** argv) {
	//	if (argc != 3) {
	//		readme();
	//		return -1;
	//	}

	VideoCapture cap(0); // open default cam
	if (!cap.isOpened())
		return -1;

	namedWindow("Features", 1);

	Mat img_prev;
	std::vector<KeyPoint> keypoints_prev;
	Mat descriptors_prev;

	// Camera intrinsic matrix
	Mat cam_intrinsic, cam_distortion;

	FileStorage fs("cam_info/201606071648.yml", FileStorage::READ);
	fs["camera_matrix"] >> cam_intrinsic;
	fs["distortion_coefficients"] >> cam_distortion;
	fs.release();
	cam_intrinsic.convertTo(cam_intrinsic, CV_64F);
	cam_distortion.convertTo(cam_distortion, CV_64F);
	std::cout << "cam_intrinsic: " << cam_intrinsic << std::endl;
	std::cout << "cam_distortion: " << cam_distortion << std::endl;

	for (;;) {
		Mat frame;
		Mat img_curr_origin, img_curr;
		cap >> frame; // new frame from cam
		cvtColor(frame, img_curr_origin, CV_BGR2GRAY);

		clock_t c_feature, c_extractor, c_match, c_homo;

		// step 0: undistorted img
		undistort(img_curr_origin, img_curr, cam_intrinsic, cam_distortion);

		// step 1
		c_feature = clock();
		int minHessian = 400;
		FastFeatureDetector detector(15);
		std::vector<KeyPoint> keypoints_curr;

		detector.detect(img_curr, keypoints_curr);

		printf("Feature Detection time: %f seconds\n",
				(float) (clock() - c_feature) / CLOCKS_PER_SEC);

		Mat img_keypoints_curr;
		drawKeypoints(img_curr, keypoints_curr, img_keypoints_curr,
				Scalar::all(-1), DrawMatchesFlags::DEFAULT);
		imshow("Features", img_keypoints_curr);

		// step 2: descriptor
		c_extractor = clock();
		SurfDescriptorExtractor extractor;
		Mat descriptors_curr;
		extractor.compute(img_curr, keypoints_curr, descriptors_curr);

		printf("Descriptor Extraction time: %f seconds\n",
				(float) (clock() - c_extractor) / CLOCKS_PER_SEC);

		// step 3: FLANN matcher
		c_match = clock();
		if (!img_prev.empty()) {
			FlannBasedMatcher matcher;
			std::vector<DMatch> matches;
			matcher.match(descriptors_curr, descriptors_prev, matches);

			printf("Match time: %f seconds\n",
					(float) (clock() - c_match) / CLOCKS_PER_SEC);

			c_homo = clock();
			// key points distance
			double min_dis = 100;
			for (int i = 0; i < descriptors_curr.rows; i++) {
				if (matches[i].distance < min_dis)
					min_dis = matches[i].distance;
			}

			std::vector<DMatch> good_matches;

			for (int i = 0; i < descriptors_curr.rows; i++) {
				if (matches[i].distance < 3 * min_dis)
					good_matches.push_back(matches[i]);
			}

			std::vector<Point2f> good_curr;
			std::vector<Point2f> good_prev;

			for (int i = 0; i < good_matches.size(); i++) {
				//-- Get the keypoints from the good matches
				good_curr.push_back(keypoints_curr[good_matches[i].queryIdx].pt);
				good_prev.push_back(keypoints_prev[good_matches[i].trainIdx].pt);
			}

			try {
				// step 4: find fundamental Mat and rectification
				Mat F = findFundamentalMat(good_prev, good_curr, CV_RANSAC);

				Mat H = findHomography(good_prev, good_curr, CV_RANSAC);

				printf("Fundamental Mat time: %f seconds\n",
						(float) (clock() - c_homo) / CLOCKS_PER_SEC);

				// step 5: compute R, t - SVD
				Mat E = cam_intrinsic.t() * F * cam_intrinsic;
				Mat w_value, u, vt, R90;
				SVD::compute(E, w_value, u, vt);
				R90 = Mat::zeros(3, 3, CV_64F);
				R90.at<double>(0, 1) = -1;
				R90.at<double>(1, 0) = 1;
				R90.at<double>(2, 2) = 1;

				std::cout << "det F: " << determinant(F) << std::endl;
				std::cout << "det E: " << determinant(E) << std::endl;
				std::cout << "det u: " << determinant(u) << std::endl;
				std::cout << "det vt: " << determinant(vt) << std::endl;
				Mat tmp_R[4] = {u * R90 * vt, u * R90.t() * vt, - u * R90 * vt, - u * R90.t() * vt};
				Mat R1, R2;
				std::vector<int> tmp_R_array;
				for(int ii = 0; ii < 4; ii++) {
					float tmp_det = determinant(tmp_R[ii]);
					if(tmp_det == 1.0) {
						tmp_R_array.push_back(ii);
						printf("selected: %d\n", ii);
					}
				}
				R1 = tmp_R[tmp_R_array[0]]; R2 = tmp_R[tmp_R_array[1]];

				Mat t1 = u.col(2);
				Mat t2 = -u.col(2);

				std::cout << "R1: " << R1 << std::endl;
				std::cout << "R2: " << R2 << std::endl;
				std::cout << "t1: " << t1 << std::endl;
				std::cout << "t2: " << t2 << std::endl;

				Mat p(3, 1, CV_64F, Scalar(1));
				printf("r1 t1:\n");
				std::cout << R1 * p + t1 << std::endl;
				printf("r1 t2:\n");
				std::cout << R1 * p + t2 << std::endl;
				printf("r2 t1:\n");
				std::cout << R2 * p + t1 << std::endl;
				printf("r2 t2:\n");
				std::cout << R2 * p + t2 << std::endl;

			} catch (...) {
			}

		}

		img_curr.copyTo(img_prev);
		keypoints_prev = keypoints_curr;
		descriptors_curr.copyTo(descriptors_prev);

		printf("Total time: %f seconds\n",
				(float) (clock() - c_feature) / CLOCKS_PER_SEC);

		if (waitKey(30) >= 0)
			break;
	}

	return 0;
}

/** @function readme */
void readme() {
	std::cout << " Usage: ./SURF_detector <img1> <img2>" << std::endl;
}
