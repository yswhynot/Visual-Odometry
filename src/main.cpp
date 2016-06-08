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

// reprojectImageTo3D

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
	Mat cam_intrinsic = Mat_<double>::zeros(3, 3);
	Mat cam_distortion = Mat_<double>::zeros(5, 1);

	FileStorage fs("com_info/201606071648.yml", FileStorage::READ);
	fs["camera_matrix"] >> cam_intrinsic;
	fs["distortion_coefficients"] >> cam_distortion;

	for (;;) {
		Mat frame;
		Mat img_curr;
		cap >> frame; // new frame from cam
		cvtColor(frame, img_curr, CV_BGR2GRAY);

		clock_t c_feature, c_extractor, c_match, c_homo;

		// step 1
		c_feature = clock();
		int minHessian = 400;
		FastFeatureDetector detector(15);
		std::vector<KeyPoint> keypoints_curr;

		detector.detect(img_curr, keypoints_curr);

		printf("Feature Detection time: %f seconds\n", (float)(clock() - c_feature) / CLOCKS_PER_SEC);

		Mat img_keypoints_curr;
		drawKeypoints(img_curr, keypoints_curr, img_keypoints_curr,
				Scalar::all(-1), DrawMatchesFlags::DEFAULT);
		imshow("Features", img_keypoints_curr);

		// step 2: descriptor
		c_extractor = clock();
		SurfDescriptorExtractor extractor;
		Mat descriptors_curr;
		extractor.compute(img_curr, keypoints_curr, descriptors_curr);

		printf("Descriptor Extraction time: %f seconds\n", (float)(clock() - c_extractor) / CLOCKS_PER_SEC);

		// step 3: FLANN matcher
		c_match = clock();
		if (!img_prev.empty()) {
			FlannBasedMatcher matcher;
			std::vector<DMatch> matches;
			matcher.match(descriptors_curr, descriptors_prev, matches);

			printf("Match time: %f seconds\n", (float)(clock() - c_match) / CLOCKS_PER_SEC);

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

			// step 4: find fundamental Mat
			Mat F = findFundamentalMat(good_curr, good_prev, CV_RANSAC);

			printf("Homography time: %f seconds\n", (float)(clock() - c_homo) / CLOCKS_PER_SEC);

			std::cout << "Foundation matrix:\n" << F << std::endl;

			// step 5: compute R, t - SVD
//			Mat E = cam_intrinsic.t() * F * cam_intrinsic;
//			Mat w, u, vt;
//			SVD::compute(E, w, u, vt);
//			Mat R1 = u * w * vt;
//			Mat R2 = u * w.t() * vt;
//			Mat t1 = u.col(2);
//			Mat t2 = - u.col(2);

			stereoRectify(cam_intrinsic, cam_distortion, cam_intrinsic, cam_distortion,
					img_curr.size(), R1);

		}

		img_curr.copyTo(img_prev);
		keypoints_prev = keypoints_curr;
		descriptors_curr.copyTo(descriptors_prev);

		printf("Total time: %f seconds\n", (float)(clock() - c_feature) / CLOCKS_PER_SEC);

		if (waitKey(30) >= 0)
			break;
	}

	return 0;
}

/** @function readme */
void readme() {
	std::cout << " Usage: ./SURF_detector <img1> <img2>" << std::endl;
}
