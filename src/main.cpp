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
//int main(int argc, char** argv) {
//	//	if (argc != 3) {
//	//		readme();
//	//		return -1;
//	//	}
//
//	VideoCapture cap(0); // open default cam
//	if (!cap.isOpened())
//		return -1;
//
//	namedWindow("Features", 1);
//
//	Mat img_prev;
//	std::vector<KeyPoint> keypoints_prev;
//	Mat descriptors_prev;
//
//	// Camera intrinsic matrix
////	Mat cam_intrinsic = (Mat_<double>(3, 3) << 825.16620138086046, 0, 273.20022871560587,
////			0, 822.93582954637532, 213.79980993132662,
////			0, 0, 1);
////	std::vector<double> cam_distortion = {0.21446347541767000, -0.43149636725486124,
////			1.9699575897250124*0.001, -1.7175748488014262*0.01};
//
//	for (;;) {
//		Mat frame;
//		Mat img_curr;
//		cap >> frame; // new frame from cam
//		cvtColor(frame, img_curr, CV_BGR2GRAY);
//
//		clock_t c_feature, c_extractor, c_match, c_homo;
//
//		// step 1
//		c_feature = clock();
//		int minHessian = 400;
//		SurfFeatureDetector detector(minHessian);
//		std::vector<KeyPoint> keypoints_curr;
//
//		detector.detect(img_curr, keypoints_curr);
//
//		printf("Feature Detection time: %f seconds\n", (float)(clock() - c_feature) / CLOCKS_PER_SEC);
//
//		Mat img_keypoints_curr;
//		drawKeypoints(img_curr, keypoints_curr, img_keypoints_curr,
//				Scalar::all(-1), DrawMatchesFlags::DEFAULT);
//		imshow("Features", img_keypoints_curr);
//
//		// step 2: descriptor
//		c_extractor = clock();
//		SurfDescriptorExtractor extractor;
//		Mat descriptors_curr;
//		extractor.compute(img_curr, keypoints_curr, descriptors_curr);
//
//		printf("Descriptor Extraction time: %f seconds\n", (float)(clock() - c_extractor) / CLOCKS_PER_SEC);
//
//		// step 3: FLANN matcher
//		c_match = clock();
//		if (!img_prev.empty()) {
//			FlannBasedMatcher matcher;
//			std::vector<DMatch> matches;
//			matcher.match(descriptors_curr, descriptors_prev, matches);
//
//			printf("Match time: %f seconds\n", (float)(clock() - c_match) / CLOCKS_PER_SEC);
//
//			c_homo = clock();
//			// key points distance
//			double min_dis = 100;
//			for (int i = 0; i < descriptors_curr.rows; i++) {
//				if (matches[i].distance < min_dis)
//					min_dis = matches[i].distance;
//			}
//
//			std::vector<DMatch> good_matches;
//
//			for (int i = 0; i < descriptors_curr.rows; i++) {
//				if (matches[i].distance < 3 * min_dis)
//					good_matches.push_back(matches[i]);
//			}
//
//			std::vector<Point2f> good_curr;
//			std::vector<Point2f> good_prev;
//
//			for (int i = 0; i < good_matches.size(); i++) {
//				//-- Get the keypoints from the good matches
//				good_curr.push_back(keypoints_curr[good_matches[i].queryIdx].pt);
//				good_prev.push_back(keypoints_prev[good_matches[i].trainIdx].pt);
//			}
//
//			Mat H = findHomography(good_curr, good_prev, CV_RANSAC);
//
//			printf("Homography time: %f seconds\n", (float)(clock() - c_homo) / CLOCKS_PER_SEC);
//
//			std::cout << "Transformation matrix:\n" << H << std::endl;
//
//		}
//
//		img_curr.copyTo(img_prev);
//		keypoints_prev = keypoints_curr;
//		descriptors_curr.copyTo(descriptors_prev);
//
//		printf("Total time: %f seconds\n", (float)(clock() - c_feature) / CLOCKS_PER_SEC);
//
//		if (waitKey(30) >= 0)
//			break;
//	}
//
//	return 0;
//}

// main for two imgs

Mat img1, img2;
std::vector<std::string> detector_gp, extractor_gp;

enum extractor_gp {
	E_SIFT, E_SURF, E_ORB, E_BRISK, E_BRIEF, E_FREAK, E_NUM
};

void DEM(int _detector, int _extractor, int _matcher) {

	std::vector<KeyPoint> keypoints1;
	std::vector<KeyPoint> keypoints2;

	clock_t c_feature, c_extractor, c_match, c_homo;

	// step 1
	c_feature = clock();

	if(_detector == D_SURF) {

	} else if(_detector == D_SIFT) {

	} else {

	}

	int minHessian = 400;
	SurfFeatureDetector detector(minHessian);
	detector.detect(img1, keypoints1);
	printf("%f ",
			(float) (clock() - c_feature) / CLOCKS_PER_SEC);

	detector.detect(img2, keypoints2);

	printf("%f ",
			(float) (clock() - c_feature) / CLOCKS_PER_SEC);

	// step 2: descriptor
	c_extractor = clock();
	SurfDescriptorExtractor extractor;
	Mat descriptors1, descriptors2;

	extractor.compute(img1, keypoints1, descriptors1);
	printf("%f ",
			(float) (clock() - c_extractor) / CLOCKS_PER_SEC);

	extractor.compute(img2, keypoints2, descriptors2);
	printf("%f ",
			(float) (clock() - c_extractor) / CLOCKS_PER_SEC);

	// step 3: FLANN matcher
	c_match = clock();

	FlannBasedMatcher matcher;
	std::vector<DMatch> matches;
	matcher.match(descriptors1, descriptors2, matches);

	printf("%f ",
			(float) (clock() - c_match) / CLOCKS_PER_SEC);

	c_homo = clock();
	// key points distance
	double min_dis = 100;
	for (int i = 0; i < descriptors1.rows; i++) {
		if (matches[i].distance < min_dis)
			min_dis = matches[i].distance;
	}

	std::vector<DMatch> good_matches;

	for (int i = 0; i < descriptors1.rows; i++) {
		if (matches[i].distance < 3 * min_dis)
			good_matches.push_back(matches[i]);
	}

	std::vector<Point2f> good1;
	std::vector<Point2f> good2;

	for (int i = 0; i < good_matches.size(); i++) {
		//-- Get the keypoints from the good matches
		good1.push_back(keypoints1[good_matches[i].queryIdx].pt);
		good2.push_back(keypoints2[good_matches[i].trainIdx].pt);
	}

	Mat H = findHomography(good1, good2, CV_RANSAC);

	printf("%f ",
			(float) (clock() - c_homo) / CLOCKS_PER_SEC);

	std::cout << "Transformation matrix:\n" << H << std::endl;

	printf(" %f\n",
			(float) (clock() - c_feature) / CLOCKS_PER_SEC);

}

void init() {
	std::string init_detect[] = {"FAST", "STAR", "SIFT", "SURF", "ORB", "BRISK",
		"MSER", "GFTT", "HARRIS", "Dense", "SimpleBlob"
	};
	for(int i=0; i < sizeof(init_detect)/sizeof(init_detect))

}

int main(int argc, char** argv) {

	//	Mat img1, img2;
	img1 = imread(argv[1], CV_BGR2GRAY);
	img2 = imread(argv[2], CV_BGR2GRAY);

	freopen("data/speed_test.txt", "w", stdout);

	init();

	for (int d = 0; d < D_NUM; d++) {
		for (int e = 0; e < E_NUM; e++) {
			DEM(d, e, 0);
		}
	}

	fclose(stdout);

	return 0;
}

/** @function readme */
void readme() {
	std::cout << " Usage: ./SURF_detector <img1> <img2>" << std::endl;
}
