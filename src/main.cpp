#include <stdio.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace cv;

const int IMG_WIDTH = 800;
const int IMG_HEIGHT = 600;

void readme();

void normalizeFeatures(std::vector<Point2f>& features, Mat& norm_t) {
	norm_t = Mat::eye(3, 3, CV_64F);
	norm_t.at<double> (0, 0) = 1 / (double) IMG_WIDTH * 2;
	norm_t.at<double> (1, 1) = 1 / (double) IMG_HEIGHT * 2;
	norm_t.at<double> (0, 2) = -0.5;
	norm_t.at<double> (1, 2) = -0.5;

	perspectiveTransform(features, features, norm_t);
}

void reconstruct3d(std::vector<Point2f>& match_keypoints,
		std::vector<Point3f>& features3d, Mat& cam_intrinsic) {
	std::vector<Point3f> features2d;
	for (unsigned int i = 0; i < match_keypoints.size(); i++) {
		Point3f tmp_p;
		tmp_p.x = match_keypoints[i].x;
		tmp_p.y = match_keypoints[i].y;
		tmp_p.z = 1;
		features2d.push_back(tmp_p);
	}

	Mat cam_trans_3d = Mat::zeros(4, 4, CV_64F);
	Mat cam_inv = cam_intrinsic.inv();
	cam_inv.copyTo(cam_trans_3d(Rect(0, 0, cam_inv.cols, cam_inv.rows)));
	cam_trans_3d.at<double> (3, 3) = 1;

	perspectiveTransform(features2d, features3d, cam_trans_3d);
}

double computePercentPositive(std::vector<Point3f>& features3d_prev,
		std::vector<Point3f>& features3d_curr, Mat& rt) {
	int size = features3d_prev.size();
	std::vector<Point3f> features3d_n_prev, features3d_n_curr;
	perspectiveTransform(features3d_prev, features3d_n_prev, rt);
	perspectiveTransform(features3d_curr, features3d_n_curr, rt);

	int count = 0;
	for (int i = 0; i < size; i++) {
		if(features3d_n_prev[i].z > 0 && features3d_n_curr[i].z > 0)
			count++;
	}

	return (double) count / (double) size;

}

int main(int argc, char** argv) {

	namedWindow("Features", 1);

	// load camera intrinsic matrix
	Mat cam_intrinsic, cam_distortion;
	FileStorage fs("cam_info/201606071648.yml", FileStorage::READ);
	fs["camera_matrix"] >> cam_intrinsic;
	fs["distortion_coefficients"] >> cam_distortion;
	fs.release();
	cam_intrinsic.convertTo(cam_intrinsic, CV_64F);
	cam_distortion.convertTo(cam_distortion, CV_64F);

	Mat img_curr_origin, img_curr, img_prev_origin, img_prev;
	img_prev_origin = imread("trans_img/s0.jpg", CV_BGR2GRAY);

	// init the prev variables
	std::vector<KeyPoint> keypoints_prev;
	std::vector<Point2f> good_prev;
	Mat descriptors_prev;

	// init detection and extraction container
	Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(50);
	Ptr<xfeatures2d::SURF> extractor = xfeatures2d::SURF::create();
	FlannBasedMatcher matcher;

	for (int img_i = 1; img_i < 10; img_i++) {
		std::stringstream ss;
		ss << "trans_img/s" << img_i << ".jpg";
		std::cout << "current idx: " << ss.str() << std::endl;
		img_curr_origin = imread(ss.str(), CV_BGR2GRAY);

		// init the img_prev parameters
		if (img_i == 1) {
			undistort(img_prev_origin, img_prev, cam_intrinsic, cam_distortion);
			detector->detect(img_prev, keypoints_prev);
			extractor->compute(img_prev, keypoints_prev, descriptors_prev);
			Mat img_keypoints_prev;
			drawKeypoints(img_prev, keypoints_prev, img_keypoints_prev,
					Scalar::all(-1), DrawMatchesFlags::DEFAULT);
			//			imwrite("output/img/s0.jpg", img_keypoints_prev);
		}

		clock_t c_feature, c_extractor, c_match, c_homo;

		// step 0: undistorted img
		undistort(img_curr_origin, img_curr, cam_intrinsic, cam_distortion);

		// step 1: detect features
		c_feature = clock();
		std::vector<KeyPoint> keypoints_curr;
		detector->detect(img_curr, keypoints_curr);
		printf("Feature Detection time: %f seconds\n",
				(float) (clock() - c_feature) / CLOCKS_PER_SEC);

		// step 2: descriptor
		c_extractor = clock();
		Mat descriptors_curr;
		extractor->compute(img_curr, keypoints_curr, descriptors_curr);

		printf("Descriptor Extraction time: %f seconds\n",
				(float) (clock() - c_extractor) / CLOCKS_PER_SEC);

		// step 3: FLANN matcher
		c_match = clock();
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

		std::vector<Point2f> good_curr, good_prev;

		for (int i = 0; i < good_matches.size(); i++) {
			//-- Get the keypoints from the good matches
			Point2f kp_curr = keypoints_curr[good_matches[i].queryIdx].pt;
			Point2f kp_prev = keypoints_prev[good_matches[i].trainIdx].pt;
			good_curr.push_back(kp_curr);
			good_prev.push_back(kp_prev);
		}

		Mat img_keypoints_curr;
		drawKeypoints(img_curr, keypoints_curr, img_keypoints_curr,
				Scalar::all(-1), DrawMatchesFlags::DEFAULT);
		imshow("Features", img_keypoints_curr);
		std::string output = "output/";
		output += ss.str();

		try {
			Mat E = findEssentialMat(good_prev, good_curr, cam_intrinsic.at<double>(0, 0),
					Point2f(cam_intrinsic.at<double>(0, 2), cam_intrinsic.at<double>(1, 2)));
			Mat R, t;
			recoverPose(E, good_prev, good_curr, R, t, cam_intrinsic.at<double>(0, 0),
					Point2f(cam_intrinsic.at<double>(0, 2), cam_intrinsic.at<double>(1, 2)));
			std::cout << "R: " << R << std::endl;
			std::cout << "t: " << t << std::endl;

		} catch (...) {
		}

		printf("Total time: %f seconds\n\n\n\n\n",
				(float) (clock() - c_feature) / CLOCKS_PER_SEC);

		// update all prev parameters
		keypoints_prev = keypoints_curr;
		descriptors_curr.copyTo(descriptors_prev);

		if (waitKey(30) >= 0)
			break;
	}

	return 0;
}

/** @function readme */
void readme() {
	std::cout << " Usage: ./SURF_detector <img1> <img2>" << std::endl;
}
