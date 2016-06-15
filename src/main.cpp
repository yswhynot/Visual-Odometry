#include <stdio.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;

const int IMG_WIDTH = 800;
const int IMG_HEIGHT = 600;

void readme();

void normalizeKeypoint(Point2f &p) {
	p.x = (p.x - IMG_WIDTH / 2) / IMG_WIDTH * 2;
	p.y = (p.y - IMG_HEIGHT / 2) / IMG_WIDTH * 2;
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
	img_prev_origin = imread("img/si0.jpg", CV_BGR2GRAY);

	// init the prev variables
	std::vector<KeyPoint> keypoints_prev;
	std::vector<Point2f> good_prev;
	Mat descriptors_prev;

	// init detection and extraction container
	FastFeatureDetector detector(15);
	SurfDescriptorExtractor extractor;
	FlannBasedMatcher matcher;

	for (int img_i = 1; img_i < 7; img_i++) {
		std::stringstream ss;
		ss << "img/si" << img_i << ".jpg";
		std::cout << "current idx: " << ss.str() << std::endl;
		img_curr_origin = imread(ss.str(), CV_BGR2GRAY);

		// init the img_prev parameters
		if(img_i == 1) {
			undistort(img_prev_origin, img_prev, cam_intrinsic, cam_distortion);
			detector.detect(img_prev, keypoints_prev);
			extractor.compute(img_prev, keypoints_prev, descriptors_prev);
		}

		clock_t c_feature, c_extractor, c_match, c_homo;

		// step 0: undistorted img
		undistort(img_curr_origin, img_curr, cam_intrinsic, cam_distortion);

		// step 1: detect features
		c_feature = clock();
		std::vector<KeyPoint> keypoints_curr;
		detector.detect(img_curr, keypoints_curr);
		printf("Feature Detection time: %f seconds\n",
				(float) (clock() - c_feature) / CLOCKS_PER_SEC);

		// step 2: descriptor
		c_extractor = clock();
		Mat descriptors_curr;
		extractor.compute(img_curr, keypoints_curr, descriptors_curr);

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
			normalizeKeypoint(kp_curr);
			normalizeKeypoint(kp_prev);
			good_curr.push_back(kp_curr);
			good_prev.push_back(kp_prev);
		}

		Mat img_keypoints_curr;
		drawKeypoints(img_curr, keypoints_curr, img_keypoints_curr,
						Scalar::all(-1), DrawMatchesFlags::DEFAULT);
		imshow("Features", img_keypoints_curr);

		try {
			// step 4: find fundamental Mat and rectification
			Mat F = findFundamentalMat(good_prev, good_curr, CV_RANSAC);

			printf("Fundamental Mat time: %f seconds\n",
					(float) (clock() - c_homo) / CLOCKS_PER_SEC);

			// step 5: normalize F
			Mat Fw_value, Fu, Fvt;
			Mat Fw = Mat::zeros(3, 3, CV_64F);
			SVD::compute(F, Fw_value, Fu, Fvt);
			Fw.at<double> (0, 0) = Fw_value.at<double> (0, 0);
			Fw.at<double> (1, 1) = Fw_value.at<double> (0, 1);
			Mat F_norm = Fu * Fw * Fvt;

			// step 6: svd
			Mat E = cam_intrinsic.t() * F_norm * cam_intrinsic;
			Mat w_value, u, vt, R90;
			SVD::compute(E, w_value, u, vt);
			R90 = Mat::zeros(3, 3, CV_64F);
			R90.at<double> (0, 1) = -1;
			R90.at<double> (1, 0) = 1;
			R90.at<double> (2, 2) = 1;

			Mat tmp_R[4] = { u * R90 * vt, u * R90.t() * vt, -u * R90 * vt, -u
					* R90.t() * vt };
			Mat R1, R2;
			std::vector<int> tmp_R_array;
			for (int ii = 0; ii < 4; ii++) {
				float tmp_det = determinant(tmp_R[ii]);
				if (tmp_det == 1.0) {
					tmp_R_array.push_back(ii);
				}
			}
			R1 = tmp_R[tmp_R_array[0]];
			R2 = tmp_R[tmp_R_array[1]];

			Mat t1 = u.col(2);
			Mat t2 = -u.col(2);

			// step 7: compute R-t, rescale t
			double scale = sqrt((good_curr.back().y - good_curr.front().y) * (good_curr.back().y - good_curr.front().y)
					+ (good_curr.back().x - good_curr.front().x) * (good_curr.back().x - good_curr.front().x))
					/ sqrt((good_prev.back().y - good_prev.front().y) * (good_prev.back().y - good_prev.front().y)
							+ (good_prev.back().x - good_prev.front().x) * (good_prev.back().x - good_prev.front().x));
//			t1 = t1 / scale;
//			t2 = t2 / scale;

			std::vector<Mat> rt;
			Mat rt_test = Mat::zeros(4, 4, CV_64F);
			for (int i = 0; i < 4; i++) {
				Mat tmp = Mat::zeros(4, 4, CV_64F);
				tmp.at<double> (3, 3) = 1;
				rt.push_back(tmp);
			}
			R1.copyTo(rt[0](Rect(0, 0, R1.cols, R1.rows)));
			t1.copyTo(rt[0](Rect(3, 0, t1.cols, t1.rows)));
			R1.copyTo(rt[1](Rect(0, 0, R1.cols, R1.rows)));
			t2.copyTo(rt[1](Rect(3, 0, t2.cols, t2.rows)));
			R2.copyTo(rt[2](Rect(0, 0, R2.cols, R2.rows)));
			t1.copyTo(rt[2](Rect(3, 0, t1.cols, t1.rows)));
			R2.copyTo(rt[3](Rect(0, 0, R2.cols, R2.rows)));
			t2.copyTo(rt[3](Rect(3, 0, t2.cols, t2.rows)));

			// step 8: reconstruct 3d
			std::vector<Point3f> features2d, features3d;

			for (unsigned int i = 0; i < keypoints_curr.size(); i++) {
				Point3f tmp_p;
				tmp_p.x = keypoints_curr[i].pt.x;
				tmp_p.y = keypoints_curr[i].pt.y;
				tmp_p.z = 1;
				features2d.push_back(tmp_p);
			}

			Mat cam_trans_3d = Mat::zeros(4, 4, CV_64F);
			Mat cam_inv = cam_intrinsic.inv();
			cam_inv.copyTo(cam_trans_3d(Rect(0, 0, cam_inv.cols, cam_inv.rows)));
			cam_trans_3d.at<double> (3, 3) = 1;

			perspectiveTransform(features2d, features3d, cam_trans_3d);

			// step 9: select r-t pair
			std::vector<double> positive_percent;
			int max_rt_count = 0;
			int selected_rt;
			for (int i = 0; i < 4; i++) {
				std::vector<Point3f> features3d_n;
				unsigned int count = 0;
				perspectiveTransform(features3d, features3d_n, rt[i]);
				for (unsigned int j = 0; j < features3d_n.size(); j++) {
					if (features3d_n[j].z > 0)
						count++;
				}
				positive_percent.push_back(
						(double) count / (double) features3d_n.size());
				if (count > max_rt_count) {
					max_rt_count = count;
					selected_rt = i;
				}
				std::cout << "rt " << i << ": " << rt[i] << std::endl;
			}
			printf("Current idx: %d\n", img_i);
			for (int i = 0; i < 4; i++) {
				printf("rt %d, percent positive: %f\n", i, positive_percent[i]);
			}
			printf("select rt: %d, percent positive: %f, count: %d\n",
					selected_rt, positive_percent[selected_rt], max_rt_count);

		} catch (...) {
		}

		printf("Total time: %f seconds\n",
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
