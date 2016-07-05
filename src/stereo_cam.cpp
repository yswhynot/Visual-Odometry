/*
 * stereo_cam.cpp
 *
 *  Created on: Jul 4, 2016
 *      Author: yisha
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <iomanip>

#include "opencv2/sfm.hpp"
#include "opencv2/sfm/robust.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/viz.hpp"

#include "cvsba/cvsba.h"

using namespace cv;
using namespace cv::sfm;
using namespace std;

const int IMG_WIDTH = 800;
const int IMG_HEIGHT = 600;
const int BUNDLE_WINDOW = 5; // should be larger than 3
const float F_RATIO = 0.8;

bool getGoodMatches(vector<KeyPoint>& keypoints_r,
		vector<KeyPoint>& keypoints_l, Mat& descriptors_r, Mat& descriptors_l,
		vector<Point2f>& good_r, vector<Point2f>& good_l) {

	FlannBasedMatcher matcher;
	vector<DMatch> good_matches;

	matcher.add(vector<Mat> (1, descriptors_l));
	vector<vector<DMatch> > matches;
	matcher.knnMatch(descriptors_r, matches, 2);

	// putative matches
	for (size_t i = 0; i < matches.size(); i++) {
		float dis0 = matches[i][0].distance;
		float dis1 = matches[i][1].distance;
		if (dis0 < F_RATIO * dis1) {
			good_matches.push_back(matches[i][0]);
		}
	}

	for (size_t i = 0; i < good_matches.size(); i++) {
		// Get the keypoints from the good matches
		Point2f kp_curr = keypoints_r[good_matches[i].queryIdx].pt;
		Point2f kp_prev = keypoints_l[good_matches[i].trainIdx].pt;

		good_r.push_back(kp_curr);
		good_l.push_back(kp_prev);

	}

	cout << "Good matches num: " << good_matches.size() << endl;

	if (good_matches.size() < 8)
		return false;

	return true;
}

int main(int argc, char** argv) {
	namedWindow("Features_curr", 1);
	namedWindow("Features_prev", 1);

	// load camera intrinsic matrix
	Mat Kl, Kr, Dl, Dr;
	FileStorage fs("dataset/stereo_calib/intrinsics.yml", FileStorage::READ);
	fs["M1"] >> Kl;
	fs["D1"] >> Dl;
	fs["M2"] >> Kr;
	fs["D2"] >> Dr;
	fs.release();
	Kl.convertTo(Kl, CV_64F);
	Dl.convertTo(Dl, CV_64F);
	Kr.convertTo(Kr, CV_64F);
	Dr.convertTo(Dr, CV_64F);
	Mat R_cam = Mat::eye(3, 3, CV_64F);
	Mat t_cam = Mat::zeros(3, 1, CV_64F);
	Mat P1, P2;
	projectionFromKRt(Kl, R_cam, t_cam, P1);
	t_cam.at<double> (0, 0) = -0.0615;
	projectionFromKRt(Kr, R_cam, t_cam, P2);
	cout << "P1: " << P1 << endl << "P2: " << P2 << endl;

	Mat img_origin_r, img_r, img_origin_l, img_l;

	// init detection and extraction container
	Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(35);
	Ptr<xfeatures2d::DAISY> extractor = xfeatures2d::DAISY::create();

	vector<Point2f> good_prev;
	vector<KeyPoint> keypoints_prev;
	Mat descriptors_prev;
	Mat img_prev;

	// rotation & translation arrays
	vector<Mat> R_vec, t_vec;
	vector<Affine3d> path;
	vector<Affine3d> path_est;

	Mat R, t, R1, R2, Q;
	vector<Vec3f> point_cloud_est;
	vector<vector<Point3d> > point3d_vec;
	//	vector<vector<vector<Point3d> > > point3d_vec_pair;

	// init sba and set params
	// run sba optimization
	cvsba::Sba sba;
	// change params if desired
	cvsba::Sba::Params params;
	params.type = cvsba::Sba::MOTIONSTRUCTURE;
	params.iterations = 500;
	params.minError = 1e-20;
	params.fixedIntrinsics = 5;
	params.fixedDistortion = 5;
	params.verbose = false;
	sba.setParams(params);

	clock_t c_begin = clock();

	for (int img_i = 0; img_i < 4; img_i++) {
		stringstream ss_r, ss_l;
		ss_r << "dataset/stereo_set1/" << setw(2) << setfill('0') << img_i
				<< "r.jpg";
		ss_l << "dataset/stereo_set1/" << setw(2) << setfill('0') << img_i
				<< "l.jpg";
		cout << "current idx: " << img_i << endl;
		img_origin_r = imread(ss_r.str(), CV_BGR2GRAY);
		img_origin_l = imread(ss_l.str(), CV_BGR2GRAY);

		clock_t c_feature, c_extractor, c_match, c_homo;

		// init params
		//		MatchImgPair match_img_pair;
		vector<KeyPoint> keypoints_r, keypoints_l;
		Mat descriptors_r, descriptors_l;
		//		vector<DMatch> matches;
		vector<DMatch> good_matches;
		vector<Point2f> good_l, good_r;

		// step 0: undistorted img
		undistort(img_origin_l, img_l, Kl, Dl);
		undistort(img_origin_r, img_r, Kr, Dr);

		// step 1: detect features
		c_feature = clock();
		detector->detect(img_l, keypoints_l);
		detector->detect(img_r, keypoints_r);
		printf("Feature Detection time: %f seconds\n",
				(float) (clock() - c_feature) / CLOCKS_PER_SEC);

		// step 2: descriptor
		c_extractor = clock();
		extractor->compute(img_l, keypoints_l, descriptors_l);
		extractor->compute(img_r, keypoints_r, descriptors_r);
		printf("Descriptor Extraction time: %f seconds\n",
				(float) (clock() - c_extractor) / CLOCKS_PER_SEC);

		// step 3: FLANN matcher
		c_match = clock();
		if (!getGoodMatches(keypoints_r, keypoints_l, descriptors_r,
				descriptors_l, good_r, good_l))
			continue;

		printf("Match time: %f seconds\n",
				(float) (clock() - c_match) / CLOCKS_PER_SEC);

		//		match_img_pair.match_pair_num = good_matches.size();

		//		string output = "output/";
		//		output += ss.str();
		//		imwrite(output, img_keypoints_r);

		try {

			// step 4: get 3d positions
			Mat point4d_homo;
			triangulatePoints(P1, P2, good_l, good_r, point4d_homo);

			vector<Point3d> points3d;
			for (int i = 0; i < point4d_homo.cols; i++) {
				Vec3f tmp_p;
				//				cout << "homo: " << point4d_homo.col(i) << endl;
				homogeneousToEuclidean(point4d_homo.col(i), tmp_p);
				points3d.push_back(Point3d(tmp_p[0], tmp_p[1], tmp_p[2]));
				//				cout << "p: " << tmp_3d_point << endl;
				//				match_img_pair.points_3d.push_back(tmp_3d_point);

				//				Vec3f tmp_vec3 = Vec3f(point4d_homo.at<float> (0, i) / z_index,
				//						point4d_homo.at<float> (1, i) / z_index,
				//						point4d_homo.at<float> (2, i) / z_index);
				//				point_cloud_est.push_back(tmp_vec3);
			}
			point3d_vec.push_back(points3d);

			if (point3d_vec.size() < 2) {
				good_prev = good_r;
				keypoints_prev = keypoints_r;
				descriptors_r.copyTo(descriptors_prev);
				img_r.copyTo(img_prev);
				continue;
			}

			// step 5: match prev feature points
			vector<Point3d> point3d_pair_prev, point3d_pair_curr;
			vector<Point2f> good_trans_prev, good_trans_curr;
			vector<int> good_id_prev, good_id_curr;
			if (!getGoodMatches(keypoints_r, keypoints_prev, descriptors_r,
					descriptors_prev, good_trans_curr, good_trans_prev))
				continue;

			for (size_t i = 0; i < good_trans_curr.size(); i++) {
				// find corresponding point
				int pos_prev = find(good_prev.begin(), good_prev.end(),
						good_trans_prev[i]) - good_prev.begin();
				int pos_curr = find(good_r.begin(), good_r.end(),
						good_trans_curr[i]) - good_r.begin();
				if (pos_prev < good_prev.size() && pos_curr < good_r.size()) {
					good_id_prev.push_back(pos_prev);
					good_id_curr.push_back(pos_curr);
					point3d_pair_prev.push_back(
							point3d_vec[img_i - 1][pos_prev]);
					point3d_pair_curr.push_back(point3d_vec[img_i][pos_curr]);
				}
			}

			Mat Rt;
			Mat mask;
			cout << "size: " << point3d_pair_prev.size() << ", "
					<< point3d_pair_curr.size() << endl;
			estimateAffine3D(point3d_pair_prev, point3d_pair_curr, Rt, mask);
			Affine3d Rt_af(Rt);
			cout << "R: " << Rt_af.rotation() << endl;
			cout << "t: " << Rt_af.translation() << endl;
			//			match_array.push_back(match_img_pair);

			Mat show_img_curr, show_img_prev;
			img_r.copyTo(show_img_curr);
			img_prev.copyTo(show_img_prev);
			for (size_t i = 0; i < good_id_prev.size(); i++) {
				circle(show_img_prev, good_prev[good_id_prev[i]], 2,
						Scalar(0, 0, 255), -1);
				circle(show_img_curr, good_r[good_id_curr[i]], 2,
						Scalar(0, 255, 0), -1);
				line(show_img_curr, good_prev[good_id_prev[i]],
						good_r[good_id_curr[i]], Scalar(255, 0, 0), 1);
			}
			imshow("Features_curr", show_img_curr);
			imshow("Features_prev", show_img_prev);
			waitKey(0);

			// step 7: bundle adjustment with window BUNDLE_WINDOW


			good_prev = good_r;
			keypoints_prev = keypoints_r;
			descriptors_r.copyTo(descriptors_prev);
			img_r.copyTo(img_prev);

		} catch (...) {
		}

		printf("Total time: %f seconds\n",
				(float) (clock() - c_feature) / CLOCKS_PER_SEC);

		if (waitKey(30) >= 0)
			break;
	}

	viz::Viz3d window("Coordinate Frame");
	window.setWindowSize(Size(500, 500));
	window.setWindowPosition(Point(200, 200));
	window.setBackgroundColor(); // black by default

	Matx33d K = Matx33d(Kl.at<double> (0, 0), 0, Kl.at<double> (0, 2), 0,
			Kl.at<double> (0, 0), Kl.at<double> (1, 2), 0, 0, 1);

	if (point_cloud_est.size() > 0) {
		cout << "Rendering points   ... ";
		viz::WCloud cloud_widget(point_cloud_est, viz::Color::green());
		window.showWidget("point_cloud", cloud_widget);
		cout << "[DONE]" << endl;
	} else {
		cout << "Cannot render points: Empty pointcloud" << endl;
	}

	if (path.size() > 0) {
		cout << "Rendering Cameras  ... ";
		window.showWidget(
				"cameras_frames_and_lines",
				viz::WTrajectory(path, viz::WTrajectory::BOTH, 0.1,
						viz::Color::green()));

		window.showWidget("cameras_frustums",
				viz::WTrajectoryFrustums(path, K, 0.1, viz::Color::yellow()));
		window.setViewerPose(path[0]);
		cout << "[DONE]" << endl;
	} else {
		cout << "Cannot render the cameras: Empty path" << endl;
	}

	cout << "Total: " << (float) (clock() - c_begin) / CLOCKS_PER_SEC << endl;

	window.spin();

	return 0;
}
