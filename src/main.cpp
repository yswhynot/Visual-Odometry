#include <stdio.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <math.h>

#include <opencv2/sfm.hpp>
#include "opencv2/sfm/robust.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
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
const int JUMP_FRAME = 3;
const float F_RATIO = 0.8;

void readme();

struct MatchImgPair {
	vector<Point2d> features_prev;
	vector<Point2d> features_curr;
	vector<Point3d> points_3d;
	Mat R;
	Mat t;
	unsigned int match_pair_num;
};

void constructProjectionMat(Mat& P1, Mat& P2, Mat& cam_intrinsic,
		vector<Affine3d>& path, vector<Affine3d>& path_est, int img_i) {
	P1 = Mat::eye(4, 4, CV_64F);
	P2 = Mat::eye(4, 4, CV_64F);

	if (img_i >= BUNDLE_WINDOW) {
		for (int i = 0; i < img_i - BUNDLE_WINDOW + 2; i++) {
			vector<Mat> pathi;
			split(path[i].matrix, pathi);
			//			cout << "In Bundle 1, path " << i << ": " << pathi[0] << endl;
			P1 = pathi[0] * P1;
			P2 = pathi[0] * P2;
		}
		for (int i = img_i - BUNDLE_WINDOW + 2; i <= img_i; i++) {
			vector<Mat> pathi;
			split(path_est[i].matrix, pathi);
			//			cout << "In Bundle 2, path " << i << ": " << pathi[0] << endl;
			P1 = pathi[0] * P1;
			P2 = pathi[0] * P2;
		}
	} else {
		for (int i = 0; i < img_i; i++) {
			vector<Mat> pathi;
			split(path_est[i].matrix, pathi);
			//			cout << "Not in Bundle, path " << i << ": " << pathi[0] << endl;
			P1 = pathi[0] * P1;
			P2 = pathi[0] * P2;
		}
	}
	P1.pop_back(1);
	P1 = cam_intrinsic * P1;
	vector<Mat> pathi;
	split(path_est[img_i].matrix, pathi);
	P2 = pathi[0] * P2;
	P2.pop_back(1);
	P2 = cam_intrinsic * P2;
	cout << "P1: " << P1 << endl << "P2: " << P2 << endl;
}

void constructProjectionMat2(Mat& P1, Mat& P2, Mat& cam_intrinsic,
		vector<Mat>& R_w, vector<Mat>& t_w) {
	Mat R1, R2, t1, t2, R_tmp, t_tmp;
	R_tmp = Mat::eye(3, 3, CV_64F);
	t_tmp = Mat::zeros(3, 1, CV_64F);

	for(size_t i = 0; i < (R_w.size() - 1); i++) {
		R_tmp = R_w[i] * R_tmp;
		t_tmp = R_w[i] * t_tmp + t_w[i];
	}
	R1 = R_tmp; t1 = t_tmp;
	R2 = R_w.back() * R_tmp;
	t2 = R_w.back() * t_tmp + t_w.back();

	cout << "R1: " << R1 << "\t\t" << "t1: " << t1 << endl;
	cout << "R2: " << R2 << "\t\t" << "t2: " << t2 << endl;

	projectionFromKRt(cam_intrinsic, R1, t1, P1);
	projectionFromKRt(cam_intrinsic, R2, t2, P2);
}

int computeVisibilityImgArray(vector<MatchImgPair>& match_array,
		vector<vector<int> >& visib_array,
		vector<vector<Point2d> >& image_points) {
	// compute for the 3d points of the first pair of imgs
	// features all visible for the first pair
	vector<int> visible;
	vector<Point2d> image;
	visible.assign(match_array[0].match_pair_num, 1);
	visib_array.push_back(visible);
	cout << "count " << 0 << ": " << visible.size() << endl;
	image_points.push_back(match_array[0].features_curr);
	visible.clear();

	for (int i = 1; i < BUNDLE_WINDOW - 1; i++) {

		vector<Point2d> prev_match = match_array[i - 1].features_curr;
		vector<Point2d> curr_match = match_array[i].features_prev;

		int count_n = 0;

		for (size_t j = 0; j < match_array[0].match_pair_num; j++) {

			// if feature point exist in prev(i), calculate curr(i+1)
			int index = -1;
			if (visib_array[i - 1][j] == 1) {
				// find corresponding feature point of prev in curr
				for (size_t k = 0; k < match_array[i].match_pair_num; k++) {
					if (prev_match[j] == curr_match[k]) {
						index = k;
						break;
					} // end if
				} // end for k
			} // end if

			// if prev feature point exists
			if (index != -1) {
				visible.push_back(1);
				image.push_back(match_array[i].features_curr[index]);
				count_n++;
			} else {
				visible.push_back(0);
				image.push_back(Point2d(0, 0));
			}

		} // end for j

		if (count_n < 8)
			return -1;
		cout << "count " << i << ": " << count_n << endl;

		visib_array.push_back(visible);
		image_points.push_back(image);
		visible.clear();
		image.clear();
	} // end for i

	return 0;
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

	// init the prev variables
	vector<KeyPoint> keypoints_prev;
	Mat descriptors_prev;
	vector<Point2f> good_prev;

	// init detection and extraction container
	Ptr<FastFeatureDetector> detector = FastFeatureDetector::create();
	Ptr<xfeatures2d::DAISY> extractor = xfeatures2d::DAISY::create();
	FlannBasedMatcher matcher;

	// rotation & translation arrays
	vector<Mat> R_vec, t_vec;
	vector<Affine3d> path;

	Mat R, t, P1, P2, R1, R2, Q;
	vector<Vec3f> point_cloud_est;
	Mat frame;

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

	// load video file
	VideoCapture cap("pure_translation2.webm");
	if (!cap.isOpened()) {
		cout << "Cannot open file" << endl;
		return -1;
	}

	clock_t c_begin, c_feature, c_extractor, c_match, c_homo;

	c_begin = clock();

	for (int img_i = 0; img_i < 4; img_i += (BUNDLE_WINDOW - 1)) {
		vector<MatchImgPair> match_array;
		vector<Mat> R_w, t_w, P_w, cam_int_w, cam_dis_w;

		for (int bundle_i = 0; bundle_i < BUNDLE_WINDOW; bundle_i++) {

			for (int i = 0; i < JUMP_FRAME; i++) {
				cap >> frame;
			}
			//		cap >> frame;
			if (frame.empty())
				break;
			cout << "iteration: " << img_i + bundle_i << endl;
			cvtColor(frame, img_curr_origin, CV_BGR2GRAY);

			// init the img_prev parameters
			if (keypoints_prev.size() == 0) {
				img_curr_origin.copyTo(img_prev_origin);
				undistort(img_prev_origin, img_prev, cam_intrinsic,
						cam_distortion);
				detector->detect(img_prev, keypoints_prev);
				extractor->compute(img_prev, keypoints_prev, descriptors_prev);
				Mat img_keypoints_prev;
				drawKeypoints(img_prev, keypoints_prev, img_keypoints_prev,
						Scalar::all(-1), DrawMatchesFlags::DEFAULT);
				//			imwrite("output/img_f/s0.jpg", img_keypoints_prev);

				R = Mat::eye(3, 3, CV_64F);
				t = Mat::zeros(3, 1, CV_64F);
				R_vec.push_back(R);
				t_vec.push_back(t);
				path.push_back(Affine3d(R, t));

				continue;
			}

			// init params
			MatchImgPair match_img_pair;
			vector<KeyPoint> keypoints_curr;
			Mat descriptors_curr;
			//		vector<DMatch> matches;
			vector<DMatch> good_matches_curr;
			vector<Point2f> good_prev, good_curr, good_prev_filter, good_curr_filter;
			Mat img_keypoints_curr;

			// step 0: undistorted img
			undistort(img_curr_origin, img_curr, cam_intrinsic, cam_distortion);

			// step 1: detect features
			c_feature = clock();
			detector->detect(img_curr, keypoints_curr);
			printf("Feature Detection time: %f seconds\n",
					(float) (clock() - c_feature) / CLOCKS_PER_SEC);
			cout << "Num of features: " << keypoints_curr.size() << endl;

			// step 2: descriptor
			c_extractor = clock();
			extractor->compute(img_curr, keypoints_curr, descriptors_curr);
			printf("Descriptor Extraction time: %f seconds\n",
					(float) (clock() - c_extractor) / CLOCKS_PER_SEC);

			// step 3: FLANN matcher
			matcher.add(vector<Mat> (1, descriptors_prev));
			vector<vector<DMatch> > matches;
			matcher.knnMatch(descriptors_curr, matches, 2);

			// putative matches
			int max_track_num = 0;
			for (size_t i = 0; i < matches.size(); i++) {
				float dis0 = matches[i][0].distance;
				float dis1 = matches[i][1].distance;
				if (dis0 < F_RATIO * dis1) {
					good_matches_curr.push_back(matches[i][0]);
				}
			}

			cout << "Good matches num: " << good_matches_curr.size() << endl;
			if (good_matches_curr.size() < 8)
				continue;

			for (size_t i = 0; i < good_matches_curr.size(); i++) {
				// Get the keypoints from the good matches
				Point2f kp_curr =
						keypoints_curr[good_matches_curr[i].queryIdx].pt;
				Point2f kp_prev =
						keypoints_prev[good_matches_curr[i].trainIdx].pt;

				good_curr.push_back(kp_curr);
				good_prev.push_back(kp_prev);

			}

			drawKeypoints(img_curr, keypoints_curr, img_keypoints_curr,
					Scalar::all(-1), DrawMatchesFlags::DEFAULT);
			imshow("Features", img_keypoints_curr);
			stringstream ss;
			ss << "output/img_f/s" << img_i << ".jpg";
			//		imwrite(ss.str(), img_keypoints_curr);

			try {
				// step 4: get essential mat
				Mat E, mask;
				Point2d pp(cam_intrinsic.at<double> (0, 2),
						cam_intrinsic.at<double> (1, 2));
				E = findEssentialMat(good_prev, good_curr,
						cam_intrinsic.at<double> (0, 0), pp, RANSAC, 0.999,
						1.0, mask);

				// step 5: get R and t
				int count_inlier = recoverPose(E, good_prev, good_curr, R, t,
						cam_intrinsic.at<double> (0, 0), pp, mask);
				cout << "Inliers: " << (float)count_inlier / (float)good_prev.size() << endl;

				R.copyTo(match_img_pair.R);
				t.copyTo(match_img_pair.t);

				cout << "R: " << R << endl;
				cout << "t: " << t << endl;
				R_w.push_back(R);
				t_w.push_back(t);

				// step 6: get 3d position
				Mat point4d_homo, P1, P2;
//				projectionFromKRt(cam_intrinsic, R, t, P1);
				//				constructProjectionMat(P1, P2, cam_intrinsic, path, path_est,
				//						img_i + bundle_i);
				constructProjectionMat2(P1, P2, cam_intrinsic, R_w, t_w);
				P_w.push_back(P2);
				triangulatePoints(P1, P2, good_prev, good_curr, point4d_homo);

				int count = 0;

				for (int i = 0; i < point4d_homo.cols; i++) {
					float z_index = point4d_homo.at<float> (3, i);
					Point3f tmp_3d_point;
					tmp_3d_point.x = point4d_homo.at<float> (0, i) / z_index;
					tmp_3d_point.y = point4d_homo.at<float> (1, i) / z_index;
					tmp_3d_point.z = point4d_homo.at<float> (2, i) / z_index;

					if (tmp_3d_point.z > 0) {
						count ++;
						match_img_pair.points_3d.push_back(tmp_3d_point);
						match_img_pair.features_prev.push_back(good_prev[i]);
						match_img_pair.features_curr.push_back(good_curr[i]);
					}
				}

				cout << "Percent positive: " << (float)count / (float)point4d_homo.cols << endl;
				match_img_pair.match_pair_num = count;
				match_array.push_back(match_img_pair);
				cam_int_w.push_back(cam_intrinsic);
				cam_dis_w.push_back(cam_distortion);

				// update all prev parameters
				keypoints_prev = keypoints_curr;
				descriptors_curr.copyTo(descriptors_prev);

				if (waitKey(30) >= 0)
					break;
			} catch (...) {
			}
		}

		// step 7: bundle adjustment with window BUNDLE_WINDOW

		vector<vector<int> > visib_array;
		vector<vector<Point2d> > image_points;

		// check if there is enough overlapping points
		if (computeVisibilityImgArray(match_array, visib_array, image_points)
				!= -1) {

			try {
				sba.run(match_array[0].points_3d, image_points, visib_array,
						cam_int_w, R_w, t_w, cam_dis_w);

				// print result
				cout << "Bundle result " << 0 << endl;
				cout << "R: " << R_w[0] << endl << "t: " << t_w[0] << endl;
				cout << "Bundle result " << 1 << endl;
				cout << "R: " << R_w[1] << endl << "t: " << t_w[1] << endl;
				cout << "Bundle result " << 2 << endl;
				cout << "R: " << R_w[2] << endl << "t: " << t_w[2] << endl;

				cout << "Optimization. Initial error="
						<< sba.getInitialReprjError() << " and Final error="
						<< sba.getFinalReprjError() << endl;

			} catch (...) {
				cout << "SBA failed" << endl;
			}

			// step 8: recover R-t and point cloud
			R_vec.insert(R_vec.end(), R_w.begin() + 1, R_w.end());
			t_vec.insert(t_vec.end(), t_w.begin() + 1, t_w.end());
			for (size_t i = 1; i < BUNDLE_WINDOW; i++)
				path.push_back(Affine3d(R_w[i], t_w[i]));

			// push back point cloud
			for (size_t i = 0; i < match_array[0].points_3d.size(); i++) {
				Point3d tmpp = match_array[0].points_3d[i];
				if (abs(tmpp.x) > 5000 || abs(tmpp.y) > 5000 || abs(tmpp.z)
						> 5000)
					continue;
				point_cloud_est.push_back(Vec3f(tmpp.x, tmpp.y, tmpp.z));
			}

		} else {
			cout << "Too few overlapping" << endl;
		}

		if (frame.empty())
			break;
	}

	viz::Viz3d window("Coordinate Frame");
	window.setWindowSize(Size(500, 500));
	window.setWindowPosition(Point(200, 200));
	window.setBackgroundColor(); // black by default

	Matx33d K = Matx33d(cam_intrinsic.at<double> (0, 0), 0,
			cam_intrinsic.at<double> (0, 2), 0,
			cam_intrinsic.at<double> (0, 0), cam_intrinsic.at<double> (1, 2),
			0, 0, 1);

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

/** @function readme */
void readme() {
	cout << " Usage: ./SURF_detector <img1> <img2>" << endl;
}
