#include <stdio.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <math.h>
#include <stdlib.h>

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
const int CAM_NUM = 20;

double getScale(Affine3d& path_element) {
	Vec3d path_mat = path_element.translation();
	return norm(path_mat);
}

void constructProjectionMat2(Mat& P1, Mat& P2, Mat& cam_intrinsic,
		vector<Mat>& R_w, vector<Mat>& t_w, int index) {
	Mat R1, R2, t1, t2, R_tmp, t_tmp;
	R_tmp = Mat::eye(3, 3, CV_64F);
	t_tmp = Mat::zeros(3, 1, CV_64F);

	for (size_t i = 0; i < (index - 1); i++) {
		R_tmp = R_w[i] * R_tmp;
		t_tmp = R_w[i] * t_tmp + t_w[i];
//		cout << "R_tmp: " << R_tmp << endl << "t_tmp: " << t_tmp << endl;
	}
	R1 = R_tmp;
	t1 = t_tmp;
	R2 = R_w.back() * R_tmp;
	t2 = R_w.back() * t_tmp + t_w.back();
//	R1 = R_w[index - 1]; R2 = R_w[index];
//	t1 = t_w[index - 1]; t2 = t_w[index];
	cout << "R1: " << R1 << endl << "t1: " << t1 << endl;
	cout << "R2: " << R2 << endl << "t2: " << t2 << endl;

//	Mat wtf(3, 1, CV_64F);
//	wtf.at<double>(0, 0) = 0.1; wtf.at<double>(1, 0) = 0.2; wtf.at<double>(2, 0) = 0.3;
//	Rodrigues(wtf, R1);
//	cout << "R1: " << R1 << endl;
	projectionFromKRt(cam_intrinsic, R1, t1, P1);
	projectionFromKRt(cam_intrinsic, R2, t2, P2);
}

void generateRandom3dPoints(vector<Vec3f>& original_3dpoints) {
	cout << "Generating 3d points" << endl;

	srand(time(NULL));
	int x, y, z;
	for (int i = 0; i < 2000; i++) {
		x = rand() % 800 - 400;
		y = rand() % 800 - 400;
		z = rand() % 200 + 50;
		Vec3f tmp(x, y, z);
		original_3dpoints.push_back(tmp);
	}
	cout << "Initial 3d points done." << endl;
}

void generateRT(vector<Affine3d>& path, vector<Mat>& R_theo,
		vector<Mat>& t_theo) {
	cout << "Generating R t" << endl;

	for (int i = 0; i < CAM_NUM; i++) {

		Mat R = Mat::eye(3, 3, CV_64F);
		Mat t = Mat::zeros(3, 1, CV_64F);

		t.at<double> (0, 0) = 1;
		t.at<double> (1, 0) = 0.5;

//		if (i > 20) {
//			Vec3f tmp(0, 0.1, 0);
//			Rodrigues(tmp, R);
//			R.convertTo(R, CV_64F);
//			t.at<double> (0, 0) = 0.1;
//			t.at<double> (1, 0) = 0.05;
//		}

		t.at<double> (0, 0) += 0.05 * i;
		t.at<double> (1, 0) += 0.02 * i;
		path.push_back(Affine3d(R, t));

		if (i > 0) {
			R = R * R_theo[i - 1];
			t = R * t_theo[i - 1] + t;
		}

		R_theo.push_back(R);
		t_theo.push_back(t);

	}

	cout << "Initial R t done" << endl;
}

void projectToCamImg(vector<Vec3f>& points3d, Mat& cam_intrinsic,
		Mat& cam_distort, Mat& R, Mat& t, vector<Point2d>& output_img,
		vector<int>& point_ids) {
	Mat homo_3d, homo_2d, eud_2d, P;
	Mat tmp;
	projectionFromKRt(cam_intrinsic, R, t, P);
	for (size_t i = 0; i < points3d.size(); i++) {
		euclideanToHomogeneous(points3d[i], homo_3d);
		P.convertTo(P, CV_64F);
		homo_3d.convertTo(homo_3d, CV_64F);
		homo_2d = P * homo_3d;
		homogeneousToEuclidean(homo_2d, tmp);
		Point2d tmp_p(tmp.at<double> (0, 0), tmp.at<double> (0, 1));
		if ((tmp_p.x > -300) && (tmp_p.x < 300) && (tmp_p.y > -200) && (tmp_p.y
				< 200)) {
			output_img.push_back(tmp_p);
			point_ids.push_back(i);
		}
	}

}

void getMatchPairs(vector<Point2d>& points2d_prev,
		vector<Point2d>& points2d_curr, vector<int>& point_id_prev,
		vector<int>& point_id_curr) {
	vector<Point2d> tmp_prev, tmp_curr;
	vector<int> tmp_id_prev, tmp_id_curr;
	for (size_t i = 0; i < point_id_prev.size(); i++) {
		int index = find(point_id_curr.begin(), point_id_curr.end(),
				point_id_prev[i]) - point_id_curr.begin();
		if (index < point_id_curr.size()) {
			tmp_prev.push_back(points2d_prev[i]);
			tmp_curr.push_back(points2d_curr[index]);
			tmp_id_prev.push_back(point_id_prev[i]);
			tmp_id_curr.push_back(point_id_curr[index]);
		}
	}

	points2d_prev = tmp_prev;
	points2d_curr = tmp_curr;
	point_id_prev = tmp_id_prev;
	point_id_curr = tmp_id_curr;
}

void drawImages(vector<Point2d>& points2d_prev, vector<Point2d>& points2d_curr,
		vector<int>& points_id_prev, vector<int>& points_id_curr,
		vector<Vec3f>& points3d) {
	Mat image_prev = Mat::zeros(400, 600, CV_8UC3);
	Mat image_curr = Mat::zeros(400, 600, CV_8UC3);
	RNG rng(12345);

	for (size_t i = 0; i < points2d_prev.size(); i++) {
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
				rng.uniform(0, 255));
		int z_prev = points3d[points_id_prev[i]].val[2];
		int z_curr = points3d[points_id_curr[i]].val[2];
		Point2d p_prev(points2d_prev[i].x + 300, points2d_prev[i].y + 200);
		Point2d p_curr(points2d_curr[i].x + 300, points2d_curr[i].y + 200);
		circle(image_prev, p_prev, 200 / z_prev, color, -1);
		circle(image_curr, p_curr, 200 / z_curr, color, -1);
	}
	imshow("Points_prev", image_prev);
	imshow("Points_curr", image_curr);

	waitKey(0);
}

//int computeVisibilityImgArray(vector<MatchImgPair>& match_array,
//		vector<vector<int> >& visib_array,
//		vector<vector<Point2d> >& image_points) {
//	// compute for the 3d points of the first pair of imgs
//	// features all visible for the first pair
//	vector<int> visible;
//	vector<Point2d> image;
//	visible.assign(match_array[0].match_pair_num, 1);
//	visib_array.push_back(visible);
//	cout << "count " << 0 << ": " << visible.size() << endl;
//	image_points.push_back(match_array[0].features_curr);
//	visible.clear();
//
//	for (int i = 1; i < BUNDLE_WINDOW - 1; i++) {
//
//		vector<Point2d> prev_match = match_array[i - 1].features_curr;
//		vector<Point2d> curr_match = match_array[i].features_prev;
//
//		int count_n = 0;
//
//		for (size_t j = 0; j < match_array[0].match_pair_num; j++) {
//
//			// if feature point exist in prev(i), calculate curr(i+1)
//			int index = -1;
//			if (visib_array[i - 1][j] == 1) {
//				// find corresponding feature point of prev in curr
//				for (size_t k = 0; k < match_array[i].match_pair_num; k++) {
//					if (prev_match[j] == curr_match[k]) {
//						index = k;
//						break;
//					} // end if
//				} // end for k
//			} // end if
//
//			// if prev feature point exists
//			if (index != -1) {
//				visible.push_back(1);
//				image.push_back(match_array[i].features_curr[index]);
//				count_n++;
//			} else {
//				visible.push_back(0);
//				image.push_back(Point2d(0, 0));
//			}
//
//		} // end for j
//
//		if (count_n < 8)
//			return -1;
//		cout << "count " << i << ": " << count_n << endl;
//
//		visib_array.push_back(visible);
//		image_points.push_back(image);
//		visible.clear();
//		image.clear();
//	} // end for i
//
//	return 0;
//}

int main(int argc, char** argv) {

 	namedWindow("Points_prev");
	namedWindow("Points_curr");

	// load camera intrinsic matrix
	Mat cam_intrinsic, cam_distortion;
	FileStorage fs("cam_info/201606071648.yml", FileStorage::READ);
	fs["camera_matrix"] >> cam_intrinsic;
	fs["distortion_coefficients"] >> cam_distortion;
	fs.release();
	cam_intrinsic.convertTo(cam_intrinsic, CV_64F);
	cam_distortion.convertTo(cam_distortion, CV_64F);

	vector<Vec3f> original_3dpoints, est_3dpoints;
	generateRandom3dPoints(original_3dpoints);

	vector<Mat> R_theo, t_theo, R_est, t_est;
	vector<Affine3d> path, path_est;
	generateRT(path, R_theo, t_theo);

	bool is_first = true;
	vector<Point2d> points2d_prev;
	vector<int> point_id_prev;

	try {

		for (int img_i = 0; img_i < CAM_NUM; img_i += (BUNDLE_WINDOW - 1)) {
			for (int wi = 0; wi < (BUNDLE_WINDOW - 1); wi++) {
				int position = img_i + wi;
				Mat img_points, E, mask, R, t;
				vector<Point2d> points2d_curr;
				vector<int> point_id_curr;
				projectToCamImg(original_3dpoints, cam_intrinsic,
						cam_distortion, R_theo[position], t_theo[position],
						points2d_curr, point_id_curr);

				if (points2d_prev.size() == 0) {
					points2d_prev = points2d_curr;
					point_id_prev = point_id_curr;
					R = Mat::eye(3, 3, CV_64F);
					t = Mat::zeros(3, 1, CV_64F);
					R_est.push_back(R);
					t_est.push_back(t);
					path_est.push_back(Affine3d(R, t));
					continue;
				}

				getMatchPairs(points2d_prev, points2d_curr, point_id_prev,
						point_id_curr);

				//				Mat point_id_mat_prev = Mat::zeros(1, point_id_prev.size(),
				//						CV_32F);
				//				Mat point_id_mat_curr = Mat::zeros(1, point_id_curr.size(),
				//						CV_32F);
				//				for (size_t i = 0; i < point_id_prev.size(); i++) {
				//					point_id_mat_prev.at<float> (0, i) = point_id_prev[i];
				//					point_id_mat_curr.at<float> (0, i) = point_id_curr[i];
				//				}
				//				cout << "point_id_mat_prev: " << point_id_mat_prev << endl
				//						<< "point_id_mat_curr: " << point_id_mat_curr << endl;
				//
				//				Mat point_mat_prev =
				//						Mat::zeros(2, point_id_prev.size(), CV_64F);
				//				Mat point_mat_curr =
				//						Mat::zeros(2, point_id_curr.size(), CV_64F);
				//				for (size_t i = 0; i < points2d_prev.size(); i++) {
				//					point_mat_prev.at<double> (0, i) = points2d_prev[i].x;
				//					point_mat_prev.at<double> (1, i) = points2d_prev[i].y;
				//					point_mat_curr.at<double> (0, i) = points2d_curr[i].x;
				//					point_mat_curr.at<double> (1, i) = points2d_curr[i].y;
				//				}
				//				cout << "points2d_prev: " << point_mat_prev << endl
				//						<< "points2d_curr: " << point_mat_curr << endl;

				Point2d pp(cam_intrinsic.at<double> (0, 2),
						cam_intrinsic.at<double> (1, 2));
				E = findEssentialMat(points2d_prev, points2d_curr,
						cam_intrinsic.at<double> (0, 0), pp, RANSAC, 0.999,
						1.0, mask);

				int count_inlier = recoverPose(E, points2d_prev, points2d_curr,
						R, t, cam_intrinsic.at<double> (0, 0), pp, mask);

				t = getScale(path[position]) * t;

				//				cout << "Mask: " << mask << endl;
				cout << "Inliers: " << (float) count_inlier
						/ (float) points2d_prev.size() << endl;
				cout << "total points: " << points2d_prev.size() << endl;

				cout << "R: " << R << endl << "t: " << t << endl;
				cout << "R_theo: " << R_theo[position] << endl << "t_theo: " << t_theo[position]
								<< endl;

				R_est.push_back(R);
				t_est.push_back(t);
				path_est.push_back(Affine3d(R, t));

				Mat point4d_homo, P1, P2;
				constructProjectionMat2(P1, P2, cam_intrinsic, R_est, t_est, position);
				triangulatePoints(P1, P2, points2d_prev, points2d_curr,
						point4d_homo);

				drawImages(points2d_prev, points2d_curr, point_id_prev,
						point_id_curr, original_3dpoints);

				int count = 0;
				for (int i = 0; i < point4d_homo.cols; i++) {
					Vec3d tmp_3d_point;
					homogeneousToEuclidean(point4d_homo.col(i), tmp_3d_point);

					if (tmp_3d_point[2] > 0) {
						count++;
						est_3dpoints.push_back(tmp_3d_point);
						cout << "point est: " << tmp_3d_point << endl;
						cout << "point origin: " << original_3dpoints[point_id_curr[i]] << endl;
					}
				}
				cout << "Percent positive: " << (float) count
						/ (float) point4d_homo.cols << endl;
				cout << "iteration: " << position << endl;

				points2d_prev = points2d_curr;
				point_id_prev = point_id_curr;
			} // end for wi

			if (is_first) {

			} // end if is_first

			is_first = false;
		} // end for img_i
	} catch (...) {
	}

	Mat points_output = Mat::zeros(3, est_3dpoints.size(), CV_64F);
	for (size_t i = 0; i < est_3dpoints.size(); i++) {
		points_output.at<double> (0, i) = est_3dpoints[i][0];
		points_output.at<double> (1, i) = est_3dpoints[i][1];
		points_output.at<double> (2, i) = est_3dpoints[i][2];
	}
	cout << "point3d: " << points_output << endl;
//	Mat R_output = Mat::zeros(3, R_est.size(), CV_64F);
//	for (size_t i = 0; i < R_est.size(); i++) {
//		cout << "R_est: " << R_est[i] << endl << "t_est: " << t_est[i] << endl;
//		cout << "R:   : " << R_theo[i] << endl << "t    : " << t_theo[i]
//				<< endl;
//	}

	viz::Viz3d window("Coordinate Frame");
	window.setWindowSize(Size(500, 500));
	window.setWindowPosition(Point(200, 200));
	window.setBackgroundColor(); // black by default

	Matx33d K = Matx33d(cam_intrinsic.at<double> (0, 0), 0,
			cam_intrinsic.at<double> (0, 2), 0,
			cam_intrinsic.at<double> (0, 0), cam_intrinsic.at<double> (1, 2),
			0, 0, 1);

	if (original_3dpoints.size() > 0) {
		cout << "Rendering points   ... ";
		viz::WCloud
				cloud_widget_origin(original_3dpoints, viz::Color::orange());
		window.showWidget("point_cloud_origin", cloud_widget_origin);

		cout << "[DONE]" << endl;
	} else {
		cout << "Cannot render points: Empty pointcloud" << endl;
	}

	if (est_3dpoints.size() > 0) {
		cout << "Rendering points est   ... ";
		viz::WCloud cloud_widget_est(est_3dpoints, viz::Color::green());
		window.showWidget("point_cloud_est", cloud_widget_est);
		cout << "[DONE]" << endl;
	} else {
		cout << "Cannot render points: Empty pointcloud_est" << endl;
	}

	if (path.size() > 0) {
		cout << "Rendering Cameras  ... ";
		window.showWidget(
				"cameras_frames_and_lines",
				viz::WTrajectory(path, viz::WTrajectory::BOTH, 0.1,
						viz::Color::orange()));

		window.showWidget("cameras_frustums",
				viz::WTrajectoryFrustums(path, K, 0.1, viz::Color::yellow()));

		window.showWidget(
				"cameras_frames_and_lines_est",
				viz::WTrajectory(path_est, viz::WTrajectory::BOTH, 0.1,
						viz::Color::green()));

		window.showWidget("cameras_frustums_est",
				viz::WTrajectoryFrustums(path_est, K, 0.1, viz::Color::blue()));
		window.setViewerPose(path[0]);
		cout << "[DONE]" << endl;
	} else {
		cout << "Cannot render the cameras: Empty path" << endl;
	}

	window.spin();

}
