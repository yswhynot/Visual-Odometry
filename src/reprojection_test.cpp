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
const int CAM_NUM = 30;

void constructProjectionMat2(Mat& P1, Mat& P2, Mat& cam_intrinsic,
		vector<Mat>& R_w, vector<Mat>& t_w) {
	Mat R1, R2, t1, t2, R_tmp, t_tmp;
	R_tmp = Mat::eye(3, 3, CV_64F);
	t_tmp = Mat::zeros(3, 1, CV_64F);

	for (size_t i = 0; i < (R_w.size() - 1); i++) {
		R_tmp = R_w[i] * R_tmp;
		t_tmp = R_w[i] * t_tmp + t_w[i];
	}
	R1 = R_tmp;
	t1 = t_tmp;
	R2 = R_w.back() * R_tmp;
	t2 = R_w.back() * t_tmp + t_w.back();

	projectionFromKRt(cam_intrinsic, R1, t1, P1);
	projectionFromKRt(cam_intrinsic, R2, t2, P2);
}

void generateRandom3dPoints(vector<Vec3f>& original_3dpoints) {
	cout << "Generating 3d points" << endl;

	srand(time(NULL));
	int x, y, z;
	for (int i = 0; i < 1000; i++) {
		x = rand() % 400 - 200;
		y = rand() % 400 - 200;
		z = rand() % 100 + 50;
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

		if (i > 20) {
			Vec3f tmp(0, 0.1, 0);
			Rodrigues(tmp, R);
			R.convertTo(R, CV_64F);
			t.at<double> (0, 0) = 0.1;
			t.at<double> (1, 0) = 0.05;
		}

		if (i > 0) {
			t.at<double> (0, 0) += 0.05 * i;
			t.at<double> (1, 0) += 0.02 * i;
			R = R_theo[i - 1] * R;
			t = R_theo[i - 1] * t + t_theo[i - 1];
		}

		R_theo.push_back(R);
		t_theo.push_back(t);
		path.push_back(Affine3d(R, t));
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
		circle(image_curr, p_curr, 200 / z_prev, color, -1);
	}
	imshow("Points_prev", image_prev);
	imshow("Points_curr", image_curr);

	waitKey(0);
}

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
				Mat img_points;
				vector<Point2d> points2d_curr;
				vector<int> point_id_curr;
				projectToCamImg(original_3dpoints, cam_intrinsic,
						cam_distortion, R_theo[position], t_theo[position],
						points2d_curr, point_id_curr);

				if (points2d_prev.size() == 0) {
					points2d_prev = points2d_curr;
					point_id_prev = point_id_curr;
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

				Mat E, mask, R, t;
				Point2d pp(cam_intrinsic.at<double> (0, 2),
						cam_intrinsic.at<double> (1, 2));
				E = findEssentialMat(points2d_prev, points2d_curr,
						cam_intrinsic.at<double> (0, 0), pp, RANSAC, 0.999,
						1.0, mask);

				int count_inlier = recoverPose(E, points2d_prev, points2d_curr,
						R, t, cam_intrinsic.at<double> (0, 0), pp, mask);
				cout << "Mask: " << mask << endl;
				cout << "Inliers: " << (float) count_inlier
						/ (float) points2d_prev.size() << endl;
				cout << "total points: " << points2d_prev.size() << endl;

				cout << "R: " << R << endl << "t: " << t << endl;
				R_est.push_back(R);
				t_est.push_back(t);

				Mat point4d_homo, P1, P2;
				constructProjectionMat2(P1, P2, cam_intrinsic, R_est, t_est);
				triangulatePoints(P1, P2, points2d_prev, points2d_curr,
						point4d_homo);

				drawImages(points2d_prev, points2d_curr, point_id_prev,
						point_id_curr, original_3dpoints);

				int count = 0;
				for (int i = 0; i < point4d_homo.cols; i++) {
					float z_index = point4d_homo.at<float> (3, i);
					Point3f tmp_3d_point;
					tmp_3d_point.x = point4d_homo.at<float> (0, i) / z_index;
					tmp_3d_point.y = point4d_homo.at<float> (1, i) / z_index;
					tmp_3d_point.z = point4d_homo.at<float> (2, i) / z_index;

					if (tmp_3d_point.z > 0) {
						count++;
						est_3dpoints.push_back(tmp_3d_point);
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
		viz::WCloud cloud_widget_est(est_3dpoints, viz::Color::green());
		window.showWidget("point_cloud_est", cloud_widget_est);
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
