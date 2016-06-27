//#define CERES_FOUND true
//
//#include <opencv2/sfm.hpp>
//#include <opencv2/viz.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/calib3d.hpp>
//#include <opencv2/core.hpp>
//#include <opencv2/videoio.hpp>
//#include <cvsba/cvsba.h>
//#include <iostream>
//#include <fstream>
//#include <string>
//#include <time.h>
//using namespace std;
//using namespace cv;
//using namespace cv::sfm;
//
//const int JUMP_FRAME = 3;
//const int BUNDLE_WINDOW = 6;
//
//void updateRTfromPrev(vector<Mat>& input_R, vector<Mat>& input_t, Mat R_prev,
//		Mat t_prev) {
//	for (size_t i = 0; i < input_R.size(); i++) {
//		input_R[i] = R_prev * input_R[i];
//		input_t[i] = R_prev * input_t[i] + t_prev;
//	}
//}
//
//int constructRT(VideoCapture& cap, int img_i, Mat& img_prev, Mat& cam_intrinsic,
//		Mat& cam_distortion, bool& is_first, vector<Mat>& Rs_est_i,
//		vector<Mat>& ts_est_i, vector<Mat>& points3d_est_i,
//		vector<Mat>& img_frames, Matx33d K, bool is_projective) {
//	Mat frame, img_curr_distort, img_curr;
//	vector<string> images_paths;
//
//	if (img_prev.empty()) {
//		is_first = true;
//		cap >> frame;
//		cvtColor(frame, img_curr_distort, CV_BGR2GRAY);
//		undistort(img_curr_distort, img_prev, cam_intrinsic, cam_distortion);
//	}
//	stringstream ss;
//	ss << "output/sfm_tmp/s" << img_i << ".jpg";
//	imwrite(ss.str(), img_prev);
//	images_paths.push_back(ss.str());
//	img_frames.push_back(img_prev);
//
//	for (int i = 0; i < BUNDLE_WINDOW - 1; i++) {
//		for (int j = 0; j < JUMP_FRAME; j++) {
//			cap >> frame;
//		}
//		if (frame.empty())
//			break;
//
//		cvtColor(frame, img_curr_distort, CV_BGR2GRAY);
//		undistort(img_curr_distort, img_curr, cam_intrinsic, cam_distortion);
//		img_frames.push_back(img_curr);
//		stringstream ss;
//		ss << "output/sfm_tmp/s" << (img_i + i + 1) << ".jpg";
//		imwrite(ss.str(), img_curr);
//		img_frames.push_back(img_curr);
//		images_paths.push_back(ss.str());
//	}
//	cout << "iteration: " << img_i << endl;
//	cout << "size: " << images_paths.size() << endl;
//
//	if (frame.empty())
//		return -1;
//
//	reconstruct(images_paths, Rs_est_i, ts_est_i, K, points3d_est_i,
//			is_projective);
//
//	return 0;
//}
//
//int main(int argc, char* argv[]) {
//	cout << "---start---" << endl;
//
//	// init params
//	float f = 439.70369653574880, cx = 306.12914762277563, cy =
//			188.74951325121089;
//	// load camera intrinsic matrix
//	Mat cam_intrinsic, cam_distortion;
//	FileStorage fs("cam_info/201606071648.yml", FileStorage::READ);
//	fs["camera_matrix"] >> cam_intrinsic;
//	fs["distortion_coefficients"] >> cam_distortion;
//	fs.release();
//	cam_intrinsic.convertTo(cam_intrinsic, CV_64F);
//	cam_distortion.convertTo(cam_distortion, CV_64F);
//
//	// init sba and set params
//	// run sba optimization
//	cvsba::Sba sba;
//	// change params if desired
//	cvsba::Sba::Params params;
//	params.type = cvsba::Sba::MOTIONSTRUCTURE;
//	params.iterations = 500;
//	params.minError = 1e-20;
//	params.fixedIntrinsics = 5;
//	params.fixedDistortion = 5;
//	params.verbose = false;
//	sba.setParams(params);
//
//	// Build instrinsics
//	Matx33d K = Matx33d(f, 0, cx, 0, f, cy, 0, 0, 1);
//	bool is_projective = true;
//
//	// init container
//	vector<Mat> Rs_est, ts_est, points3d_est;
//	Mat frame, img_curr_distort, img_curr, img_prev, R_prev, t_prev;
//
//	// load video file
//	VideoCapture cap("pure_translation2.webm");
//	if (!cap.isOpened()) {
//		cout << "Cannot open file" << endl;
//		return -1;
//	}
//
//	clock_t c_begin = clock();
//
//	for (int img_i = 0; img_i < 35; img_i += (BUNDLE_WINDOW - 1)) {
//		vector<Mat> img_frames, Rs_est_i, ts_est_i, points3d_est_i;
//		bool is_first = false;
//
//		if (constructRT(cap, img_i, img_prev, cam_intrinsic, cam_distortion,
//				is_first, Rs_est_i, ts_est_i, points3d_est_i, img_frames, K,
//				is_projective) == -1)
//			break;
//
//		img_prev = img_frames.back();
//		R_prev = Rs_est_i.back();
//		t_prev = ts_est_i.back();
//
//		if (is_first) {
//			//			Rs_est.reserve(Rs_est_i.size());
//			//			ts_est.reserve(ts_est_i.size());
//			//			points3d_est.reserve(points3d_est_i.size());
//			Rs_est.push_back(Rs_est_i.front());
//			ts_est.push_back(ts_est_i.front());
//			points3d_est.push_back(points3d_est_i.front());
//		}
//
//		//		Rs_est.reserve(Rs_est.size() + Rs_est_i.size() - 1);
//		//		ts_est.reserve(ts_est.size() + ts_est_i.size() - 1);
//		//		points3d_est.reserve(points3d_est.size() + points3d_est_i.size() - 1);
//
//		Rs_est.insert(Rs_est.end(), Rs_est_i.begin() + 1, Rs_est_i.end());
//		ts_est.insert(ts_est.end(), ts_est_i.begin() + 1, ts_est_i.end());
//		points3d_est.insert(points3d_est.end(), points3d_est_i.begin() + 1,
//				points3d_est_i.end());
//
//		cout << "iteration " << img_i << " done." << endl;
//
//	}
//	// Print output
//	cout << "\n----------------------------\n" << endl;
//	cout << "Reconstruction: " << endl;
//	cout << "============================" << endl;
//	cout << "Estimated 3D points: " << points3d_est.size() << endl;
//	cout << "Estimated cameras: " << Rs_est.size() << endl;
//	cout << "Refined intrinsics: " << endl << K << endl << endl;
//	cout << "3D Visualization: " << endl;
//	cout << "============================" << endl;
//	viz::Viz3d window("Coordinate Frame");
//	window.setWindowSize(Size(500, 500));
//	window.setWindowPosition(Point(150, 150));
//	window.setBackgroundColor(); // black by default
//	// Create the pointcloud
//	cout << "Recovering points  ... ";
//	// recover estimated points3d
//	vector<Vec3f> point_cloud_est;
//	for (int i = 0; i < points3d_est.size(); ++i)
//		point_cloud_est.push_back(Vec3f(points3d_est[i]));
//	cout << "[DONE]" << endl;
//	cout << "Recovering cameras ... ";
//	vector<Affine3d> path;
//	for (size_t i = 0; i < Rs_est.size(); ++i)
//		path.push_back(Affine3d(Rs_est[i], ts_est[i]));
//	cout << "[DONE]" << endl;
//	if (point_cloud_est.size() > 0) {
//		cout << "Rendering points   ... ";
//		viz::WCloud cloud_widget(point_cloud_est, viz::Color::green());
//		window.showWidget("point_cloud", cloud_widget);
//		cout << "[DONE]" << endl;
//	} else {
//		cout << "Cannot render points: Empty pointcloud" << endl;
//	}
//	if (path.size() > 0) {
//		cout << "Rendering Cameras  ... ";
//		window.showWidget(
//				"cameras_frames_and_lines",
//				viz::WTrajectory(path, viz::WTrajectory::BOTH, 0.1,
//						viz::Color::green()));
//		window.showWidget("cameras_frustums",
//				viz::WTrajectoryFrustums(path, K, 0.1, viz::Color::yellow()));
//		window.setViewerPose(path[0]);
//		cout << "[DONE]" << endl;
//	} else {
//		cout << "Cannot render the cameras: Empty path" << endl;
//	}
//	for (int i = 0; i < Rs_est.size(); i++) {
//		cout << "Rotation " << i << ": " << Rs_est[i] << endl;
//		cout << "Trans " << i << ": " << ts_est[i] << endl;
//	}
//	cout << endl << "Press 'q' to close each windows ... " << endl;
//	window.spin();
//	return 0;
//}
