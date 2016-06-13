//
//Mat img1, img2;
//enum detector_gp {
//	D_FAST,
//	D_STAR,
//	D_SIFT,
//	D_SURF,
//	D_ORB,
//	D_BRISK,
//	D_MSER,
//	D_GFTT,
//	D_HARRIS,
//	D_Dense,
//	D_SimpleBlob,
//	D_NUM
//};
//
//enum extractor_gp {
//	E_SIFT, E_SURF, E_ORB, E_BRISK, E_BRIEF, E_FREAK, E_NUM
//};
//
//void DEM(int _detector, int _extractor, int _matcher, std::string& img1_name,
//		std::string& img2_name) {
//	try {
//		freopen("data/06-06-12-54pm.txt", "a", stdout);
//
//		std::vector<KeyPoint> keypoints1;
//		std::vector<KeyPoint> keypoints2;
//
//		clock_t c_feature, c_extractor, c_match, c_homo;
//
//		printf("%s %s ", img1_name.c_str(), img2_name.c_str());
//
//		// step 1
//		c_feature = clock();
//
//		Ptr<FeatureDetector> detector;
//		switch (_detector) {
//		case D_FAST:
//			printf("FAST  ");
//			detector = new FastFeatureDetector(15);
//			break;
//		case D_STAR:
//			printf("STAR  ");
//			detector = new StarFeatureDetector();
//			break;
//		case D_SIFT:
//			printf("SIFT  ");
//			detector = new SiftFeatureDetector();
//			break;
//		case D_SURF:
//			printf("SURF  ");
//			detector = new SurfFeatureDetector();
//			break;
//		case D_ORB:
//			printf("ORB   ");
//			detector = FeatureDetector::create("ORB");
//			break;
//		case D_BRISK:
//			printf("BRISK ");
//			detector = FeatureDetector::create("BRISK");
//			break;
//		case D_MSER:
//			printf("MSER  ");
//			detector = FeatureDetector::create("MSER");
//			break;
//		case D_GFTT:
//			printf("GFTT  ");
//			detector = new GoodFeaturesToTrackDetector();
//			break;
//		case D_HARRIS:
//			printf("HARRI ");
//			detector = new GoodFeaturesToTrackDetector(1000, 0.01, 1, 3, true);
//			break;
//		case D_Dense:
//			printf("Dense ");
//			detector = new DenseFeatureDetector();
//			break;
//		case D_SimpleBlob:
//			printf("SBlob ");
//			detector = new SimpleBlobDetector();
//			break;
//		default:
//			break;
//		}
//
//		detector->detect(img1, keypoints1);
//		printf("%f ", (float) (clock() - c_feature) / CLOCKS_PER_SEC);
//		detector->detect(img2, keypoints2);
//		printf("%f ", (float) (clock() - c_feature) / CLOCKS_PER_SEC);
//		printf("%d ", keypoints1.size());
//		printf("%d ", keypoints2.size());
//
//		// step 2: descriptor
//		c_extractor = clock();
//		Ptr<DescriptorExtractor> extractor;
//		Mat descriptors1, descriptors2;
//		switch (_extractor) {
//		case E_SIFT:
//			printf("SIFT  ");
//			extractor = new SiftDescriptorExtractor();
//			break;
//		case E_SURF:
//			printf("SURF  ");
//			extractor = new SurfDescriptorExtractor();
//			break;
//		case E_ORB:
//			printf("ORB   ");
//			extractor = DescriptorExtractor::create("ORB");
//			break;
//		case E_BRISK:
//			printf("BRISK ");
//			extractor = DescriptorExtractor::create("BRISK");
//			break;
//		case E_BRIEF:
//			printf("BRIEF ");
//			extractor = new BriefDescriptorExtractor();
//			break;
//		case E_FREAK:
//			printf("FREAK ");
//			extractor = DescriptorExtractor::create("FREAK");
//			break;
//		default:
//			break;
//		}
//
//		extractor->compute(img1, keypoints1, descriptors1);
//		printf("%f ", (float) (clock() - c_extractor) / CLOCKS_PER_SEC);
//		extractor->compute(img2, keypoints2, descriptors2);
//		printf("%f ", (float) (clock() - c_extractor) / CLOCKS_PER_SEC);
//
//		// step 3: FLANN matcher
//		c_match = clock();
//
//		if (descriptors1.type() != CV_32F) {
//			descriptors1.convertTo(descriptors1, CV_32F);
//			descriptors2.convertTo(descriptors2, CV_32F);
//		}
//
//		FlannBasedMatcher matcher;
//		std::vector<DMatch> matches;
//		matcher.match(descriptors1, descriptors2, matches);
//
//		printf("%f ", (float) (clock() - c_match) / CLOCKS_PER_SEC);
//		printf("%d ", matches.size());
//
//		c_homo = clock();
//		// key points distance
//		double min_dis = 100;
//		for (int i = 0; i < descriptors1.rows; i++) {
//			if (matches[i].distance < min_dis)
//				min_dis = matches[i].distance;
//		}
//
//		std::vector<DMatch> good_matches;
//
//		for (int i = 0; i < descriptors1.rows; i++) {
//			if (matches[i].distance < 3 * min_dis)
//				good_matches.push_back(matches[i]);
//		}
//
//		std::vector<Point2f> good1;
//		std::vector<Point2f> good2;
//
//		for (int i = 0; i < good_matches.size(); i++) {
//			//-- Get the keypoints from the good matches
//			good1.push_back(keypoints1[good_matches[i].queryIdx].pt);
//			good2.push_back(keypoints2[good_matches[i].trainIdx].pt);
//		}
//
//		if (good1.size() > 3 && good2.size() > 3) {
//			Mat H = findHomography(good1, good2, CV_RANSAC);
//			printf("%f ", (float) (clock() - c_homo) / CLOCKS_PER_SEC);
//		} else {
//			printf("-------- ");
//		}
//
//		//	std::cout << "Transformation matrix:\n" << H << std::endl;
//
//		printf(" %f\n", (float) (clock() - c_feature) * 1000000 / (keypoints1.size() * CLOCKS_PER_SEC));
//
//		fclose(stdout);
//
//	} catch (Exception& e) {
//		std::cout << e.what() << std::endl;
//	}
//
//}
//
//int test(int argc, char** argv) {
//
//	//	Mat img1, img2;
////	char img_list[21] = { 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j',
////			'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u' };
//	char img_list[8] = { 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
//
//	for (int ii = 0; ii < 8; ii++) {
//		std::string img1_name = "dataset/";
//		img1_name += img_list[ii];
//		img1_name += std::string("0.png");
//		std::string img2_name = "dataset/";
//		img2_name += img_list[ii];
//		img2_name += std::string("1.png");
//		img1 = imread(img1_name, CV_BGR2GRAY);
//		img2 = imread(img2_name, CV_BGR2GRAY);
//
//		for (int i = 0; i < 3; i++) {
//			for (int d = 0; d < D_NUM; d++) {
//				for (int e = 0; e < E_NUM; e++) {
//					DEM(d, e, 0, img1_name, img2_name);
//				}
//			}
//		}
//
////		DEM(D_FAST, E_SURF, 0, img1_name, img2_name);
//	}
//
//	return 0;
//}
