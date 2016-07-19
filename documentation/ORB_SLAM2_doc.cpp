'''
 Pseudo code for ORB_SLAM2
 Author: Sha Yi
'''

<System>
	System() {

		mpVocabulary = new ORBVocabulary();

		mpKeyFrameDatabase = new KeyFrameDatabase(mpVocabulary);
		mpMap = new Map();

		mpTracker = new Tracking();

		mpLocalMapper = new LocalMapping();
		mapper_thread.run

		mpViewer = new Viewer();
		viewer_thread.run
	}

	TrackMonocular() {
		<Tracking>
			GrabImageMonocular() {
				if(!initialized) {
					mpIniORBextractor = new ORBextractor();
					<ORBExtractor>
						ORBextractor() {
							resize_everything
							factor = 1.0f / scaleFactor;
							nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - pow(factor, nlevels));
							do_ORB_settings
						}
					</ORBExtractor>
					
					mCurrentFrame = Frame(.., mpIniORBextractor, ..);
					<Frame>

					</Frame>
				}
				Track();
			}

			Track() {
				if(!initialized) {
					MonocularInitialization();
				}
			}

			MonocularInitialization() {

			}
		</Tracking>
	}
</System>