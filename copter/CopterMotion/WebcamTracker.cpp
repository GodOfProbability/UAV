////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <QtCore>
#include <QImage>
#include "WebcamTracker.h"

//Constructor
WebcamTracker::WebcamTracker(QObject *parent) :
QThread(parent)
{
	restart = false;
	abort = false;

	//Set threshold values
	//Blue umbrella
	//iLowH = 90;
	//iHighH = 140;
	//iLowS = 80;
	//iHighS = 180;
	//iLowV = 180;
	//iHighV = 255;
	//Pink marker
	iLowH = 100;
	iHighH = 180;
	iLowS = 100;
	iHighS = 180;
	iLowV = 120;
	iHighV = 255;
}

//Destructor
WebcamTracker::~WebcamTracker()
{
	mutex.lock();
	abort = true;
	condition.wakeOne();
	mutex.unlock();
	wait();
}

void WebcamTracker::StartTracking()
{
	QMutexLocker locker(&mutex);

	if (!isRunning()) {
		start(LowPriority);
	}
	else {
		restart = true;
		condition.wakeOne();
	}
}

void WebcamTracker::StopTracking()
{
	stop();
	abort = false;
}


void WebcamTracker::run()
{
	forever{

		qDebug() << "WebcamTracker Running";

		//Modes for convenience
		bool debugMode = true;
		bool ctrlEnabled = false;
		bool trackingEnabled = false;
		//pause and resume code
		bool pause = false;

		int iLastX = -1;
		int iLastY = -1;

		VideoCapture cap(0); //capture the video from webcam

		if (!cap.isOpened())  // if not success, exit program
		{
			qDebug() << "Cannot open the web cam" << endl;
		}

		currentFrameNum = 0;

		//Capture a temporary image from the camera
		cv::Mat imgTmp;
		cap.read(imgTmp);

		//Determine size of video input
		int irows = imgTmp.rows;
		int icols = imgTmp.cols;

		//Create a black image with the size as the camera output
		cv::Mat imgLines = cv::Mat::zeros(imgTmp.size(), CV_8UC3);

		forever{

			if (restart) break;
			if (abort) return;

			cv::Mat currentFrame;
			bool bSuccess = cap.read(currentFrame); // read a new frame from video

			if (!bSuccess) //if not success, break loop
			{
				qDebug() << "Cannot read a frame from video stream" << endl;
				break;
			}

			currentFrameNum++;


				cv::Mat imgHSV;

                cvtColor(currentFrame, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV: COLOR_BGR2HSV

				cv::Mat imgThresholded;

				inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

				//Width and height for morph
				int morph_width = 5;
				int morph_height = 5;

				//morphological opening (removes small objects from the foreground)
				erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));
				dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));

				//morphological closing (removes small holes from the foreground)
				dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));
				erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));

				//Calculate the moments of the thresholded image
				Moments oMoments = moments(imgThresholded);

				double dM01 = oMoments.m01;
				double dM10 = oMoments.m10;
				double dArea = oMoments.m00;

				// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
				if (dArea > 10000)
				{
					//calculate the position of the object
					int posX = dM10 / dArea;
					int posY = dM01 / dArea;

					//center of frame
					int centerX = icols / 2;
					int centerY = irows / 2;

					//distance from center
					int diffX = (centerX - posX);
					int diffY = (centerY - posY);

					putText(currentFrame, std::to_string(currentFrameNum), Point(icols - 120, irows - 20), 1, 1, Scalar(0, 0, 0), 1);

					if (diffX >= 50)
					{
						putText(currentFrame, "Target left", Point(posX, posY - 10), 1, 1, Scalar(255, 0, 0), 1);
						emit TargetLeft();
					}

					if (diffX <= -50)
					{
						putText(currentFrame, "Target right", Point(posX, posY - 10), 1, 1, Scalar(255, 0, 0), 1);
						emit TargetRight();
					}

					if (diffY >= 50)
					{
						putText(currentFrame, "Target front", Point(posX, posY + 10), 1, 1, Scalar(255, 0, 0), 1);
						emit TargetFront();
					}

					if (diffY <= -50)
					{
						putText(currentFrame, "Target back", Point(posX, posY + 10), 1, 1, Scalar(255, 0, 0), 1);
						emit TargetBack();
					}

					if (abs(diffX) < 50 && abs(diffY) < 50)
					{
						putText(currentFrame, "Target centered", Point(posX, posY - 10), 1, 1, Scalar(255, 0, 0), 1);
						emit TargetCenter();
					}

					//if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
					//{
						//Draw a red line from the previous point to the current point
						//line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0, 0, 255), 2);
						//Draw green crosshairs around the object
						circle(currentFrame, Point(posX, posY), 20, Scalar(0, 255, 0), 2);
						line(currentFrame, Point(posX, posY), Point(posX, posY - 25), Scalar(0, 255, 0), 2);
						line(currentFrame, Point(posX, posY), Point(posX, posY + 25), Scalar(0, 255, 0), 2);
						line(currentFrame, Point(posX, posY), Point(posX - 25, posY), Scalar(0, 255, 0), 2);
						line(currentFrame, Point(posX, posY), Point(posX + 25, posY), Scalar(0, 255, 0), 2);
					//}

					iLastX = posX;
					iLastY = posY;
				}

				//Nothing detected
				else
				{
					//Hold position
					emit TargetCenter();
				}

				emit newFrame(currentFrame, currentFrameNum);
                qDebug() << "Current Frame: " << currentFrameNum;
                msleep(500);

///*
				if (debugMode == true){
					imshow("Thresholded Image", imgThresholded); //show the thresholded image
				}
				else {
					//if not in debug mode, destroy the window
					cv::destroyWindow("Thresholded Image");
				}

				if (ctrlEnabled == true){
					namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

					//Create trackbars in "Control" window
					createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
					createTrackbar("HighH", "Control", &iHighH, 179);

					createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
					createTrackbar("HighS", "Control", &iHighS, 255);

					createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
					createTrackbar("HighV", "Control", &iHighV, 255);
				}
				else {
					//if not in ctrl mode, destroy the window
					cv::destroyWindow("Control");
				}

				if (trackingEnabled == true){
					currentFrame = currentFrame + imgLines; //Show tracking lines
				}
				else {
					//if tracking is not enabled, don't show tracking lines
					imgLines = cv::Mat::zeros(imgTmp.size(), CV_8UC3); //Clear existing tracking lines
				}

                imshow("Original", currentFrame); //show the original image

            switch (waitKey(50)){

			case 27: //'esc' key has been pressed, exit program.
				stop();

			case 116: //'t' has been pressed. Toggle tracking
				trackingEnabled = !trackingEnabled;
				if (trackingEnabled == false) cout << "Tracking disabled." << endl;
				else cout << "Tracking enabled." << endl;
				break;

			case 100: //'d' has been pressed. Toggle debug
				debugMode = !debugMode;
				if (debugMode == false) cout << "Debug disabled." << endl;
				else cout << "Debug enabled." << endl;
				break;

			case 99: //'c' has been pressed. Toggle control
				ctrlEnabled = !ctrlEnabled;
				if (ctrlEnabled == false) cout << "Control disabled." << endl;
				else cout << "Control enabled." << endl;
				break;

			case 112: //'p' has been pressed. this will pause/resume the code.
				pause = !pause;
				if (pause == true){
					cout << "Code paused, press 'p' again to resume" << endl;
					while (pause == true){
						//stay in this loop until 
						switch (waitKey()){
							//a switch statement inside a switch statement? Mind blown.
						case 112:
							//change pause back to false
							pause = false;
							cout << "Code Resumed" << endl;
							break;
						}
					}
				}

            }
//*/

		}

		mutex.lock();
		if (!restart)
			condition.wait(&mutex);
		restart = false;
		mutex.unlock();
	}

}

void WebcamTracker::stop()
{
    qDebug() << "WebcamTracker Stopping";
	//Make sure the copter holds position
	emit TargetCenter();
	//Clean up
	cv::destroyAllWindows();
	mutex.lock();
	abort = true;
	condition.wakeOne();
	mutex.unlock();
	wait();
}
