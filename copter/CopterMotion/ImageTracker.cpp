////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <QtCore>
#include "ImageTracker.h"

//Constructor
ImageTracker::ImageTracker(QObject *parent) :
QThread(parent)
{
	restart = false;
	abort = false;

	//Set threshold values
	//Blue umbrella
	iLowH = 90;
	iHighH = 140;
	iLowS = 80;
	iHighS = 180;
	iLowV = 180;
	iHighV = 255;
}

//Destructor
ImageTracker::~ImageTracker()
{
	mutex.lock();
	abort = true;
	condition.wakeOne();
	mutex.unlock();
	wait();
}

void ImageTracker::StartTracking()
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

void ImageTracker::StopTracking()
{
	stop();
	abort = false;
}


void ImageTracker::run()
{
	forever{

		qDebug() << "ImageTracker Running";

        //Modes for convenience
		bool debugMode = true;
		bool ctrlEnabled = false;
		bool trackingEnabled = false;
		//pause and resume code
		bool pause = false;

		//Initialize variables
		int num_low = 1;
		int num_high = 60;
		int img_num = num_low;

        std::string img_name = "Presentation1/Slide" + std::to_string(img_num) + ".JPG";
        int iLastX = -1;
		int iLastY = -1;

        /*// Create an IplImage object *image
        IplImage *image = cvLoadImage(img_name.c_str());

        // Display Image Attributes by accessing a IplImage object's data members
        qDebug() << "Width:" <<  image->width;
        qDebug() << "Height:" <<  image->height;
        qDebug() << "Pixel Depth:" <<  image->depth;
        qDebug() << "Channels:" <<  image->nChannels<<endl;
        */
        //Load a temporary image
		cv::Mat imgTmp;
        imgTmp = imread(img_name);

		//Create a black image with the size as the camera output
        cv::Mat imgLines = cv::Mat::zeros(imgTmp.size(), CV_8UC3);
        qDebug() << "Width:" <<  imgTmp.size().width;
        qDebug() << "Height:" <<  imgTmp.size().height<<endl;

		forever{

			if (restart) break;
			if (abort) return;

				cv::Mat newFrame;
				if (img_num > num_high) {
					img_num = num_low;
				}

                img_name = "Presentation1/Slide" + std::to_string(img_num) + ".JPG";

                newFrame = imread(img_name, CV_LOAD_IMAGE_COLOR);
				img_num++;

				//Determine size of video input
				int irows_newFrame = newFrame.rows;
				int icols_newFrame = newFrame.cols;

				cv::Mat imgHSV;

                if(newFrame.empty()) {
                    qDebug() << "Frame is empty!";
                }

                cvtColor(newFrame, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

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
					int centerX = icols_newFrame / 2;
					int centerY = irows_newFrame / 2;

					//distance from center
					int diffX = (centerX - posX);
					int diffY = (centerY - posY);

					putText(newFrame, img_name, Point(icols_newFrame - 120, irows_newFrame - 20), 1, 1, Scalar(0, 0, 0), 1);

					if (diffX >= 50)
					{
						putText(newFrame, "Target left", Point(posX, posY - 10), 1, 1, Scalar(255, 0, 0), 1);
						emit TargetLeft();
					}

					if (diffX <= -50)
					{
						putText(newFrame, "Target right", Point(posX, posY - 10), 1, 1, Scalar(255, 0, 0), 1);
						emit TargetRight();
					}

					if (diffY >= 50)
					{
						putText(newFrame, "Target front", Point(posX, posY + 10), 1, 1, Scalar(255, 0, 0), 1);
						emit TargetFront();
					}

					if (diffY <= -50)
					{
						putText(newFrame, "Target back", Point(posX, posY + 10), 1, 1, Scalar(255, 0, 0), 1);
						emit TargetBack();
					}

					if (abs(diffX) < 50 && abs(diffY) < 50)
					{
						putText(newFrame, "Target centered", Point(posX, posY - 10), 1, 1, Scalar(255, 0, 0), 1);
						emit TargetCenter();
					}

					if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
					{
						//Draw a red line from the previous point to the current point
						line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0, 0, 255), 2);
						//Draw green crosshairs around the object
						circle(newFrame, Point(posX, posY), 20, Scalar(0, 255, 0), 2);
						line(newFrame, Point(posX, posY), Point(posX, posY - 25), Scalar(0, 255, 0), 2);
						line(newFrame, Point(posX, posY), Point(posX, posY + 25), Scalar(0, 255, 0), 2);
						line(newFrame, Point(posX, posY), Point(posX - 25, posY), Scalar(0, 255, 0), 2);
						line(newFrame, Point(posX, posY), Point(posX + 25, posY), Scalar(0, 255, 0), 2);
					}

					iLastX = posX;
					iLastY = posY;
				}

                qDebug() << "Current Frame: " << img_num;
                msleep(500);
/*
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
					newFrame = newFrame + imgLines; //Show tracking lines
				}
				else {
					//if tracking is not enabled, don't show tracking lines
					imgLines = cv::Mat::zeros(imgTmp.size(), CV_8UC3); //Clear existing tracking lines
				}

				imshow("Original", newFrame); //show the original image

                switch (waitKey(300)){

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

*/
			}

		mutex.lock();
		if (!restart)
			condition.wait(&mutex);
		restart = false;
		mutex.unlock();
	}

}

void ImageTracker::stop()
{
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
