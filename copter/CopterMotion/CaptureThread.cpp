#include <QtCore>
#include "CaptureThread.h"

//Constructor
CaptureThread::CaptureThread(MAVLinkConnection* connection)
{
	_connection = connection;
	restart = false;
	abort = false;
    mode = captureMode::LOG;
	path = "Log/";
	bNewFrame = false;
	logEnable = true;
}

//Destructor
CaptureThread::~CaptureThread()
{
	mutex.lock();
	abort = true;
	condition.wakeOne();
	mutex.unlock();
	wait();
}

void CaptureThread::StartCapture()
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

void CaptureThread::StopCapture()
{
	stop();
	abort = false;
}


void CaptureThread::run()
{
	forever{
		qDebug() << "CaptureThread Running";
		
		if (mode == LOG) {
			//Initialize log file
			QFile myFile(QString::fromStdString(path) + "logfile.txt");
			if (!myFile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
			{
				qDebug() << "Cannot open file" << endl;
				return;
			}
			QTextStream myLog(&myFile);

			//Start capture loop
			forever{
				if (restart) { //myFile.close(); 
					break; }
				if (abort) { //myFile.close(); 
					return; }
				if (bNewFrame) {
					//Save frame
					imwrite(path + std::to_string(count) + ".jpg", frame);
					qDebug() << "Frame " << count << "written to disk.";
					
					if (logEnable) {
						//Write log
						MAVLinkConnection::VFRHUD const vfrHUD = _connection->vfrHUD();
						myLog << count
							<< ", VFR HUD airspeed = " << vfrHUD.airspeed
							<< ", groundspeed = " << vfrHUD.groundspeed
							<< ", heading = " << vfrHUD.heading
							<< ", throttlePercentage = " << vfrHUD.throttlePercentage
							<< ", altitude = " << vfrHUD.altitude
							<< ", climb = " << vfrHUD.climb << '\n';
					}

					mutex.lock();
					bNewFrame = false;
					mutex.unlock();
				}
			}
		}

		if (mode == WEBCAM) {

			VideoCapture cap(0); //capture the video from webcam

			if (!cap.isOpened())  // if not success, exit program
			{
				qDebug() << "Cannot open the web cam" << endl;
			}
			//Initialize counter
			int camFrameCount = 0;

			forever{

				if (restart) break;
				if (abort) return;

				cv::Mat camFrame;
				bool bSuccess = cap.read(camFrame); // read a new frame from video
				camFrameCount++;

				if (!bSuccess) //if not success, break loop
				{
					qDebug() << "Cannot read a frame from video stream" << endl;
					break;
				}
				qDebug() << "Frame " << camFrameCount << "written to disk.";
				imwrite(path + std::to_string(camFrameCount) + ".jpg", camFrame);
				sleep(1);
			}
		}

		//Restart
		mutex.lock();
		if (!restart)
			condition.wait(&mutex);
		restart = false;
		mutex.unlock();
	}
}

void CaptureThread::stop()
{
	mutex.lock();
	abort = true;
	condition.wakeOne();
	mutex.unlock();
	wait();
}

void CaptureThread::onNewFrame(cv::Mat newFrame, int frameCount)
{
	//qDebug() << "Frame received: " << frameCount;
	count = frameCount;
	frame = newFrame;
	mutex.lock();
	bNewFrame = true;
	mutex.unlock();
}

void CaptureThread::write2Log(int frameNum) {
	QFile myFile(QString::fromStdString(path) + "logfile.txt");
	if (!myFile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
	{
		qDebug() << "Cannot open file" << endl;
		return;
	}
	QTextStream myLog(&myFile);
	MAVLinkConnection::VFRHUD const vfrHUD = _connection->vfrHUD();
	myLog	<< frameNum
			<< ", VFR HUD airspeed = " << vfrHUD.airspeed
			<< ", groundspeed = " << vfrHUD.groundspeed
			<< ", heading = " << vfrHUD.heading
			<< ", throttlePercentage = " << vfrHUD.throttlePercentage
			<< ", altitude = " << vfrHUD.altitude
			<< ", climb = " << vfrHUD.climb << '\n';
	myFile.close();
}

