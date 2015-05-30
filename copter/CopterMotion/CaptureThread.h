#ifndef CAPTURETHREAD_H
#define CAPTURETHREAD_H
#include <QMutex>
#include <QThread>
#include <QWaitCondition>

#include "MAVLinkConnection.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

class CaptureThread : public QThread
{
	Q_OBJECT

public:
	explicit CaptureThread(MAVLinkConnection* connection);
	~CaptureThread();
	int mode;
	std::string path;

public slots:
	void StartCapture();
	void StopCapture();
	void onNewFrame(cv::Mat newFrame, int frameCount);

signals:
	void newFrame(cv::Mat, int);

protected:
	void run() Q_DECL_OVERRIDE;
	void stop();

private:
	MAVLinkConnection* _connection;
	QMutex mutex;
	QWaitCondition condition;
	bool restart;
	bool abort;
	enum captureMode
	{
		LOG,
		WEBCAM,
	}; 
	cv::Mat frame;
	int count;
	bool bNewFrame;
	bool logEnable;

	void write2Log(int frameNum);
};

#endif