#ifndef IMAGETRACKER_H
#define IMAGETRACKER_H

#include <QMutex>
#include <QThread>
#include <QWaitCondition>

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

class ImageTracker : public QThread
{
	Q_OBJECT

public:
	ImageTracker(QObject *parent = 0);
	~ImageTracker();

	int iLowH;
	int iHighH;
	int iLowS;
	int iHighS;
	int iLowV;
	int iHighV;

	public slots:
	void StartTracking();
	void StopTracking();

signals:
	void TargetLeft();
	void TargetRight();
	void TargetFront();
	void TargetBack();
	void TargetCenter();

protected:
	void run() Q_DECL_OVERRIDE;
	void stop();

private:
	QMutex mutex;
	QWaitCondition condition;
	bool restart;
	bool abort;
};

#endif