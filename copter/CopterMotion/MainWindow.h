#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "MAVLinkConnection.h"
#include "ImageTracker.h"
#include "CaptureThread.h"
#include "ControlThread.h"
#include "WebcamTracker.h"

class QPlainTextEdit;
class QSpinBox;
class QDoubleSpinBox;
class QComboBox;
class QSlider;
class QTimer;
class QPushButton;
class QLabel;

class ControlThread;
//class WebcamTracker;

class MainWindow : public QMainWindow {
	Q_OBJECT
public:
	explicit MainWindow(QWidget * parent = 0);

	public slots:
	bool fileExists(QString path);
	void connectDisconnect(bool checked);
	void outputSystemStatus();
	void outputLocalPosition();
	void outputGlobalPosition();
	void outputVFRHUD();
	void outputAll();

	void setModeApm(int mode);

	void onThrottleSpinBox(int throttlePercent);
	void onPitchSlider(int pitchPercent);
	void onRollSlider(int rollPercent);
	void onYawSlider(int yawPercent);

	//void onNewFrame(cv::Mat newFrame, int frameCount);

	//void onButtonArm();
	//void onButtonDisarm();
	//void onButtonFront();
	//void onButtonLeft();
	//void onButtonRight();
	//void onButtonBack();
	//void onButtonHold();
	//void onButtonRcReset();
	
	//void navigate();
	//void setROI();
	
private:
	int rollScale;
	int pitchScale;
	int yawScale;
	
	MAVLinkConnection*	connection;
	ImageTracker*		offlineTracking;
	WebcamTracker*		liveTracking;
	CaptureThread*		CapThread;
	ControlThread*		CtrlThread;
	

	QToolBar *        toolbar;
	QAction *         connectDisconnectAction;
	QWidget *         mainWidget;

	//QComboBox *       COMPort;

	QPushButton *buttonStreamAll;
	QPushButton *buttonStreamRaw;
	QPushButton *buttonStreamRc;
	QPushButton *buttonStreamOff;
	QPlainTextEdit *  output;

	QTimer*			  heartbeatTimer;

	QSlider *		  pitchSlider;
	QSlider *		  rollSlider;
	QSlider *		  yawSlider;
	QSpinBox *		  throttleSpinBox;

	QLabel *labelPitch;
	QLabel *labelThrottle;
	QLabel *labelRoll;
	QLabel *labelYaw;
	QLabel *labelMode;

	QPushButton *buttonArm;
	QPushButton *buttonDisarm;
	QPushButton *buttonFront;
	QPushButton *buttonLeft;
	QPushButton *buttonRight;
	QPushButton *buttonBack;
	QPushButton *buttonHold;
	QPushButton *buttonRcReset;

	QComboBox *       mavMode;

	//QSpinBox *        waypointTime;
	//QDoubleSpinBox *  waypointAcceptanceRadius;
	//QDoubleSpinBox *  waypointTrajectoryControl;
	//QDoubleSpinBox *  waypointYaw;
	//QDoubleSpinBox *  waypointLatitude;
	//QDoubleSpinBox *  waypointLongitude;
	//QDoubleSpinBox *  waypointAltitude;

	//QComboBox *       roiType;
	//QSpinBox *        roiMissionIndex;
	//QSpinBox *        roiTargetID;
	//QSpinBox *        roiIndex;
	//QDoubleSpinBox *  roiX;
	//QDoubleSpinBox *  roiY;
	//QDoubleSpinBox *  roiZ;
};

#endif