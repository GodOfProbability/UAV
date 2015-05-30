#include "MainWindow.h"
#include <QToolBar>
#include <QPlainTextEdit>
#include <QAction>
#include <QStatusBar>
#include <QTextStream>
#include <QBoxLayout>
#include <QGridLayout>
#include <QFormLayout>
#include <QLabel>
#include <QSpinBox>
#include <QDial>
#include <QGroupBox>
#include <QPushButton>
#include <QComboBox>
#include <QSlider>
#include <QTimer>
#include <QFileInfo>
#include <QtCore/QDebug>

#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <iostream>

#include "ConfigFile.h"

MainWindow::MainWindow(QWidget * parent)
: QMainWindow(parent),
  toolbar(0),
  mainWidget(0),
  output(0)
/*  waypointTime(0),
  waypointAcceptanceRadius(0),
  waypointTrajectoryControl(0),
  waypointYaw(0),
  waypointLatitude(0),
  waypointLongitude(0),
  waypointAltitude(0),
  roiType(0),
  roiMissionIndex(0),
  roiTargetID(0),
  roiIndex(0),
  roiX(0),
  roiY(0),
  roiZ(0)*/ {

	  if (fileExists("config.txt")) {
		  //Read Config File
		  ConfigFile cf("config.txt");
		  rollScale = cf.Value("MainWindow", "rollScale");
		  pitchScale = cf.Value("MainWindow", "pitchScale");
		  yawScale = cf.Value("MainWindow", "yawScale");
	  }
	  else {
		  //Initialize default values and display error message
		  int rollScale = 100;
		  int pitchScale = 100;
		  int yawScale = 100;
		  qDebug() << "MainWindow: No Configuration File! Using default values from constructor.";
	  }

	connection = new MAVLinkConnection(this);
	if (fileExists("config.txt")) {
		//Read Config File
		ConfigFile cf("config.txt");
		connection->baudRate = cf.Value("MAVLinkConnection", "baudRate");
	}
	else {
		qDebug() << "MAVLinkConnection: No Configuration File! Using default values from constructor.";
	}

	CapThread = new CaptureThread(connection);

	CtrlThread = new ControlThread(connection);
	if (fileExists("config.txt")) {
	//Read Config File
	ConfigFile cf("config.txt");
	CtrlThread->rollTargetLeft = cf.Value("Control", "rollTargetLeft");
	CtrlThread->rollTargetRight = cf.Value("Control", "rollTargetRight");
	CtrlThread->pitchTargetFront = cf.Value("Control", "pitchTargetFront");
	CtrlThread->pitchTargetBack = cf.Value("Control", "pitchTargetBack");
	CtrlThread->rampIncrement = cf.Value("Control", "rampIncrement");
	CtrlThread->transmissionDelay = cf.Value("Control", "transmissionDelay");
	CtrlThread->rampDelay = cf.Value("Control", "rampDelay");
	}
	else {
		qDebug() << "Control Thread: No Configuration File! Using default values from constructor.";
	}

	offlineTracking = new ImageTracker();
	if (fileExists("config.txt")) {
		//Read Config File
		ConfigFile cf("config.txt");
		offlineTracking->iLowH = cf.Value("ImageTracker", "iLowH");
		offlineTracking->iHighH = cf.Value("ImageTracker", "iHighH");
		offlineTracking->iLowS = cf.Value("ImageTracker", "iLowS");
		offlineTracking->iHighS = cf.Value("ImageTracker", "iHighS");
		offlineTracking->iLowV = cf.Value("ImageTracker", "iLowV");
		offlineTracking->iHighV = cf.Value("ImageTracker", "iHighV");
	}
	else {
		qDebug() << "ImageTracker: No Configuration File! Using default values from constructor.";
	}

	liveTracking = new WebcamTracker();
	if (fileExists("config.txt")) {
		//Read Config File
		ConfigFile cf("config.txt");
		liveTracking->iLowH = cf.Value("WebcamTracker", "iLowH");
		liveTracking->iHighH = cf.Value("WebcamTracker", "iHighH");
		liveTracking->iLowS = cf.Value("WebcamTracker", "iLowS");
		liveTracking->iHighS = cf.Value("WebcamTracker", "iHighS");
		liveTracking->iLowV = cf.Value("WebcamTracker", "iLowV");
		liveTracking->iHighV = cf.Value("WebcamTracker", "iHighV");
	}
	else {
		qDebug() << "WebcamTracker: No Configuration File! Using default values from constructor.";
	}

	heartbeatTimer = new QTimer();

	//Connect Heartbeat message to timer
	connect(heartbeatTimer, SIGNAL(timeout()), connection, SLOT(sendHeartBeat()));

    setMinimumWidth(640);
    toolbar = addToolBar("Connection");
    connectDisconnectAction = new QAction("&Connect", this);
    connectDisconnectAction->setCheckable(true);
    toolbar->addAction(connectDisconnectAction);
    connect(connectDisconnectAction, SIGNAL(toggled(bool)), connection, SLOT(connectDisconnect(bool)));
    connect(connectDisconnectAction, SIGNAL(toggled(bool)), this, SLOT(connectDisconnect(bool)));

    QAction * statusAction = new QAction("&Status", this);
    toolbar->addAction(statusAction);
    connect(statusAction, SIGNAL(triggered()), this, SLOT(outputSystemStatus()));

	QAction *LiveTrackOnAction = new QAction("&LiveTrackOn", this);
	toolbar->addAction(LiveTrackOnAction);
	connect(LiveTrackOnAction, SIGNAL(triggered()), liveTracking, SLOT(StartTracking()));

	QAction *LiveTrackOffAction = new QAction("&LiveTrackOff", this);
	toolbar->addAction(LiveTrackOffAction);
	connect(LiveTrackOffAction, SIGNAL(triggered()), liveTracking, SLOT(StopTracking()));

	QAction *OfflineTrackOnAction = new QAction("&OfflineTrackOn", this);
	toolbar->addAction(OfflineTrackOnAction);
	connect(OfflineTrackOnAction, SIGNAL(triggered()), offlineTracking, SLOT(StartTracking()));

	QAction *OfflineTrackOffAction = new QAction("&OfflineTrackOff", this);
	toolbar->addAction(OfflineTrackOffAction);
	connect(OfflineTrackOffAction, SIGNAL(triggered()), offlineTracking, SLOT(StopTracking()));

	QAction *StartCapAction = new QAction("&StartCapture", this);
	toolbar->addAction(StartCapAction);
	connect(StartCapAction, SIGNAL(triggered()), CapThread, SLOT(StartCapture()));

	QAction *StopCapAction = new QAction("&StopCapture", this);
	toolbar->addAction(StopCapAction);
	connect(StopCapAction, SIGNAL(triggered()), CapThread, SLOT(StopCapture()));

	QAction *StartCtrlAction = new QAction("&StartCtrl", this);
	toolbar->addAction(StartCtrlAction);
	connect(StartCtrlAction, SIGNAL(triggered()), CtrlThread, SLOT(StartControl()));

	QAction *StopCtrlAction = new QAction("&StopCtrl", this);
	toolbar->addAction(StopCtrlAction);
	connect(StopCtrlAction, SIGNAL(triggered()), CtrlThread, SLOT(StopControl()));

	QList<QAction *> actions = toolbar->actions();
    for(QList<QAction *>::iterator i = ++actions.begin(); i != actions.end(); ++i)
        (*i)->setEnabled(false);

	//Connect Channel 7 of RX to Live Tracker and Capture
	connect(connection, SIGNAL(ch7ToggleOn()), liveTracking, SLOT(StartTracking()));
	connect(connection, SIGNAL(ch7ToggleOff()), liveTracking, SLOT(StopTracking()));
	connect(connection, SIGNAL(ch7ToggleOn()), CapThread, SLOT(StartCapture()));
	connect(connection, SIGNAL(ch7ToggleOff()), CapThread, SLOT(StopCapture()));

	//Connect ImageTracker
	connect(offlineTracking, SIGNAL(TargetRight()), CtrlThread, SLOT(MoveRight()));
	connect(offlineTracking, SIGNAL(TargetLeft()), CtrlThread, SLOT(MoveLeft()));
	connect(offlineTracking, SIGNAL(TargetFront()), CtrlThread, SLOT(MoveFront()));
	connect(offlineTracking, SIGNAL(TargetBack()), CtrlThread, SLOT(MoveBack()));
	connect(offlineTracking, SIGNAL(TargetCenter()), CtrlThread, SLOT(MoveHold()));

	//Connect WebcamTracker
	connect(liveTracking, SIGNAL(TargetRight()), CtrlThread, SLOT(MoveRight()));
	connect(liveTracking, SIGNAL(TargetLeft()), CtrlThread, SLOT(MoveLeft()));
	connect(liveTracking, SIGNAL(TargetFront()), CtrlThread, SLOT(MoveFront()));
	connect(liveTracking, SIGNAL(TargetBack()), CtrlThread, SLOT(MoveBack()));
	connect(liveTracking, SIGNAL(TargetCenter()), CtrlThread, SLOT(MoveHold()));

	//Q_DECLARE_METATYPE(cv::Mat);
	qRegisterMetaType<cv::Mat>("cv::Mat");
	connect(liveTracking, SIGNAL(newFrame(cv::Mat, int)), CapThread, SLOT(onNewFrame(cv::Mat, int)));

	connect(connection, SIGNAL(statusChanged(QString const &)), statusBar(), SLOT(showMessage(QString const &)));
	connect(connection, SIGNAL(armed(QString const &)), statusBar(), SLOT(showMessage(QString const &)));
	connect(connection, SIGNAL(disarmed(QString const &)), statusBar(), SLOT(showMessage(QString const &)));
	connect(connection, SIGNAL(flightMode(QString const &)), statusBar(), SLOT(showMessage(QString const &)));
	connect(connection, SIGNAL(commandAcknowledged(QString const &)), statusBar(), SLOT(showMessage(QString const &)));

	connect(connection, SIGNAL(localPositionChanged()), this, SLOT(outputLocalPosition()));
	connect(connection, SIGNAL(globalPositionChanged()), this, SLOT(outputGlobalPosition()));
	connect(connection, SIGNAL(VFRHUDChanged()), this, SLOT(outputVFRHUD()));
	//connect(connection, SIGNAL(heartbeatReceived()), this, SLOT(outputAll()));
	
	mainWidget = new QWidget(this);
    setCentralWidget(mainWidget);

    QBoxLayout * mainLayout = new QBoxLayout(QBoxLayout::LeftToRight);
    mainWidget->setLayout(mainLayout);

	mainWidget->hide();

	//COMPort = new QComboBox;
	//QFormLayout * COMPortLayout = new QFormLayout;
	//foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
	//	COMPort->addItem(info.portName());
	//}
	//COMPortLayout->addRow("&COMPort", COMPort);

	//QGroupBox * COMPortGroupBox = new QGroupBox("Select COM Port");
	//COMPortGroupBox->setLayout(COMPortLayout);

	//mainLayout->addWidget(COMPortGroupBox);

	buttonStreamAll = new QPushButton("ALL");
	buttonStreamRaw = new QPushButton("Raw Sensors");
	buttonStreamRc = new QPushButton("RC Channels");
	buttonStreamOff = new QPushButton("OFF");
	output = new QPlainTextEdit(this);
	output->setReadOnly(true);

	QGridLayout *StreamLayout = new QGridLayout;
	StreamLayout->addWidget(buttonStreamAll, 0, 0);
	StreamLayout->addWidget(buttonStreamRaw, 0, 1);
	StreamLayout->addWidget(buttonStreamRc, 1, 0);
	StreamLayout->addWidget(buttonStreamOff, 1, 1);
	StreamLayout->addWidget(output, 2, 0, 4, 2);

	QGroupBox *StreamGroupBox = new QGroupBox("Data Stream");
	StreamGroupBox->setLayout(StreamLayout);
	mainLayout->addWidget(StreamGroupBox);

	//Connect Buttons
	connect(buttonStreamAll, SIGNAL(clicked()), connection, SLOT(streamAll()));
	connect(buttonStreamRaw, SIGNAL(clicked()), connection, SLOT(streamRawSensors()));
	connect(buttonStreamRc, SIGNAL(clicked()), connection, SLOT(streamRcChannels()));
	connect(buttonStreamOff, SIGNAL(clicked()), connection, SLOT(streamOff()));

	//RcControl Section
	pitchSlider = new QSlider(Qt::Vertical, this);
	//Invert pitch
	pitchSlider->setInvertedAppearance(true);
	pitchSlider->setRange(-1 * pitchScale, 1 * pitchScale);
	pitchSlider->setValue(0);
	rollSlider = new QSlider(Qt::Horizontal, this);
	rollSlider->setRange(-1 * rollScale, 1 * rollScale);
	rollSlider->setValue(0);
	yawSlider = new QSlider(Qt::Horizontal, this);
	yawSlider->setRange(-1 * yawScale, 1 * yawScale);
	yawSlider->setValue(0);
	throttleSpinBox = new QSpinBox;
	throttleSpinBox->setRange(0, 100);

	labelPitch = new QLabel("Pitch");
	labelThrottle = new QLabel("Throttle");
	labelRoll = new QLabel("Roll");
	labelYaw = new QLabel("Yaw");
	labelMode = new QLabel("Set Mode");

	buttonArm = new QPushButton("Arm");
	buttonDisarm = new QPushButton("Disarm");
	buttonFront = new QPushButton("Front");
	buttonLeft = new QPushButton("Left");
	buttonRight = new QPushButton("Right");
	buttonBack = new QPushButton("Back");
	buttonHold = new QPushButton("Hold");
	buttonRcReset = new QPushButton("Reset");

	mavMode = new QComboBox;
	mavMode->addItem("Stabilize");
	mavMode->addItem("AltHold");
	mavMode->addItem("Auto");
	mavMode->addItem("Guided");
	mavMode->addItem("Loiter");
	mavMode->addItem("RTL");
	mavMode->addItem("Land");
	
	QGridLayout *RcCtrlLayout = new QGridLayout;
	RcCtrlLayout->addWidget(labelThrottle, 0, 0);
	RcCtrlLayout->addWidget(throttleSpinBox, 0, 1);
	RcCtrlLayout->addWidget(labelPitch, 1, 0);
	RcCtrlLayout->addWidget(pitchSlider, 1, 1,3,1,Qt::AlignCenter);
	RcCtrlLayout->addWidget(labelRoll, 4, 0);
	RcCtrlLayout->addWidget(rollSlider, 4, 1,Qt::AlignCenter);
	RcCtrlLayout->addWidget(labelYaw, 5, 0);
	RcCtrlLayout->addWidget(yawSlider, 5, 1, Qt::AlignCenter);

	RcCtrlLayout->addWidget(labelMode, 0, 2,Qt::AlignCenter);
	RcCtrlLayout->addWidget(mavMode, 0, 3);
	RcCtrlLayout->addWidget(buttonArm, 1, 2);
	RcCtrlLayout->addWidget(buttonDisarm, 1, 3);
	RcCtrlLayout->addWidget(buttonRcReset, 2, 2);
	RcCtrlLayout->addWidget(buttonHold, 2, 3);
	RcCtrlLayout->addWidget(buttonFront, 3, 2, 1, 2);
	RcCtrlLayout->addWidget(buttonLeft, 4, 2);
	RcCtrlLayout->addWidget(buttonRight, 4, 3);
	RcCtrlLayout->addWidget(buttonBack, 5, 2, 1, 2);


	//Alternative Layout
	//QFormLayout * RcCtrlLayout = new QFormLayout;
	//RcCtrlLayout->addRow("&Throttle:", throttleSpinBox);
	//RcCtrlLayout->addRow("&Pitch:", pitchSlider);
	//RcCtrlLayout->addRow("&Roll:", rollSlider);
	//RcCtrlLayout->addRow("&Yaw:", yawSlider);
	//RcCtrlLayout->addRow("&Mode:", mavMode);
		
	QGroupBox *RcCtrlGroupBox = new QGroupBox("RC Control");
	RcCtrlGroupBox->setLayout(RcCtrlLayout);
	mainLayout->addWidget(RcCtrlGroupBox);

	//Connect Buttons
	connect(buttonArm, SIGNAL(clicked()), connection, SLOT(arm()));
	connect(buttonDisarm, SIGNAL(clicked()), connection, SLOT(disarm()));
	connect(buttonFront, SIGNAL(clicked()), CtrlThread, SLOT(MoveFront()));
	connect(buttonLeft, SIGNAL(clicked()), CtrlThread, SLOT(MoveLeft()));
	connect(buttonRight, SIGNAL(clicked()), CtrlThread, SLOT(MoveRight()));
	connect(buttonBack, SIGNAL(clicked()), CtrlThread, SLOT(MoveBack()));
	connect(buttonHold, SIGNAL(clicked()), CtrlThread, SLOT(MoveHold()));
	connect(buttonRcReset, SIGNAL(clicked()), CtrlThread, SLOT(setRcReset()));

	//Connect Channel 6 of RX to Override Reset and Stop Tracking
	connect(connection, SIGNAL(ch6High()), CtrlThread, SLOT(setRcReset()));
	connect(connection, SIGNAL(ch6High()), liveTracking, SLOT(StopTracking()));

	//Connect Sliders
	connect(throttleSpinBox, SIGNAL(valueChanged(int)), this, SLOT(onThrottleSpinBox(int)));
	connect(pitchSlider, SIGNAL(valueChanged(int)), this, SLOT(onPitchSlider(int)));
	connect(rollSlider, SIGNAL(valueChanged(int)), this, SLOT(onRollSlider(int)));
	connect(yawSlider, SIGNAL(valueChanged(int)), this, SLOT(onYawSlider(int)));

	//Connect Dropdown
	connect(mavMode, SIGNAL(activated(int)), this, SLOT(setModeApm(int)));

//	//Waypoint Section
//	waypointTime = new QSpinBox(this);
//    waypointTime->setRange(1, 3600);
//    waypointTime->setSuffix(" s");
//
//    waypointAcceptanceRadius = new QDoubleSpinBox(this);
//    waypointAcceptanceRadius->setRange(-1000, 1000);
//    waypointAcceptanceRadius->setSuffix(" m");
//
//    waypointTrajectoryControl = new QDoubleSpinBox(this);
//    waypointTrajectoryControl->setRange(-1000, 1000);
//    waypointTrajectoryControl->setSuffix(" m");
//
//    waypointYaw = new QDoubleSpinBox(this);
//    waypointYaw->setRange(0, 360);
//    waypointYaw->setSuffix(" °");
//
//    waypointLatitude = new QDoubleSpinBox(this);
//    waypointLatitude->setRange(0, 360);
//    waypointLatitude->setSuffix(" °");
//
//    waypointLongitude = new QDoubleSpinBox(this);
//    waypointLongitude->setRange(0, 360);
//    waypointLongitude->setSuffix(" °");
//
//    waypointAltitude = new QDoubleSpinBox(this);
//    waypointAltitude->setRange(0, 1000);
//    waypointAltitude->setSuffix(" m");
//
//    QPushButton * waypointNavigate = new QPushButton("Navigate");
//    connect(waypointNavigate, SIGNAL(clicked()), this, SLOT(navigate()));
//
//    QFormLayout * waypointLayout = new QFormLayout;
//    waypointLayout->addRow("&Time:", waypointTime);
//    waypointLayout->addRow("&Acceptance Radius:", waypointAcceptanceRadius);
//    waypointLayout->addRow("&Trajectory Control:", waypointTrajectoryControl);
//    waypointLayout->addRow("&Yaw:", waypointYaw);
//    waypointLayout->addRow("&Latitude:", waypointLatitude);
//    waypointLayout->addRow("&Longitude:", waypointLongitude);
//    waypointLayout->addRow("&Altitude", waypointAltitude);
//    waypointLayout->addRow(waypointNavigate);
//
//    QGroupBox * waypointGroupBox = new QGroupBox("Navigate to Waypoint");
//    waypointGroupBox->setLayout(waypointLayout);
//
//    mainLayout->addWidget(waypointGroupBox);
//
//    roiType = new QComboBox;
//    roiType->addItem("None");
//    roiType->addItem("Next Mission");
//    roiType->addItem("Given Mission Index");
//    roiType->addItem("Location");
//    roiType->addItem("Target");
//
//    roiMissionIndex = new QSpinBox;
//    roiMissionIndex->setRange(0, 1000);
//
//    roiTargetID = new QSpinBox;
//    roiTargetID->setRange(0, 1000);
//
//    roiIndex = new QSpinBox;
//    roiIndex->setRange(0, 1000);
//
//    roiX = new QDoubleSpinBox;
//    roiY = new QDoubleSpinBox;
//    roiZ = new QDoubleSpinBox;
//
//    QPushButton * roiSetROI = new QPushButton("Set ROI");
//    connect(roiSetROI, SIGNAL(clicked()), this, SLOT(setROI()));
//
//    QFormLayout * roiLayout = new QFormLayout;
//    roiLayout->addRow("&ROI", roiType);
//    roiLayout->addRow("&Mission Index", roiMissionIndex);
//    roiLayout->addRow("&Target ID", roiTargetID);
//    roiLayout->addRow("&ROI Index", roiIndex);
//    roiLayout->addRow("&X", roiX);
//    roiLayout->addRow("&Y", roiY);
//    roiLayout->addRow("&Z", roiZ);
//    roiLayout->addRow(roiSetROI);
//
//    QGroupBox * roiGroupBox = new QGroupBox("Region of Interest");
//    roiGroupBox->setLayout(roiLayout);
//
//    mainLayout->addWidget(roiGroupBox);
}

bool MainWindow::fileExists(QString path) {
	QFileInfo checkFile(path);
	// check if file exists and if yes: Is it really a file and no directory?
	if (checkFile.exists() && checkFile.isFile()) {
		return true;
	}
	else {
		return false;
	}
}

void MainWindow::connectDisconnect(bool checked) {
    connectDisconnectAction->setText(checked? "&Disconnect" : "Connect");
	//Enable depending on active connection
    QList<QAction *> actions = toolbar->actions();
    for(QList<QAction *>::iterator i = ++actions.begin(); i != actions.end(); ++i)
        (*i)->setEnabled(checked);
	if (checked) mainWidget->show();
	else mainWidget->hide();
	//Enable/Disable Heartbeat Timer
	if (heartbeatTimer->isActive()) heartbeatTimer->stop();
	else heartbeatTimer->start(1000);
}

void MainWindow::outputSystemStatus() {
    QString buffer;
    QTextStream text(&buffer);
    for(int i = 0; i < MAVLinkConnection::SENSORS_COUNT; ++i) {
        MAVLinkConnection::Sensor const sensor = static_cast<MAVLinkConnection::Sensor>(i);
        if(connection->isSensorPresent(sensor))
            text << "Sensor \"" << connection->nameOfSensor(sensor) << "\" is "
                 << (connection->isSensorEnabled(sensor)? "enabled" : "not enabled") << " and "
                 << (connection->isSensorOperational(sensor)? "operational\n" : "not operational\n");
    }
    text << "Battery is at: " << connection->batteryRemaining() << "%\n";
    output->appendPlainText(*text.string());
}

void MainWindow::outputLocalPosition() {
    QString buffer;
    QTextStream text(&buffer);

    MAVLinkConnection::LocalPosition const localPosition = connection->localPosition();
    text << "Local Position timestamp = " << localPosition.timestamp
         << ", position(" << localPosition.x << ',' << localPosition.y << ',' << localPosition.z
         << "), velocity(" << localPosition.velocityX << ',' << localPosition.velocityY << ',' << localPosition.velocityZ << ")\n";
    output->appendPlainText(*text.string());
}
    
void MainWindow::outputGlobalPosition() {
    QString buffer;
    QTextStream text(&buffer);

    MAVLinkConnection::GlobalPosition const globalPosition = connection->globalPosition();
    text << "Global Position timestamp = " << globalPosition.timestamp
         << ", lattitude = " << globalPosition.lattitude
         << ", longitude = " << globalPosition.longitude
         << ", altitude = " << globalPosition.altitude
         << ", relative_altitude = " << globalPosition.relative_altitude
         << ", velocityX = " << globalPosition.velocityX
         << ", velocityY = " << globalPosition.velocityY
         << ", velocityZ = " << globalPosition.velocityZ
         << ", heading = " << globalPosition.heading << '\n';

    output->appendPlainText(*text.string());
}

void MainWindow::outputVFRHUD() {
    QString buffer;
    QTextStream text(&buffer);

    MAVLinkConnection::VFRHUD const vfrHUD = connection->vfrHUD();
	text << "VFR HUD airspeed = " << vfrHUD.airspeed
		 << ", groundspeed = " << vfrHUD.groundspeed
		 << ", heading = " << vfrHUD.heading
		 << ", throttlePercentage = " << vfrHUD.throttlePercentage
		 << ", altitude = " << vfrHUD.altitude
		 << ", climb = " << vfrHUD.climb << '\n';
		
	output->appendPlainText(*text.string());

}

void MainWindow::outputAll() {
	QString buffer;
	QTextStream text(&buffer);
	//output_counter++;
	MAVLinkConnection::VFRHUD const vfrHUD = connection->vfrHUD();
	MAVLinkConnection::GlobalPosition const globalPosition = connection->globalPosition();
	MAVLinkConnection::LocalPosition const localPosition = connection->localPosition();
	text //<< output_counter << '\n'
  		 << "VFR HUD airspeed = " << vfrHUD.airspeed
		 << ", groundspeed = " << vfrHUD.groundspeed
		 << ", heading = " << vfrHUD.heading
		 << ", throttlePercentage = " << vfrHUD.throttlePercentage
		 << ", altitude = " << vfrHUD.altitude
		 << ", climb = " << vfrHUD.climb << '\n'
		 << "Local Position timestamp = " << localPosition.timestamp
		 << ", position(" << localPosition.x << ',' << localPosition.y << ',' << localPosition.z
		 << "), velocity(" << localPosition.velocityX << ',' << localPosition.velocityY << ',' << localPosition.velocityZ << ")\n"
		 << "Global Position timestamp = " << globalPosition.timestamp
		 << ", lattitude = " << globalPosition.lattitude
		 << ", longitude = " << globalPosition.longitude
		 << ", altitude = " << globalPosition.altitude
		 << ", relative_altitude = " << globalPosition.relative_altitude
		 << ", velocityX = " << globalPosition.velocityX
		 << ", velocityY = " << globalPosition.velocityY
		 << ", velocityZ = " << globalPosition.velocityZ
		 << ", heading = " << globalPosition.heading << '\n';

	output->appendPlainText(*text.string());

	//qDebug() << *text.string();
	//write2file(*text.string());

}

void MainWindow::setModeApm(int mode) {
	switch(mode) {
	case 0:
		connection->setModeApmStabilize();
		//qDebug() << "Stabilize";
		break;
	case 1:
		connection->setModeApmAltHold();
		//qDebug() << "AltHold";
		break;
	case 2:
		connection->setModeApmAuto();
		//qDebug() << "Auto";
		break;
	case 3:
		connection->setModeApmGuided();
		//qDebug() << "Guided";
		break;
	case 4:
		connection->setModeApmLoiter();
		//qDebug() << "Loiter";
		break;
	case 5:
		connection->setModeApmRtl();
		//qDebug() << "RTL";
		break;
	case 6:
		connection->setModeApmLand();
		//qDebug() << "Land";
		break;
	default:
		connection->setModeApmStabilize();
		break;

	}

}

void MainWindow::onThrottleSpinBox(int throttlePercent) {
	int throttleRaw = 1000 + (throttlePercent * 10);
	qDebug() << "Throttle Percent: " << throttlePercent << " Raw: " << throttleRaw;
	connection->sendThrottle(throttleRaw);
}
void MainWindow::onPitchSlider(int pitchPercent) {
	int pitchRaw = 1500 + (pitchPercent * 5);
	qDebug() << "Pitch Percent: " << pitchPercent << " Raw: " << pitchRaw;
	connection->sendPitch(pitchRaw);
}
void MainWindow::onRollSlider(int rollPercent) {
	int rollRaw = 1500 + (rollPercent * 5);
	qDebug() << "Roll Percent: " << rollPercent << " Raw: " << rollRaw;
	connection->sendRoll(rollRaw);
}
void MainWindow::onYawSlider(int yawPercent) {
	int yawRaw = 1500 + (yawPercent * 5);
	qDebug() << "Yaw Percent: " << yawPercent << " Raw: " << yawRaw;
	connection->sendYaw(yawRaw);
}

//void MainWindow::onNewFrame(cv::Mat newFrame, int frameCount)
//{
//	qDebug() << "Frame received: " << frameCount;
//}

//void MainWindow::navigate() {
//    connection->navigateToWaypoint(waypointTime->value(),
//                                  waypointAcceptanceRadius->value(),
//                                  waypointTrajectoryControl->value(),
//                                  waypointYaw->value(),
//                                  waypointLatitude->value(),
//                                  waypointLongitude->value(),
//                                  waypointAltitude->value());
//}
//
//void MainWindow::setROI() {
//    connection->setROI(static_cast<MAVLinkConnection::ROI>(roiType->currentIndex()),
//                      roiMissionIndex->value(),
//                      roiTargetID->value(),
//                      roiIndex->value(),
//                      roiX->value(),
//                      roiY->value(),
//                      roiZ->value());
//}

/*
void MainWindow::outputGPS() {
QString buffer;
QTextStream text(&buffer);

QVector<MAVLinkConnection::Satellite> const gps = connection->gps();
int const satellitesCount = gps.size();

for(int i = 0; i < satellitesCount; ++i)
text << "Satellite " << gps[i].id << " is " << (gps[i].isUsed? "used" : "not used")
<< " with elevation = " << gps[i].elevation << ", azimuth = " << gps[i].azimuth << ", SNR = " << gps[i].snr << '\n';

output->appendPlainText(*text.string());
}
*/

//void MainWindow::onButtonArm() {
//	connection->arm();
//}
//
//void MainWindow::onButtonDisarm() {
//	connection->disarm();
//}
//
//void MainWindow::onButtonFront() {
//	CtrlThread->MoveFront();
//}
//
//void MainWindow::onButtonLeft() {
//	CtrlThread->MoveLeft();
//}
//
//void MainWindow::onButtonRight() {
//	CtrlThread->MoveRight();
//}
//
//void MainWindow::onButtonBack() {
//	CtrlThread->MoveBack();
//}
//
//void MainWindow::onButtonHold() {
//	CtrlThread->MoveHold();
//}
//
//void MainWindow::onButtonRcReset() {
//	CtrlThread->setRcReset();
//}

