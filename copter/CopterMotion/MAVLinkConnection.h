#ifndef MAVLINKCONNECTION_H
#define MAVLINKCONNECTION_H

#include <mavlink.h>
#include <QSerialPort>
#include <QVector>
#include <QDateTime>

class MAVLinkConnection
: public QObject {
    Q_OBJECT
public:
    enum Sensor {
        SENSOR_3D_GYRO,
        SENSOR_3D_ACCEL,
        SENSOR_3D_MAG,
        SENSOR_ABSOLUTE_PRESSURE,
        SENSOR_DIFFERENTIAL_PRESSURE,
        SENSOR_GPS,
        SENSOR_OPTICAL_FLOW,
        SENSOR_VISION_POSITION,
        SENSOR_LASER_POSITION,
        SENSOR_EXTERNAL_GROUND_TRUTH,
        SENSOR_3D_ANGULAR_RATE_CONTROL,
        SENSOR_ATTITUDE_STABILIZATION,
        SENSOR_YAW_POSITION,
        SENSOR_Z_ALTITUDE_CONTROL,
        SENSOR_XY_POSITION_CONTROL,
        SENSOR_MOTOR_OUTPUTS,
        SENSOR_RC_RECEIVER,
        SENSOR_3D_GYRO2,
        SENSOR_3D_ACCEL2,
        SENSOR_3D_MAG2,
        SENSORS_COUNT
    };
    MAVLinkConnection(QObject * parent);
	int baudRate;
    QString nameOfSensor(Sensor const sensor) const;
    bool isSensorPresent(Sensor const sensor) const;
    bool isSensorEnabled(Sensor const sensor) const;
    bool isSensorOperational(Sensor const sensor) const;
    // Returns value from 0% to 100% indicating the percentage of the battery remaining.
    int batteryRemaining() const;
	
	bool systemIsArmed=0;

	quint64 SystemTimeUsecs()
		{
			return SystemTimeMillisecs() * 1000;
		}

	quint64 SystemTimeMillisecs()
		{
			return static_cast<quint64>(QDateTime::currentMSecsSinceEpoch());
		}

	qreal SystemTimeSecs()
		{
			return static_cast<qreal>(SystemTimeMillisecs()) / 1000.0f;
		}

	qint64 lastHeartbeat;

    struct Satellite {
        int   id;
        bool  isUsed; // true if used for localization
        int   elevation; // 0: right on top of receiver, 90: on the horizon
        float azimuth; // in degrees
        int   snr;
    };
    QVector<Satellite> satellites() const;

    struct LocalPosition {
        float timestamp;
        float x, y, z;
        float velocityX, velocityY, velocityZ;
    };

    LocalPosition localPosition() const;

    struct GlobalPosition {
        float timestamp;
        float lattitude;
        float longitude;
        float altitude; // meters
        float relative_altitude; // meters
        float velocityX, velocityY, velocityZ; // m/s
        float heading; // in degrees
    };
    
    GlobalPosition globalPosition() const;

    struct VFRHUD {
        float airspeed; // m/s
        float groundspeed; // m/s
        float heading; // degrees
        float throttlePercentage; // 0 to 100
        float altitude; // meters
        float climb; // m/s
    };

    VFRHUD vfrHUD() const;

    void navigateToWaypoint(float time, // seconds
                            float acceptanceRadius, // meters
                            float trajectoryControl, // 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
                            float yaw,
                            float latitude,
                            float longitude,
                            float altitude);

    enum ROI {
        ROI_NONE = MAV_ROI_NONE,
        ROI_NEXT_MISSION = MAV_ROI_WPNEXT,
        ROI_GIVEN_MISSION_INDEX = MAV_ROI_WPINDEX,
        ROI_LOCATION = MAV_ROI_LOCATION,
        ROI_TARGET = MAV_ROI_TARGET
    }; 

    void setROI(ROI roi,
                int missionIndex, // Only used when roi == ROI_GIVEN_MISSION_INDEX
                int targetID, // Only used when roi == ROI_TARGET
                int roiIndex,
                float x,
                float y,
                float z);

    enum Mode {
        MODE_PREFLIGHT = MAV_MODE_PREFLIGHT, // System is not ready to fly, booting, calibrating, etc. No flag is set. | 
        MODE_MANUAL_DISARMED = MAV_MODE_MANUAL_DISARMED, // System is allowed to be active, under manual (RC) control, no stabilization | 
        MODE_TEST_DISARMED = MAV_MODE_TEST_DISARMED, // UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | 
        MODE_STABILIZE_DISARMED = MAV_MODE_STABILIZE_DISARMED, // System is allowed to be active, under assisted RC control. | 
        MODE_GUIDED_DISARMED = MAV_MODE_GUIDED_DISARMED, // System is allowed to be active, under autonomous control, manual setpoint | 
        MODE_AUTO_DISARMED = MAV_MODE_AUTO_DISARMED, // System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs) | 
        MODE_MANUAL_ARMED = MAV_MODE_MANUAL_ARMED, // System is allowed to be active, under manual (RC) control, no stabilization | 
        MODE_TEST_ARMED = MAV_MODE_TEST_ARMED, // UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | 
        MODE_STABILIZE_ARMED = MAV_MODE_STABILIZE_ARMED, // System is allowed to be active, under assisted RC control. | 
        MODE_GUIDED_ARMED = MAV_MODE_GUIDED_ARMED, // System is allowed to be active, under autonomous control, manual setpoint | 
        MODE_AUTO_ARMED = MAV_MODE_AUTO_ARMED, // System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs) | 
        MODE_ENUM_END = MAV_MODE_ENUM_END
    };

	void sendMessage(mavlink_message_t message);
	void setMode(uint8_t base_mode, uint32_t custom_mode);

	
	//void goAutonomous();
	//void goManual();
	//void toggleAutonomy();
	//void setMode(uint8_t newBaseMode, uint32_t newCustomMode);
	//void setModeArm(uint8_t newBaseMode, uint32_t newCustomMode);
	
public slots:
    // This is a convenience slot to make it easier to connect directly from a toggle button. It will call connectToDevice() or disconnectFromDevice() internally depending on its first argument.
    void connectDisconnect(bool shouldConnect);
    void connectToDevice();
    void disconnectFromDevice();
    void streamAll();
	void streamRawSensors();
	void streamRcChannels();
	void streamOff();
	void arm();
	void disarm();
	void sendHeartBeat();
	void RcChannelsOverride(int ch1, int ch2, int ch3, int ch4, int ch5, int ch6, int ch7, int ch8);
	void sendThrottle(int val);
	void sendPitch(int val);
	void sendRoll(int val);
	void sendYaw(int val);
	void sendRc(int roll, int pitch, int throttle, int yaw);
	void sendRc(int roll, int pitch, int throttle, int yaw, int ch5, int ch6, int ch7, int ch8);
	void sendRcReset();

	void setModeApmStabilize();
	void setModeApmAltHold();
	void setModeApmAuto();
	void setModeApmGuided();
	void setModeApmLoiter();
	void setModeApmRtl();
	void setModeApmLand();

	void RequestAutoPilotCapabilities();
	void TargetControlCommands();
	void setTargetControlCommands(double x, double y, double z, double roll, double pitch, double yaw);
	void setManualControlCommands(float roll, float pitch, float yaw, float thrust);
	
	//void armSystem();
	//void disarmSystem();
	//void toggleArmedState();
    
private slots:
    void readData();
signals:
    void connectionError(QString const & error);
    void statusChanged(QString const & status);
	void armed(QString const & armed);
	void disarmed(QString const & disarmed);
	void flightMode(QString const & customMode);
    void systemStatusChanged();
    void localPositionChanged();
    void globalPositionChanged();
    void VFRHUDChanged();
    void commandAcknowledged(QString const & result);
	void heartbeatReceived();
	void ch6High();
	void ch7ToggleOn();
	void ch7ToggleOff();
	void ch8ToggleOn();
	void ch8ToggleOff();
private:
	void sendCommand(MAV_CMD command, float param1, float param2, float param3, float param4, float param5, float param6, float param7, int confirmation, int component);
    void sendCommand(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7);
	void sendCommand(MAV_CMD command);

private:
    QSerialPort            port;
	uint8_t				   base_mode;                 
	uint32_t			   custom_mode;         
    uint8_t                target_system;
    uint8_t                target_component;
    mavlink_heartbeat_t    heartbeat;
	mavlink_system_time_t  system_time;
    mavlink_system_t       system;
    mavlink_sys_status_t   sys_status;
    mavlink_statustext_t   statustext;
    mavlink_attitude_t     attitude;
    mavlink_gps_status_t   gps_status;
    mavlink_gps_raw_int_t  gps_data;
    mavlink_local_position_ned_t  local_position;
    mavlink_global_position_int_t global_position;
    mavlink_raw_imu_t      imu;
    mavlink_raw_pressure_t pressure;
    mavlink_vfr_hud_t      vfr_hud;
    mavlink_command_ack_t  command_ack;
	mavlink_rc_channels_raw_t rc_channels_raw;
	mavlink_autopilot_version_t autopilot_version;
};

#endif

