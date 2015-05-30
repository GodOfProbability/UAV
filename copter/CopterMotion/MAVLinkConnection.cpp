#include "MAVLinkConnection.h"

#include <QtCore/QDebug>
 
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <QtConcurrent/QtConcurrentRun>
//#include <QtConcurrent / QtConcurrent>

#include <time.h>
#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

// #include <chrono>
// #include <thread>

#include <iostream>
using namespace std;

QT_USE_NAMESPACE

MAVLinkConnection::MAVLinkConnection(QObject * parent)
: QObject(parent),
  port(parent),
  target_system(0),
  target_component(0)  {

    memset(&heartbeat, 0, sizeof(heartbeat));
    memset(&system, 0, sizeof(system));
    memset(&sys_status, 0, sizeof(sys_status));
    memset(&statustext, 0, sizeof(statustext));
    memset(&attitude, 0, sizeof(attitude));
    memset(&gps_status, 0, sizeof(gps_status));
    memset(&gps_data, 0, sizeof(gps_data));
    memset(&local_position, 0, sizeof(local_position));
    memset(&global_position, 0, sizeof(global_position));
    memset(&imu, 0, sizeof(imu));
    memset(&pressure, 0, sizeof(pressure));
    memset(&vfr_hud, 0, sizeof(vfr_hud));
	memset(&autopilot_version, 0, sizeof(autopilot_version));

    system.sysid  = 255;  //Mission Planner ID
    system.compid = MAV_COMP_ID_ALL;
	//system.type   = MAV_TYPE_GENERIC;
	baudRate = QSerialPort::Baud57600;
}
 
QString MAVLinkConnection::nameOfSensor(Sensor const sensor) const {
    switch(sensor) {
        case SENSOR_3D_GYRO:
            return "3D gyro";
        case SENSOR_3D_ACCEL:
            return "3D accelerometer";
        case SENSOR_3D_MAG:
            return "3D magnetometer";
        case SENSOR_ABSOLUTE_PRESSURE:
            return "Absolute pressure";
        case SENSOR_DIFFERENTIAL_PRESSURE:
            return "Differential pressure";
        case SENSOR_GPS:
            return "GPS";
        case SENSOR_OPTICAL_FLOW:
            return "Optical flow";
        case SENSOR_VISION_POSITION:
            return "Computer vision position";
        case SENSOR_LASER_POSITION:
            return "Laser based position";
        case SENSOR_EXTERNAL_GROUND_TRUTH:
            return "External ground truth (Vicon or Leica)";
        case SENSOR_3D_ANGULAR_RATE_CONTROL:
            return "3D angular rate control";
        case SENSOR_ATTITUDE_STABILIZATION:
            return "Attitude stabilization";
        case SENSOR_YAW_POSITION:
            return "Yaw position";
        case SENSOR_Z_ALTITUDE_CONTROL:
            return "Z/altitude control";
        case SENSOR_XY_POSITION_CONTROL:
            return "X/Y position control";
        case SENSOR_MOTOR_OUTPUTS:
            return "Motor outputs/control";
        case SENSOR_RC_RECEIVER:
            return "RC receiver";
        case SENSOR_3D_GYRO2:
            return "2nd 3D gyro";
        case SENSOR_3D_ACCEL2:
            return "2nd 3D accelerometer";
        case SENSOR_3D_MAG2:
            return "2nd 3D magnetometer";
        default:
            return "Unknown sensor";
    };
}

bool MAVLinkConnection::isSensorPresent(Sensor const sensor) const {
    return sys_status.onboard_control_sensors_present & (0x01 << sensor);
}

bool MAVLinkConnection::isSensorEnabled(Sensor const sensor) const {
    return sys_status.onboard_control_sensors_enabled & (0x01 << sensor);
}

bool MAVLinkConnection::isSensorOperational(Sensor const sensor) const {
    return sys_status.onboard_control_sensors_health & (0x01 << sensor);
}

int MAVLinkConnection::batteryRemaining() const {
    return sys_status.battery_remaining;
}

MAVLinkConnection::LocalPosition MAVLinkConnection::localPosition() const {
    return { static_cast<float>(local_position.time_boot_ms),
             local_position.x, local_position.y, local_position.z,
             local_position.vx, local_position.vy, local_position.vz };
}

MAVLinkConnection::GlobalPosition MAVLinkConnection::globalPosition() const {
    return { static_cast<float>(global_position.time_boot_ms),
             global_position.lat * 1e-7f,
             global_position.lon * 1e-7f,
             global_position.alt * 1e-3f,
             global_position.relative_alt * 1e-3f,
             global_position.vx * 1e-3f,
             global_position.vy * 1e-3f,
             global_position.vz * 1e-3f,
             static_cast<float>(global_position.hdg) };
}

QVector<MAVLinkConnection::Satellite> MAVLinkConnection::satellites() const {
    QVector<Satellite> result(gps_status.satellites_visible);
    for(int i = 0; i < gps_status.satellites_visible; ++i) {
        result[i].id = gps_status.satellite_prn[i];
        result[i].isUsed = gps_status.satellite_used[i];
        result[i].elevation = gps_status.satellite_elevation[i];
        result[i].azimuth = (gps_status.satellite_azimuth[i]/255.0f)*360;
        result[i].snr = gps_status.satellite_snr[i];
    }
    return result;
}


MAVLinkConnection::VFRHUD MAVLinkConnection::vfrHUD() const {
    return { vfr_hud.airspeed,
             vfr_hud.groundspeed,
             static_cast<float>(vfr_hud.heading),
             static_cast<float>(vfr_hud.throttle),
             vfr_hud.alt,
             vfr_hud.climb };
}

void MAVLinkConnection::connectDisconnect(bool shouldConnect) {
    if(shouldConnect)
        connectToDevice();
    else
        disconnectFromDevice();
}

void MAVLinkConnection::connectToDevice() {
    disconnectFromDevice();
    QSerialPortInfo serialPortInfo;
    bool foundPort = false;
    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
		qDebug() << "Name        : " << info.portName();
        qDebug() << "Description : " << info.description();
        qDebug() << "Manufacturer: " << info.manufacturer();
        if(info.portName().contains("COM6", Qt::CaseInsensitive) ||
           info.portName().contains("ttyUSB0", Qt::CaseInsensitive) ||
           info.manufacturer().contains("FTDI", Qt::CaseInsensitive)) {
            serialPortInfo = info;
            foundPort = true;
            break;
        }
    }

    if(!foundPort)
        //emit connectionError("Could not find port");
        qDebug() << "Could not find port";

    port.setPort(serialPortInfo);

    if(!port.open(QIODevice::ReadWrite))
    {
        //emit connectionError("Could not open port " + port.portName());
        qDebug() << "Could not open port " + port.portName();
        qDebug() << port.errorString();
    }
	port.setBaudRate(baudRate);
    connect(&port, SIGNAL(readyRead()), this, SLOT(readData()));
}

void MAVLinkConnection::disconnectFromDevice() {
    if(port.isOpen()) {
		streamOff();
        port.close();
    }
}

void MAVLinkConnection::readData() {
    QByteArray data = port.readAll();
    // cout << data.data();
    mavlink_message_t msg;
    mavlink_status_t status;
    int const size = data.size();
    for(int i = 0; i < size; ++i) {
        if(mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
            cout.flush();
        target_system = msg.sysid;
        target_component = msg.compid;
		//qDebug() << "SYS:" << msg.sysid << "COMP:" << msg.compid << "LEN:" << msg.len << "MSG ID:" << msg.msgid;
        switch(msg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT: {
				//qDebug() << "Autopilot: " << (int)heartbeat.autopilot;
				//qDebug() << "Mavlink Version: " << (int)heartbeat.mavlink_version;
				//qDebug() << "Frame Type: " << (int)heartbeat.type << endl;
				emit heartbeatReceived();
				lastHeartbeat = MAVLinkConnection::SystemTimeMillisecs();
                uint8_t const system_status_previous = heartbeat.system_status;
				uint8_t const base_mode_previous = heartbeat.base_mode;
				uint32_t const custom_mode_previous = heartbeat.custom_mode;
                mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                if(heartbeat.system_status != system_status_previous)
                    cout << "System state changed to: " << (int)heartbeat.system_status << endl;
				if (heartbeat.base_mode != base_mode_previous) {
					//qDebug() << "Base Mode changed to: " << (int)heartbeat.base_mode << endl;
					base_mode = heartbeat.base_mode;
				}
				if (heartbeat.custom_mode != custom_mode_previous) {
					//qDebug() << "Custom Mode changed to: " << (int)heartbeat.custom_mode << endl;
					custom_mode = heartbeat.custom_mode;
					switch (custom_mode) {
						case 0:
							emit flightMode("FlightMode: Stabilize");
							break;
						case 2:
							emit flightMode("FlightMode: AltHold");
							break;
						case 3:
							emit flightMode("FlightMode: Auto");
							break;
						case 4:
							emit flightMode("FlightMode: Guided");
							break;
						case 5:
							emit flightMode("FlightMode: Loiter");
							break;
						case 6:
							emit flightMode("FlightMode: RTL");
							break;
						case 9:
							emit flightMode("FlightMode: Land");
							break;
						default:
							break;

						}
				}

				bool currentlyArmed = heartbeat.base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY;

				if (systemIsArmed != currentlyArmed)
				{
					systemIsArmed = currentlyArmed;

					if (systemIsArmed)
					{
						emit armed("ARMED");
					}
					else
					{
						emit disarmed("DISARMED");
					}
				}
                break;
            }
			case MAVLINK_MSG_ID_SYSTEM_TIME:
				mavlink_msg_system_time_decode(&msg, &system_time);
				break;
            case MAVLINK_MSG_ID_SYS_STATUS:
                mavlink_msg_sys_status_decode(&msg, &sys_status);
                emit systemStatusChanged();
                break;
            case MAVLINK_MSG_ID_STATUSTEXT:
                mavlink_msg_statustext_decode(&msg, &statustext);
                emit statusChanged(statustext.text);
                break;
            case MAVLINK_MSG_ID_ATTITUDE:
                mavlink_msg_attitude_decode(&msg, &attitude);
                cout << "Got attitude" << endl;
                break;
            case MAVLINK_MSG_ID_GPS_STATUS:
                mavlink_msg_gps_status_decode(&msg, &gps_status);        
                cout << "Got gps status" << endl;
                break;
            case MAVLINK_MSG_ID_GPS_RAW_INT:
                mavlink_msg_gps_raw_int_decode(&msg, &gps_data);
                cout << "Got gps data" << endl;
                break;
            case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                mavlink_msg_local_position_ned_decode(&msg, &local_position);
                qDebug() << "Got local position" << endl;
                emit localPositionChanged();
                break;
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                mavlink_msg_global_position_int_decode(&msg, &global_position);
                emit globalPositionChanged();
                break;
            case MAVLINK_MSG_ID_RAW_IMU:
                mavlink_msg_raw_imu_decode(&msg, &imu);
                cout << "Got imu" << endl;
                break;
            case MAVLINK_MSG_ID_RAW_PRESSURE:
                mavlink_msg_raw_pressure_decode(&msg, &pressure);
                cout << "Got pressure" << endl;
                break;
            case MAVLINK_MSG_ID_VFR_HUD:
                mavlink_msg_vfr_hud_decode(&msg, &vfr_hud);
                emit VFRHUDChanged();
                break;
            case MAVLINK_MSG_ID_COMMAND_ACK:
                mavlink_msg_command_ack_decode(&msg, &command_ack);
                switch(command_ack.result) {
                    case MAV_RESULT_ACCEPTED:
                        emit commandAcknowledged("Command ACCEPTED and EXECUTED");
                        break;
                    case MAV_RESULT_TEMPORARILY_REJECTED:
                        emit commandAcknowledged("Command TEMPORARY REJECTED/DENIED");
                        break;
                    case MAV_RESULT_DENIED:
                        emit commandAcknowledged("Command PERMANENTLY DENIED");
                        break;
                    case MAV_RESULT_UNSUPPORTED:
                        emit commandAcknowledged("Command UNKNOWN/UNSUPPORTED");
                        break;
                    case MAV_RESULT_FAILED:
                        emit commandAcknowledged("Command executed, but failed");
                        break;
                    case MAV_RESULT_ENUM_END:
                        break;
                };
                break;
			case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
				uint16_t const ch7_previous = rc_channels_raw.chan7_raw;
				uint16_t const ch8_previous = rc_channels_raw.chan8_raw;
				mavlink_msg_rc_channels_raw_decode(&msg, &rc_channels_raw);
				//qDebug() << "Throttle: " << (int)rc_channels_raw.chan3_raw;
				//qDebug() << "Roll: " << (int)rc_channels_raw.chan1_raw;
				//qDebug() << "Pitch: " << (int)rc_channels_raw.chan2_raw;
				if (rc_channels_raw.chan6_raw > 1500)
				{
					emit ch6High();
				}

				if ((rc_channels_raw.chan7_raw - ch7_previous) > 500)
				{
					//qDebug() << "Ch7 Toggle on";
					emit ch7ToggleOn();
				}
				if ((rc_channels_raw.chan7_raw - ch7_previous) < -500)
				{
					//qDebug() << "Ch7 Toggle off";
					emit ch7ToggleOff();
				}

				if ((rc_channels_raw.chan8_raw - ch8_previous) > 500)
				{
					//qDebug() << "Ch8 Toggle on";
					emit ch8ToggleOn();
				}
				if ((rc_channels_raw.chan8_raw - ch8_previous) < -500)
				{
					//qDebug() << "Ch8 Toggle off";
					emit ch8ToggleOff();
				}
				break;
			}
			case MAVLINK_MSG_ID_AUTOPILOT_VERSION: 
					mavlink_msg_autopilot_version_decode(&msg, &autopilot_version);
					qDebug() << "Capabilities: " << (int)autopilot_version.capabilities << endl;
					break;
            default:
                printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
                break;
          }
        }
    }
}

void MAVLinkConnection::streamAll() {
    mavlink_message_t msg;
	int stream_rate = 1;
	mavlink_msg_request_data_stream_pack(system.sysid, system.compid, &msg, target_system, target_component, MAV_DATA_STREAM_ALL, stream_rate, 1);
	sendMessage(msg);
}

void MAVLinkConnection::streamRawSensors() {
	mavlink_message_t msg;
	int stream_rate = 1;
	mavlink_msg_request_data_stream_pack(system.sysid, system.compid, &msg, target_system, target_component, MAV_DATA_STREAM_RAW_SENSORS, stream_rate, 1);
	sendMessage(msg);
}

void MAVLinkConnection::streamRcChannels() {
	mavlink_message_t msg;
	int stream_rate = 10;
	mavlink_msg_request_data_stream_pack(system.sysid, system.compid, &msg, target_system, target_component, MAV_DATA_STREAM_RC_CHANNELS, stream_rate, 1);
	sendMessage(msg);
}

void MAVLinkConnection::streamOff() {
	mavlink_message_t msg;
	mavlink_msg_request_data_stream_pack(system.sysid, system.compid, &msg, target_system, target_component, MAV_DATA_STREAM_ALL, 0, 0);
	sendMessage(msg);
}

void MAVLinkConnection::navigateToWaypoint(float time,
                                           float acceptanceRadius,
                                           float trajectoryControl,
                                           float yaw,
                                           float latitude,
                                           float longitude,
                                           float altitude) {
    sendCommand(MAV_CMD_NAV_WAYPOINT, time, acceptanceRadius, trajectoryControl, yaw, latitude, longitude, altitude);
}

void MAVLinkConnection::setROI(ROI roi,
                               int missionIndex,
                               int targetID,
                               int roiIndex,
                               float x,
                               float y,
                               float z) {
    sendCommand(MAV_CMD_NAV_ROI, roi,
                (roi == ROI_GIVEN_MISSION_INDEX? missionIndex
                 : (roi == ROI_TARGET? targetID : 0)),
                roiIndex, 0, x, y, z);

}

void MAVLinkConnection::sendCommand(MAV_CMD command) {
	mavlink_message_t msg;
	mavlink_command_long_t cmd;
	cmd.command = (uint16_t)command;
	cmd.confirmation = 0;
	cmd.param1 = 0.0f;
	cmd.param2 = 0.0f;
	cmd.param3 = 0.0f;
	cmd.param4 = 0.0f;
	cmd.param5 = 0.0f;
	cmd.param6 = 0.0f;
	cmd.param7 = 0.0f;
	cmd.target_system = target_system;
	cmd.target_component = target_component;
	mavlink_msg_command_long_encode(system.sysid, system.compid, &msg, &cmd);
	sendMessage(msg);
}

void MAVLinkConnection::sendCommand(MAV_CMD command, float param1, float param2, float param3, float param4, float param5, float param6, float param7, int confirmation, int component)
{
	mavlink_message_t msg;
	mavlink_command_long_t cmd;
	cmd.command = (uint16_t)command;
	cmd.confirmation = confirmation;
	cmd.param1 = param1;
	cmd.param2 = param2;
	cmd.param3 = param3;
	cmd.param4 = param4;
	cmd.param5 = param5;
	cmd.param6 = param6;
	cmd.param7 = param7;
	cmd.target_system = target_system;
	cmd.target_component = component;
	mavlink_msg_command_long_encode(system.sysid, system.compid, &msg, &cmd);
	sendMessage(msg);
}

void MAVLinkConnection::sendCommand(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(system.sysid, system.compid, &msg, target_system, target_component, command, 0,
                                  param1, param2, param3, param4, param5, param6, param7);
	sendMessage(msg);
}

void MAVLinkConnection::sendMessage(mavlink_message_t message) {
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;
	len = mavlink_msg_to_send_buffer(buf, &message);
	port.write(reinterpret_cast<const char *>(buf), len);
}

void MAVLinkConnection::sendHeartBeat() {
	//qDebug() << "Sending Heartbeat";
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(system.sysid, system.compid, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0);
	sendMessage(msg);
}

void MAVLinkConnection::arm()
{
	sendCommand(MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0, 0, MAV_COMP_ID_SYSTEM_CONTROL);
	/*mavlink_message_t msg;
	mavlink_msg_command_long_pack(system.sysid, MAV_COMP_ID_SYSTEM_CONTROL, &msg, target_system, target_component, MAV_CMD_COMPONENT_ARM_DISARM, 0,1, 0, 0, 0, 0, 0, 0);
	sendMessage(msg);*/
}

void MAVLinkConnection::disarm()
{
	sendCommand(MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0, MAV_COMP_ID_SYSTEM_CONTROL);
	/*mavlink_message_t msg;
	mavlink_msg_command_long_pack(system.sysid, MAV_COMP_ID_SYSTEM_CONTROL, &msg, target_system, target_component, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0);
	sendMessage(msg);*/
}

void MAVLinkConnection::setMode(uint8_t base_mode, uint32_t custom_mode) {
	mavlink_message_t msg;
	mavlink_msg_set_mode_pack(system.sysid, system.compid, &msg, target_system, base_mode, custom_mode);
	sendMessage(msg);
}

void MAVLinkConnection::setModeApmStabilize(){
	setMode(base_mode, 0);
}

void MAVLinkConnection::setModeApmAltHold(){
	setMode(base_mode, 2);
}

void MAVLinkConnection::setModeApmAuto(){
	setMode(base_mode, 3);
}

void MAVLinkConnection::setModeApmGuided(){
	setMode(base_mode, 4);
}

void MAVLinkConnection::setModeApmLoiter(){
	setMode(base_mode, 5);
}

void MAVLinkConnection::setModeApmRtl(){
	setMode(base_mode, 6);
}

void MAVLinkConnection::setModeApmLand(){
	setMode(base_mode, 9);
}

void MAVLinkConnection::RcChannelsOverride(int ch1, int ch2, int ch3, int ch4, int ch5, int ch6, int ch7, int ch8)
{		
	mavlink_rc_channels_override_t rc_override; //Msg to override rc channels
	mavlink_message_t msg_rc;
	rc_override.chan1_raw = ch1;
	rc_override.chan2_raw = ch2;
	rc_override.chan3_raw = ch3; 
	rc_override.chan4_raw = ch4;
	rc_override.chan5_raw = ch5;
	rc_override.chan6_raw = ch6;
	rc_override.chan7_raw = ch7;
	rc_override.chan8_raw = ch8;
	rc_override.target_system = target_system; 
	rc_override.target_component = target_component;
	mavlink_msg_rc_channels_override_encode(255, system.compid, &msg_rc, &rc_override); //system.sysid = 255 for Misson Planner
	sendMessage(msg_rc);
}

void MAVLinkConnection::sendThrottle(int val){
	//qDebug() << "Sending Throttle" << val;
	RcChannelsOverride(-1, -1, val, -1, -1, -1, -1, -1);
}

void MAVLinkConnection::sendPitch(int val){
	//qDebug() << "Sending Pitch" << val;
	RcChannelsOverride(-1, val, -1, -1, -1, -1, -1, -1);
}

void MAVLinkConnection::sendRoll(int val){
	//qDebug() << "Sending Roll" << val;
	RcChannelsOverride(val, -1, -1, -1, -1, -1, -1, -1);
}

void MAVLinkConnection::sendYaw(int val){
	//qDebug() << "Sending Yaw" << val;
	RcChannelsOverride(-1, -1, -1, val, -1, -1, -1, -1);
}

void MAVLinkConnection::sendRc(int roll, int pitch, int throttle, int yaw){
	RcChannelsOverride(roll, pitch, throttle, yaw, -1, -1, -1, -1);
}

void MAVLinkConnection::sendRc(int roll, int pitch, int throttle, int yaw, int ch5, int ch6, int ch7, int ch8){
	//qDebug() << "Sending Roll" << val << endl;
	RcChannelsOverride(roll, pitch, throttle, yaw, ch5, ch6, ch7, ch8);
}

void MAVLinkConnection::sendRcReset(){
	qDebug() << "Sending RcReset";
	RcChannelsOverride(0,0,0,0,0,0,0,0);
}

void MAVLinkConnection::RequestAutoPilotCapabilities()
{
	sendCommand(MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 1, 0, 0, 0, 0, 0, 0);
}


//void MAVLinkConnection::TargetControlCommands()
//{
//	double x = 10;
//	double y = 10;
//	double z = 0;
//	double roll = 0.5;
//	double pitch = 0.5;
//	double yaw = 0;
//	//sendCommand(MAV_CMD_NAV_GUIDED_ENABLE, 1, 0, 0, 0, 0, 0, 0);
//	setTargetControlCommands(x, y, z, roll, pitch, yaw);
//}
//
//void MAVLinkConnection::setTargetControlCommands(double x, double y, double z, double roll, double pitch, double yaw)
//{
//	/*if (base_mode & MAV_MODE_FLAG_DECODE_POSITION_MANUAL)
//		qDebug() << "Manual Mode";
//	if (base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY)
//		qDebug() << "Armed";
//	if (base_mode & MAV_MODE_FLAG_DECODE_POSITION_GUIDED)
//		qDebug() << "Guided";*/
//
//	// If system has manual inputs enabled and is armed
//	if (((base_mode & MAV_MODE_FLAG_DECODE_POSITION_MANUAL) && (base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY)) || (base_mode & MAV_MODE_FLAG_HIL_ENABLED))
//	{
//		mavlink_message_t message;
//		float q[4];
//		mavlink_euler_to_quaternion(roll, pitch, yaw, q);
//		float yawrate = 0.0f;
//
//		// Do not control rates and throttle
//		quint8 mask = (1 << 0) | (1 << 1) | (1 << 2); // ignore rates
//		mask |= (1 << 6); // ignore throttle
//		mavlink_msg_set_attitude_target_pack(system.sysid, system.compid,
//			&message, MAVLinkConnection::SystemTimeMillisecs(), target_system, target_component,
//			mask, q, 0, 0, 0, 0);
//		sendMessage(message);
//
//		quint16 position_mask = (1 << 3) | (1 << 4) | (1 << 5) |
//			(1 << 6) | (1 << 7) | (1 << 8);
//		mavlink_msg_set_position_target_local_ned_pack(system.sysid, system.compid,
//			&message, MAVLinkConnection::SystemTimeMillisecs(), target_system, target_component,
//			MAV_FRAME_LOCAL_NED, position_mask, x, y, z, 0, 0, 0, 0, 0, 0, yaw, yawrate);
//		sendMessage(message);
//		qDebug() << __FILE__ << __LINE__ << ": SENT TARGET CONTROL MESSAGES: x" << x << " y: " << y << " z: " << z << " roll: " << roll << " pitch: " << pitch << " yaw: " << yaw;
//
//		//emit attitudeThrustSetPointChanged(this, roll, pitch, yaw, thrust, MAVLinkConnection::SystemTimeMillisecs());
//	}
//	else
//	{
//		qDebug() << "TARGET CONTROL: IGNORING COMMANDS: Set mode to MANUAL to send TARGET commands first";
//	}
//}
//
///**
//* Set the manual control commands.
//* This can only be done if the system has manual inputs enabled and is armed.
//*/
//void MAVLinkConnection::setManualControlCommands(float roll, float pitch, float yaw, float thrust)
//{
//
//	//Store the previous manual commands
//	static float manualRollAngle = 0.0;
//	static float manualPitchAngle = 0.0;
//	static float manualYawAngle = 0.0;
//	static float manualThrust = 0.0;
//	static quint16 manualButtons = 0;
//	static quint8 countSinceLastTransmission = 0; // Track how many calls to this function have occurred since the last MAVLink transmission
//
//	//We only transmit manual command messages if the system has manual inputs enabled and is armed
//	if (((base_mode & MAV_MODE_FLAG_DECODE_POSITION_MANUAL) && (base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY)) || (base_mode & MAV_MODE_FLAG_HIL_ENABLED))
//	{
//		qDebug() << "Manual Control";
//
//		//		Transmit the manual commands only if they've changed OR if it's been a little bit since they were last transmit.To make sure there aren't issues with
//		//			response rate, we make sure that a message is transmit when the commands have changed, then one more time, and then switch to the lower transmission rate
//		//			if no command inputs have changed.
//		//				The default transmission rate is 50Hz, but when no inputs have changed it drops down to 5Hz.
//		bool sendCommand = false;
//		if (countSinceLastTransmission++ >= 10)
//		{
//			sendCommand = true;
//			countSinceLastTransmission = 0;
//		}
//		else if ((!isnan(roll) && roll != manualRollAngle) || (!isnan(pitch) && pitch != manualPitchAngle) ||
//			(!isnan(yaw) && yaw != manualYawAngle) || (!isnan(thrust) && thrust != manualThrust))
//		{
//			sendCommand = true;
//
//			//			Ensure that another message will be sent the next time this function is called
//			countSinceLastTransmission = 10;
//		}
//
//		//		Now if we should trigger an update, let's do that
//		if (sendCommand)
//		{
//			//				Save the new manual control inputs
//			manualRollAngle = roll;
//			manualPitchAngle = pitch;
//			manualYawAngle = yaw;
//			manualThrust = thrust;
//
//			//				Store scaling values for all 3 axes
//			const float axesScaling = 1.0 * 1000.0;
//
//			//				Calculate the new commands for roll, pitch, yaw, and thrust
//			const float newRollCommand = roll * axesScaling;
//			const float newPitchCommand = pitch * axesScaling;
//			const float newYawCommand = yaw * axesScaling;
//			const float newThrustCommand = thrust * axesScaling;
//
//			//				Send the MANUAL_COMMAND message
//			mavlink_message_t message;
//			mavlink_msg_manual_control_pack(system.sysid, system.compid, &message, target_system, newPitchCommand, newRollCommand, newThrustCommand, newYawCommand, 0);
//			sendMessage(message);
//
//			//				Emit an update in control values to other UI elements, like the HSI display
//			//					emit attitudeThrustSetPointChanged(this, roll, pitch, yaw, thrust, QGC::groundTimeMilliseconds());
//		}
//	}
//}
//
//void MAVLinkConnection::setMode(uint8_t base_mode, uint32_t custom_mode) {
//	sendCommand(MAV_CMD_DO_SET_MODE, base_mode, custom_mode, 0, 0, 0, 0, 0,0,255);
//}

//void MAVLinkConnection::goAutonomous()
//{
//	setMode((base_mode & ~(MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)) | (MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED), 0);
//	qDebug() << __FILE__ << __LINE__ << "Going autonomous";
//}
//
//void MAVLinkConnection::goManual()
//{
//	setMode((base_mode & ~(MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED)) | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 0);
//	qDebug() << __FILE__ << __LINE__ << "Going manual";
//}
//
//void MAVLinkConnection::toggleAutonomy()
//{
//	setMode(base_mode ^ MAV_MODE_FLAG_AUTO_ENABLED ^ MAV_MODE_FLAG_MANUAL_INPUT_ENABLED ^ MAV_MODE_FLAG_GUIDED_ENABLED ^ MAV_MODE_FLAG_STABILIZE_ENABLED, 0);
//	qDebug() << __FILE__ << __LINE__ << "Toggling autonomy";
//}
//


//NOT WORKING
//void MAVLinkConnection::armSystem()
//{
//	setMode(base_mode | MAV_MODE_FLAG_SAFETY_ARMED, custom_mode);
//}

///**
//* @warning Depending on the UAS, this might completely stop all motors.
//*
//*/
//void MAVLinkConnection::disarmSystem()
//{
//	setModeArm(base_mode & ~(MAV_MODE_FLAG_SAFETY_ARMED), custom_mode);
//}
//
//void MAVLinkConnection::toggleArmedState()
//{
//	setModeArm(base_mode ^ (MAV_MODE_FLAG_SAFETY_ARMED), custom_mode);
//}

///**
//* @param newBaseMode that UAS is to be set to.
//* @param newCustomMode that UAS is to be set to.
//*/
//void MAVLinkConnection::setMode(uint8_t newBaseMode, uint32_t newCustomMode)
//{
//		// Strip armed / disarmed call for safety reasons, this is not relevant for setting the mode
//		newBaseMode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
//
//		setModeArm(newBaseMode, newCustomMode);
//}
//
///**
//* @param newBaseMode that UAS is to be set to.
//* @param newCustomMode that UAS is to be set to.
//*/
//void MAVLinkConnection::setModeArm(uint8_t newBaseMode, uint32_t newCustomMode)
//{
//		mavlink_message_t msg;
//		mavlink_msg_set_mode_pack(system.sysid, system.compid, &msg, target_system, newBaseMode, newCustomMode);
//		sendMessage(msg);
//		qDebug() << "SENDING REQUEST TO SET MODE TO SYSTEM" << ", MODE " << newBaseMode << "-" << newCustomMode;
//}
