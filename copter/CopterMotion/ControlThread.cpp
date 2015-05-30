#include <QtCore>
#include "ControlThread.h"
#include "MainWindow.h"

//Constructor
ControlThread::ControlThread(MAVLinkConnection* connection)
{
	bRestart = false;
	bAbort = false;
	rollVal = 1500;
	pitchVal = 1500;
	rollTargetLeft = 1350;
	rollTargetRight = 1650;
	pitchTargetFront = 1350;
	pitchTargetBack = 1650;
	rampIncrement = 10;
	transmissionDelay = 20;
	rampDelay = 20;
	_connection = connection;
}

//Destructor
ControlThread::~ControlThread()
{
	mutex.lock();
	bAbort = true;
	condition.wakeOne();
	mutex.unlock();
	wait();
}

void ControlThread::StartControl()
{
	QMutexLocker locker(&mutex);

	if (!isRunning()) {
		start(HighPriority);
	}
	else {
		bRestart = true;
		condition.wakeOne();
	}
}

void ControlThread::StopControl()
{
	stop();
	bAbort = false;
}


void ControlThread::run()
{	
	forever{

		qDebug() << "ControlThread Running";

		//Initialize
		int rollTarget = 1500;
		int pitchTarget = 1500;

		forever{
			//Check exit conditions
			if (bRestart) break;
			if (bAbort) return;

			//Check motion conditions
			if (bMoveLeft) rollTarget = rollTargetLeft;
			if (bMoveRight) rollTarget = rollTargetRight;
			if (bMoveFront) pitchTarget = pitchTargetFront;
			if (bMoveBack) pitchTarget = pitchTargetBack;
			
			if (bMoveHold) {
				rollTarget = 1500;
				pitchTarget = 1500;
			}
			
			if (bMoveLeft | bMoveRight | bMoveFront | bMoveBack | bMoveHold) {
				mutex.lock();
				bMoveLeft = false, bMoveRight = false, bMoveFront = false, bMoveBack = false, bMoveHold = false;
				mutex.unlock();

				rampRollPitch(rollTarget, pitchTarget);
			}
		}

		mutex.lock();
		if (!bRestart)
			condition.wait(&mutex);
		bRestart = false;
		mutex.unlock();
	}

}

void ControlThread::stop()
{
	//Make sure the copter holds position before exiting the control thread
	MoveHold();
	//Give control back to TX
	setRcReset();
	//Quit thread
	mutex.lock();
	bAbort = true;
	condition.wakeOne();
	mutex.unlock();
	wait();
}

void ControlThread::MoveLeft(){
	qDebug() << "Moving Left";
	rampRoll(rollTargetLeft);
	//mutex.lock();
	//bMoveLeft = true;
	//mutex.unlock();
}

void ControlThread::MoveRight(){
	qDebug() << "Moving Right";
	rampRoll(rollTargetRight);
	//mutex.lock();
	//bMoveRight = true;
	//mutex.unlock();
}

void ControlThread::MoveFront(){
	qDebug() << "Moving Forward";
	rampPitch(pitchTargetFront);
	//mutex.lock();
	//bMoveFront = true;
	//mutex.unlock();
}

void ControlThread::MoveBack(){
	qDebug() << "Moving Back";
	rampPitch(pitchTargetBack);
	//mutex.lock();
	//bMoveBack = true;
	//mutex.unlock();
}

void ControlThread::MoveHold(){
	qDebug() << "Holding Position";
	rampRollPitch(1500, 1500);
	//mutex.lock();
	//bMoveHold = true;
	//mutex.unlock();
}

void ControlThread::Halt(){
	qDebug() << "Halt";
	setRollPitch(1500,1500);
}

void ControlThread::rampRollPitch(int rollTarget, int pitchTarget){

	while (rollVal != rollTarget || pitchVal != pitchTarget)
	{
		if (rollVal < rollTarget) rollVal = rollVal + rampIncrement;
		if (rollVal > rollTarget) rollVal = rollVal - rampIncrement;
		if (pitchVal < pitchTarget) pitchVal = pitchVal + rampIncrement;
		if (pitchVal > pitchTarget) pitchVal = pitchVal - rampIncrement;

		_connection->sendRc(rollVal,pitchVal,-1,-1);
		msleep(transmissionDelay);
		_connection->sendRc(rollVal, pitchVal, -1, -1);
		msleep(rampDelay);
		qDebug() << "rollVal: " << rollVal << "rollTarget: " << rollTarget;
		qDebug() << "pitchVal: " << pitchVal << "pitchTarget: " << pitchTarget;
	}

}

void ControlThread::rampRoll(int rollTarget){

	while (rollVal < rollTarget)
	{
		rollVal = rollVal + rampIncrement;
		_connection->sendRoll(rollVal);
		msleep(transmissionDelay);
		_connection->sendRoll(rollVal);
		msleep(rampDelay);
		qDebug() << "rollVal: " << rollVal << "rollTarget: " << rollTarget;
	}

	while (rollVal > rollTarget)
	{
		rollVal = rollVal - rampIncrement;
		_connection->sendRoll(rollVal);
		msleep(transmissionDelay);
		_connection->sendRoll(rollVal);
		msleep(rampDelay);
		qDebug() << "rollVal: " << rollVal << "rollTarget: " << rollTarget;
	}

}

void ControlThread::rampPitch(int pitchTarget){
	
	while (pitchVal < pitchTarget)
	{
		pitchVal = pitchVal + rampIncrement;
		_connection->sendPitch(pitchVal);
		msleep(transmissionDelay);
		_connection->sendPitch(pitchVal);
		msleep(rampDelay);
		qDebug() << "pitchVal: " << pitchVal << "pitchTarget: " << pitchTarget;
	}

	while (pitchVal > pitchTarget)
	{
		pitchVal = pitchVal - rampIncrement;
		_connection->sendPitch(pitchVal);
		msleep(transmissionDelay);
		_connection->sendPitch(pitchVal);
		msleep(rampDelay);
		qDebug() << "pitchVal: " << pitchVal << "pitchTarget: " << pitchTarget;
	}

}

void ControlThread::setRcReset() {
	for (int i=0; i < 5; i++) {
		_connection->sendRcReset();
		msleep(transmissionDelay);
	}
}


void ControlThread::setRollPitch(int roll, int pitch){
	_connection->sendRc(roll, pitch, -1, -1);
	msleep(transmissionDelay);
}

void ControlThread::setRoll(int roll){
	_connection->sendRoll(roll);
	msleep(transmissionDelay);
}

void ControlThread::setPitch(int pitch){
	_connection->sendPitch(pitch);
	msleep(transmissionDelay);
}