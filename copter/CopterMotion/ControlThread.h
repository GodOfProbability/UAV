#ifndef CONTROLTHREAD_H
#define CONTROLTHREAD_H

#include <QMutex>
#include <QThread>
#include <QWaitCondition>

#include "MAVLinkConnection.h"
#include "MainWindow.h"


class ControlThread : public QThread
{
	Q_OBJECT

public:
	explicit ControlThread(MAVLinkConnection* connection);
	~ControlThread();

	int rollTargetLeft;
	int	rollTargetRight;
	int	pitchTargetFront;
	int	pitchTargetBack;
	int rampIncrement;
	int transmissionDelay;
	int rampDelay;

public slots:
	void StartControl();
	void StopControl();

	void MoveLeft();
	void MoveRight();
	void MoveFront();
	void MoveBack();
	void MoveHold();
	void Halt();
	void setRcReset();

protected:
	void run() Q_DECL_OVERRIDE;
	void stop();

private:
	QMutex mutex;
	QWaitCondition condition;
	bool bRestart;
	bool bAbort;
	bool bMoveLeft;
	bool bMoveRight;
	bool bMoveFront;
	bool bMoveBack;
	bool bMoveHold;
	
	MAVLinkConnection* _connection;

	int rollVal;
	int pitchVal;

	void setPitch(int pitchTarget);
	void setRoll(int rollTarget);
	void setRollPitch(int rollTarget, int pitchTarget);

	void rampPitch(int pitchTarget);
	void rampRoll(int rollTarget);
	void rampRollPitch(int rollTarget, int pitchTarget);
};

#endif // CONTROLTHREAD_H