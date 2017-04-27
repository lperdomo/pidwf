#include <iostream>
#include "Aria.h"

using namespace std;

double timeInterval = 0.5;
//double kProb = 0.6, kDer = 0.03, kInt = 0.0002;
//double kProb = 0.8, kDer = 0.5, kInt = 0;
double kProb = 1, kDer = 0.5, kInt = 0;
//double kProb = 0.8, kDer = 0.3, kInt = 0;


bool checkWall(ArRangeDevice &sensor, double startAngle, double endAngle, double target)
{
	if (sensor.currentReadingPolar(startAngle, endAngle) < target*1.5) {
		return true;
	}
	return false;
}

double calcPID(ArRangeDevice &sensor, double &integral, double &prevErr, double startAngle, double endAngle, double target)
{
	double err = target - sensor.currentReadingPolar(startAngle, endAngle);
	integral += err*timeInterval;
	double derivative = (err-prevErr)/timeInterval;
	prevErr = err;
	return kProb*err + kInt*integral + kDer*derivative;
}

void followWall(ArRobot &robot, double pid)
{
	double e1 = 0, e2 = 0;
    double v1 = 300, v2 = 300;

    if (pid > 0) {
        e1 = pid;
    } else {
        e2 = -pid;
    }
    if (pid > v1) {
        e1 = v1;
    } else if (pid < -v1) {
        e2 = v2;
    }
    cout << "v1 " << v1 << " v1-e1 " << v1-e1;
    cout << " v2 " << v2 << " v2-e2 " << v2-e2 << endl;
    robot.setVel2(v1-e1,v2-e2);
}

int main(int argc, char** argv)
{
	Aria::init();
	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();
	ArRobot robot;
	ArAnalogGyro gyro(&robot);
	ArSonarDevice sonar;
	ArSick sick;
	ArRobotConnector robotConnector(&parser, &robot);
	ArLaserConnector laserConnector(&parser, &robot, &robotConnector);

	if(!robotConnector.connectRobot()) {
		ArLog::log(ArLog::Terse, "Failed to connect with Pioneer");
		if(parser.checkHelpAndWarnUnparsed()) {
			Aria::logOptions();
			Aria::exit(1);
		}
	}
	ArLog::log(ArLog::Normal, "Connected with Pioneer");
	robot.addRangeDevice(&sonar);
	robot.addRangeDevice(&sick);
	robot.runAsync(true);

	sick.runAsync();
	laserConnector.setupLaser(&sick);
	if(!laserConnector.connectLaser(&sick)) {
		ArLog::log(ArLog::Terse, "Failed to connect with Sick");
		if(parser.checkHelpAndWarnUnparsed()) {
			Aria::logOptions();
			Aria::exit(1);
		}
	}

	int sonars[8];
	vector<ArSensorReading>* sickReadings;

    double leftIntegral = 0, leftPrevError = 0, rightIntegral = 0, rightPrevError = 0;
	double frontStartAngle = -30, frontFinishAngle = 30, frontDesire = 1000;

	robot.enableMotors();
	while (Aria::getRunning()) {
		robot.lock();

		//for (int i=0;i<8;i++) sonars[i] = robot.getSonarRange(i);

		//sick.lockDevice();
		//sickReadings = sick.getRawReadingsAsVector();
		//sick.unlockDevice();


		if (checkWall(sick, 30, 60, 1000)) {
			followWall(robot, calcPID(sick, leftIntegral, leftPrevError, 30, 60, 1000));
		}
		if (checkWall(sick, -30, -60, 1000)) {
			if (sick.currentReadingPolar(-30, -60) < sick.currentReadingPolar(30, 60)) {
				followWall(robot, calcPID(sick, rightIntegral, rightPrevError, -30, -60, 1000));
			}
		}

		robot.unlock();
		ArUtil::sleep(1000*timeInterval);
	}

	Aria::exit(0);
	return 0;
}
