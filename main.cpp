#include <iostream>
#include "Aria.h"

using namespace std;

double timeInterval = 0.5;

double calcPID(double closestReading, double target, double &integral, double &prevError)
{
	double kProb = 1, kDer = 0.8, kInt = 0.002;
	//double kProb = 2, kDer = 1.8, kInt = 0.5;
	double err = target - closestReading;
	integral += err*timeInterval;
	double derivative = (err-prevError)/timeInterval;
	prevError = err;
	return kProb*err + kInt*integral + kDer*derivative;
}

void followWall(ArRobot &robot, double pid)
{
	double e1 = 0, e2 = 0, v1 = 200, v2 = 200;

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
    robot.setVel2(v1-e1, v2-e2);
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

	bool following, leftFollowing = false, rightFollowing = false
	, frontWall = false, leftWall = false, rightWall = false;
    double leftIntegral = 0, leftPrevError = 0, leftStartAngle = 30, leftEndAngle = 60
    , rightIntegral = 0, rightPrevError = 0, rightStartAngle = -30, rightEndAngle = -60
    , frontStartAngle = -30, frontEndAngle = 30
	, target = 1000, range = 1500;

	robot.enableMotors();
	while (Aria::getRunning()) {
		robot.lock();
		frontWall = (sick.currentReadingPolar(frontStartAngle, frontEndAngle) < range);
		leftWall = (sick.currentReadingPolar(leftStartAngle, leftEndAngle) < range);
		rightWall = (sick.currentReadingPolar(rightStartAngle, rightEndAngle) < range);
		following = false;
		if (frontWall) {
			robot.setVel2(0, 0);
			if (rightWall) {//turning left
				robot.setDeltaHeading(45);
			} else {//default or left wall, turn right
				robot.setDeltaHeading(-45);
			}
		}

		if (leftWall) {//sticking to the wall
			following = true;
			leftFollowing = true;
			rightFollowing = false;
			followWall(robot, calcPID(sick.currentReadingPolar(leftStartAngle, leftEndAngle), target, leftIntegral, leftPrevError));
		}
		if (rightWall) {
			following = true;
			rightFollowing = true;
			leftFollowing = false;
			followWall(robot, calcPID(sick.currentReadingPolar(rightStartAngle, rightEndAngle), target, rightIntegral, rightPrevError));
		}

		if (following == false) {
			if (leftFollowing) {//small turn, just checking if it was a sharp wall edge
				robot.setDeltaHeading(45);
				leftFollowing = false;
			} else if (rightFollowing) {
				robot.setDeltaHeading(-45);
				rightFollowing = false;
			} else {//wander
				robot.setVel2(300, 300);
			}
		}

		robot.unlock();
		ArUtil::sleep(1000*timeInterval);
	}

	Aria::exit(0);
	return 0;
}
