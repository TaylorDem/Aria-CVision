#include <Aria.h>
#include <opencv2\opencv.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <videoInput.h>
#include <math.h>
#include <chrono>
#include <thread>
#include "Map_BFS.h"
#include "Command.h"
#include "Voice.h"
#include "Speech.cpp"

using namespace std;
using namespace cv;

struct RobotPosition
{
	double x;
	double z;
	int angle;
};

const unsigned int MAP_UNIT_DIST = 508;
const double FLOOR_HEIGHT = -508;
const double CAMERA_Z_OFFSET = -241.3;
const int MAP_WIDTH = 32;
const int MAP_HEIGHT = 32;
bool commandMode = false;
vector<Point2i> knownLocations;

const Point3i NORMAL_VECTOR(0, 1, 0);
const Point3d START_POINT(0, 0, 0);
const Point3d END_POINT(MAP_UNIT_DIST, 0, 0);

RobotPosition SNOWFLAKE_STATE;
Map *worldMap;

Mat rawWorldMap = Mat::zeros((int)((MAP_HEIGHT * MAP_UNIT_DIST)/10), (int)((MAP_WIDTH * MAP_UNIT_DIST)/10), CV_8UC3);

class ExecuteCommands
{
	enum { FOREWARD = 0, RIGHT = 1, LEFT = 2, BACKWARD = 3 }; //Foreward = North, Right = East, Left = West, Backward = South
	ArRobot *myRobot;
	unsigned int unit_dist;
	unsigned int velocity;
	unsigned int heading_secs = 3000;
	unsigned int sleep_secs;
	unsigned int facing = FOREWARD;

public:
	ExecuteCommands(ArRobot *robot, unsigned int map_unit_dist, unsigned int vel) : myRobot(robot), unit_dist(map_unit_dist), velocity(vel)
	{
		sleep_secs = int ( 1000 * ( ( map_unit_dist ) / ( double( vel ) ) ) );		
	}

	void execute(vector<Command> &commandList)
	{
		for (unsigned int i = 0; i < commandList.size(); i++)
		{
			switch (commandList[i].heading)
			{
			case FOREWARD:
				moveNorth(commandList[i].steps);
				break;
			case RIGHT:
				moveEast(commandList[i].steps);
				break;
			case LEFT:
				moveWest(commandList[i].steps);
				break;
			case BACKWARD:
				moveSouth(commandList[i].steps);
				break;
			default:
				cout << "Invalid heading, skipping command." << endl;
				break;
			}
		}

		return;
	}

	void moveNorth(int mapSteps)
	{
		switch (facing)
		{
		case FOREWARD:
			break;
		case RIGHT:
			myRobot->lock();
			myRobot->setDeltaHeading(-90);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = FOREWARD;
			break;
		case LEFT:
			myRobot->lock();
			myRobot->setDeltaHeading(90);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = FOREWARD;
			break;
		case BACKWARD:
			myRobot->lock();
			myRobot->setDeltaHeading(180);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = FOREWARD;
			break;
		default:
			cout << "Invalid direction." << endl;
			return;
		}

		SNOWFLAKE_STATE.angle = 90;
		SNOWFLAKE_STATE.z += mapSteps*MAP_UNIT_DIST;

		worldMap->setPos(SNOWFLAKE_STATE.z / MAP_UNIT_DIST, SNOWFLAKE_STATE.x / MAP_UNIT_DIST);

		move(mapSteps);

		return;
	}

	void moveNorth(int mapSteps, unsigned int vel)
	{
		switch (facing)
		{
		case FOREWARD:
			break;
		case RIGHT:
			myRobot->lock();
			myRobot->setDeltaHeading(-90);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = FOREWARD;
			break;
		case LEFT:
			myRobot->lock();
			myRobot->setDeltaHeading(90);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = FOREWARD;
			break;
		case BACKWARD:
			myRobot->lock();
			myRobot->setDeltaHeading(180);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = FOREWARD;
			break;
		default:
			cout << "Invalid direction." << endl;
			return;
		}

		SNOWFLAKE_STATE.angle = 90;
		SNOWFLAKE_STATE.z += mapSteps*MAP_UNIT_DIST;

		worldMap->setPos(SNOWFLAKE_STATE.z / MAP_UNIT_DIST, SNOWFLAKE_STATE.x / MAP_UNIT_DIST);

		move(mapSteps, vel);

		return;
	}

	void moveSouth(int mapSteps)
	{
		switch (facing)
		{
		case FOREWARD:
			myRobot->lock();
			myRobot->setDeltaHeading(180);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = BACKWARD;
			break;
		case RIGHT:
			myRobot->lock();
			myRobot->setDeltaHeading(90);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = BACKWARD;
			break;
		case LEFT:
			myRobot->lock();
			myRobot->setDeltaHeading(-90);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = BACKWARD;
			break;
		case BACKWARD:
			break;
		default:
			cout << "Invalid direction." << endl;
			return;
		}

		SNOWFLAKE_STATE.angle = 270;
		SNOWFLAKE_STATE.z -= mapSteps*MAP_UNIT_DIST;

		worldMap->setPos(SNOWFLAKE_STATE.z / MAP_UNIT_DIST, SNOWFLAKE_STATE.x / MAP_UNIT_DIST);

		move(mapSteps);

		return;
	}

	void moveSouth(int mapSteps, unsigned int vel)
	{
		switch (facing)
		{
		case FOREWARD:
			myRobot->lock();
			myRobot->setDeltaHeading(180);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = BACKWARD;
			break;
		case RIGHT:
			myRobot->lock();
			myRobot->setDeltaHeading(90);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = BACKWARD;
			break;
		case LEFT:
			myRobot->lock();
			myRobot->setDeltaHeading(-90);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = BACKWARD;
			break;
		case BACKWARD:
			break;
		default:
			cout << "Invalid direction." << endl;
			return;
		}

		SNOWFLAKE_STATE.angle = 270;
		SNOWFLAKE_STATE.z -= mapSteps*MAP_UNIT_DIST;

		worldMap->setPos(SNOWFLAKE_STATE.z / MAP_UNIT_DIST, SNOWFLAKE_STATE.x / MAP_UNIT_DIST);

		move(mapSteps, vel);

		return;
	}

	void moveWest(int mapSteps)
	{
		switch (facing)
		{
		case FOREWARD:
			myRobot->lock();
			myRobot->setDeltaHeading(-90);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = LEFT;
			break;
		case RIGHT:
			myRobot->lock();
			myRobot->setDeltaHeading(-180);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = LEFT;
			break;
		case LEFT:
			break;
		case BACKWARD:
			myRobot->lock();
			myRobot->setDeltaHeading(90);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = LEFT;
			break;
		default:
			cout << "Invalid direction." << endl;
			return;
		}

		SNOWFLAKE_STATE.angle = 180;
		SNOWFLAKE_STATE.x -= mapSteps*MAP_UNIT_DIST;

		worldMap->setPos(SNOWFLAKE_STATE.z / MAP_UNIT_DIST, SNOWFLAKE_STATE.x / MAP_UNIT_DIST);

		move(mapSteps);

		return;
	}

	void moveWest(int mapSteps, unsigned int vel)
	{
		switch (facing)
		{
		case FOREWARD:
			myRobot->lock();
			myRobot->setDeltaHeading(-90);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = LEFT;
			break;
		case RIGHT:
			myRobot->lock();
			myRobot->setDeltaHeading(-180);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = LEFT;
			break;
		case LEFT:
			break;
		case BACKWARD:
			myRobot->lock();
			myRobot->setDeltaHeading(90);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = LEFT;
			break;
		default:
			cout << "Invalid direction." << endl;
			return;
		}

		SNOWFLAKE_STATE.angle = 180;
		SNOWFLAKE_STATE.x -= mapSteps*MAP_UNIT_DIST;

		worldMap->setPos(SNOWFLAKE_STATE.z / MAP_UNIT_DIST, SNOWFLAKE_STATE.x / MAP_UNIT_DIST);

		move(mapSteps, vel);

		return;
	}

	void moveEast(int mapSteps)
	{
		switch (facing)
		{
		case FOREWARD:
			myRobot->lock();
			myRobot->setDeltaHeading(90);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = RIGHT;
			break;
		case RIGHT:
			break;
		case LEFT:
			myRobot->lock();
			myRobot->setDeltaHeading(180);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = RIGHT;
			break;
		case BACKWARD:
			myRobot->lock();
			myRobot->setDeltaHeading(-90);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = RIGHT;
			break;
		default:
			cout << "Invalid direction." << endl;
			return;
		}

		SNOWFLAKE_STATE.angle = 0;
		SNOWFLAKE_STATE.x += mapSteps*MAP_UNIT_DIST;

		worldMap->setPos(SNOWFLAKE_STATE.z / MAP_UNIT_DIST, SNOWFLAKE_STATE.x / MAP_UNIT_DIST);

		move(mapSteps);

		return;
	}

	void moveEast(int mapSteps, unsigned int vel)
	{
		switch (facing)
		{
		case FOREWARD:
			myRobot->lock();
			myRobot->setDeltaHeading(90);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = 1;
			break;
		case RIGHT:
			break;
		case LEFT:
			myRobot->lock();
			myRobot->setDeltaHeading(180);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = 1;
			break;
		case BACKWARD:
			myRobot->lock();
			myRobot->setDeltaHeading(-90);
			myRobot->unlock();
			ArUtil::sleep(heading_secs); 
			facing = 1;
			break;
		default:
			cout << "Invalid direction." << endl;
			return;
		}

		SNOWFLAKE_STATE.angle = 0;
		SNOWFLAKE_STATE.x += mapSteps*MAP_UNIT_DIST;

		worldMap->setPos(SNOWFLAKE_STATE.z / MAP_UNIT_DIST, SNOWFLAKE_STATE.x / MAP_UNIT_DIST);

		move(mapSteps, vel);
		
		return;
	}

	void move(int mapSteps)
	{
		myRobot->lock();
		myRobot->setVel(velocity);
		myRobot->unlock();
		ArUtil::sleep(sleep_secs * mapSteps);

		myRobot->lock();
		myRobot->setVel(0);
		myRobot->unlock();

		return;
	}

	void move(int mapSteps, unsigned int vel)
	{
		myRobot->lock();
		myRobot->setVel(vel);
		myRobot->unlock();
		ArUtil::sleep((int(1000 * ((unit_dist) / (double(vel))))) * mapSteps);

		myRobot->lock();
		myRobot->setVel(0);
		myRobot->unlock();

		return;
	}

	void moveForward(int mapSteps)
	{
		myRobot->lock();
		myRobot->setVel(velocity);
		myRobot->unlock();
		ArUtil::sleep(sleep_secs * mapSteps);

		cout << "I'm DONE moving! " << (sleep_secs * mapSteps) << endl;

		myRobot->lock();
		myRobot->setVel(0);
		myRobot->unlock();

		switch (SNOWFLAKE_STATE.angle)
		{
		case 0:
			SNOWFLAKE_STATE.x += MAP_UNIT_DIST * mapSteps;
			break;
		case 90:
			SNOWFLAKE_STATE.z += MAP_UNIT_DIST * mapSteps;
			break;
		case 180: 
			SNOWFLAKE_STATE.x -= MAP_UNIT_DIST * mapSteps;
			break;
		case 270:
			SNOWFLAKE_STATE.z -= MAP_UNIT_DIST * mapSteps;
			break;
		default:
			cout << "Angle is wonky." << endl;
			break;
		}

		worldMap->setPos(SNOWFLAKE_STATE.z / MAP_UNIT_DIST, SNOWFLAKE_STATE.x / MAP_UNIT_DIST);

		return;
	}

	void moveForward(int mapSteps, unsigned int vel)
	{
		myRobot->lock();
		myRobot->setVel(vel);
		myRobot->unlock();
		ArUtil::sleep((int(1000 * ((unit_dist) / (double(vel))))) * mapSteps);

		myRobot->lock();
		myRobot->setVel(0);
		myRobot->unlock();

		switch (SNOWFLAKE_STATE.angle)
		{
		case 0:
			SNOWFLAKE_STATE.x += MAP_UNIT_DIST * mapSteps;
			break;
		case 90:
			SNOWFLAKE_STATE.z += MAP_UNIT_DIST * mapSteps;
			break;
		case 180:
			SNOWFLAKE_STATE.x -= MAP_UNIT_DIST * mapSteps;
			break;
		case 270:
			SNOWFLAKE_STATE.z -= MAP_UNIT_DIST * mapSteps;
			break;
		default:
			cout << "Angle is wonky." << endl;
			break;
		}

		worldMap->setPos(SNOWFLAKE_STATE.z / MAP_UNIT_DIST, SNOWFLAKE_STATE.x / MAP_UNIT_DIST);

		return;
	}
};

class Camera : public ArASyncTask
{
	ArCondition myCondition;
	ArMutex myMutex;
	Mat webcamFrame;
	Vec3b pixel;
	const int device = 0;
	int index;

public:
	Camera()
	{
	}

	void* runThread(void*)
	{
		cout << "Camera thread started." << endl;

		videoInput vid;

		vid.setupDevice(device);
		//vid.setupDevice(device, 1920, 1080);

		const unsigned int width = vid.getWidth(device);
		const unsigned int height = vid.getHeight(device);
		const unsigned int size = vid.getSize(device);

		vid.setIdealFramerate(device, 60);

		unsigned char * vidBuffer = new unsigned char[size];

		Mat Frame(height, width, CV_8UC3);

		webcamFrame = Frame;

		cout << "Width: " << width << endl;
		cout << "Height: " << height << endl;
		
		while(this->getRunningWithLock())
		{
			myMutex.lock();

			/*
			if (vid.isFrameNew(device))
			{
				vid.getPixels(device, vidBuffer, false, true);
			}
			*/
			vid.getPixels(device, vidBuffer, false, true);

			for (unsigned int a = 0; a < height; a++)
			{
				for (unsigned int b = 0; b < width; b++)
				{
					index = 3 * (a*width + b);
					pixel.val[0] = vidBuffer[index];
					pixel.val[1] = vidBuffer[index + 1];
					pixel.val[2] = vidBuffer[index + 2];

					webcamFrame.at<Vec3b>(a, b) = pixel;
				}
			}

			//imshow("Image", webcamFrame);
			//waitKey(1);

			myCondition.signal();

			myMutex.unlock();
			ArUtil::sleep(33);
		}

		vid.stopDevice(device);

		return NULL;
	}

	void waitOnCondition()
	{
		myCondition.wait();
	}

	void lockMutex()
	{
		myMutex.lock();
	}
	
	void unlockMutex()
	{
		myMutex.unlock();
	}

	Mat getImage()
	{
		return webcamFrame;
	}
};

class ConnHandler
{
	/*
	Connection Handler code taken from connection demo in Aria
	as well as lines from its declaration, connecting to the robot, and parsing arguments
	*/
public:
	ConnHandler(ArRobot *robot) :
		myConnectedCB(this, &ConnHandler::connected),
		myConnFailCB(this, &ConnHandler::connFail),
		myDisconnectedCB(this, &ConnHandler::disconnected)
	{
		myRobot = robot;
		myRobot->addConnectCB(&myConnectedCB, ArListPos::FIRST);
		myRobot->addFailedConnectCB(&myConnFailCB, ArListPos::FIRST);
		myRobot->addDisconnectNormallyCB(&myDisconnectedCB, ArListPos::FIRST);
		myRobot->addDisconnectOnErrorCB(&myDisconnectedCB, ArListPos::FIRST);
	}
	
	~ConnHandler(void) {};
	
	void connected(void)
	{
		printf("Connection handler: Connected\n");
		myRobot->comInt(ArCommands::SONAR, 0);
		myRobot->comInt(ArCommands::ENABLE, 1);
		myRobot->comInt(ArCommands::SOUNDTOG, 1);
	}

	void connFail(void)
	{
		printf("Connection handler: Failed to connect.\n");
		myRobot->stopRunning();
		Aria::exit(1);
		return;
	}
	
	void disconnected(void)
	{
		printf("Connection handler: Lost connection, exiting program.\n");
		Aria::exit(0);
	}
protected:
	ArRobot *myRobot;
	ArFunctorC<ConnHandler> myConnectedCB;
	ArFunctorC<ConnHandler> myConnFailCB;
	ArFunctorC<ConnHandler> myDisconnectedCB;
};

int compareDist(const DMatch a, const DMatch b)
{
	return (float)(a.distance < b.distance);
}

Point3d convertPoint(Point2f pixel) 
{
	Point3d point;

	double imageScale = 1.0; //3.0
	double ox = 329.222*imageScale;
	double oy = 245.245*imageScale;
	double fsx = 667.416*imageScale;
	double fsy = 663.209*imageScale;
	double skew = 0;
	double vx = pixel.x;
	double vy = pixel.y;
	//double vz = -1.0;

	// Subtract the center
	vx -= ox;
	vy -= oy;

	// Scale accordingly
	vx /= fsx;
	vy /= fsy;

	// Undo skewing
	vx -= skew*vy;

	point.x = 1.0;
	point.y = vy;
	point.z = -vx;

	//cout << "(" << pixel.x << "," << pixel.y << ") --> ( " << point.x << " , " << point.y << " , " << point.z << " )" << endl;

	return point;
}

void convertKeyPoints(vector<Point2f> &keyPointsIn, vector<Point3d> &keyPointsOut)
{
	for (unsigned int i = 0; i < keyPointsIn.size(); i++)
	{
		keyPointsOut.push_back(convertPoint(keyPointsIn[i]));
	}

	return;
}

Point2d projectPlane(Point3d in)
{
	Point2d out(0, 0);
	Point3d temp(0, 0, 0);

	temp = in;// in - ((in.dot(NORMAL_VECTOR) / norm(NORMAL_VECTOR))*(Point3d)NORMAL_VECTOR);

	out.x = temp.x;
	out.y = temp.z;

	return out;
}

vector<Point2d> getProjections(vector<Point3d> &in)
{
	vector<Point2d> out;

	for (unsigned int i = 0; i < in.size(); i++)
	{
		out.push_back(projectPlane(in[i]));
	}

	return out;
}

Mat makeA(Point2d slope1, Point2d slope2)
{
	Mat A(2, 2, CV_32F);

	A.at<float>(0, 0) = slope1.x;
	A.at<float>(0, 1) = -slope2.x;
	A.at<float>(1, 0) = slope1.y;
	A.at<float>(1, 1) = -slope2.y;

	return A;
}

Point2d leastSquareFit(Mat A, Point2d b, unsigned int svd = 0)
{
	Point2d fit(0, 0);
	Mat stuff;

	if (!svd)
	{
		stuff = (A.t() * A).inv() * A.t();
	}
	else
	{
		stuff = ((A.t() * A).inv(DECOMP_SVD) * A.t());
	}
	
	fit.x = stuff.at<float>(0,0) * fit.x + stuff.at<float>(0,1) * fit.y;
	fit.y = stuff.at<float>(1,0) * fit.x + stuff.at<float>(1,1) * fit.y;

	return fit;
}

Point2d intersectionVals(Point2d v1, Point2d v2)
{
	Point2d intersectionVals(0, 0);
	Mat A = makeA(v1, v2);
	Point2d b;

	b.x = -START_POINT.x + END_POINT.x;
	b.y = -START_POINT.z + END_POINT.z;

	if (determinant(A))
	{
		//leastSquareFit(A, b);
		Mat Z;
		Mat B(2, 1, CV_32F);
		B.at<float>(0, 0) = b.x;
		B.at<float>(1, 0) = b.y;
		cv::solve(A, B, Z);
		intersectionVals.x = Z.at<float>(0, 0);
		intersectionVals.y = Z.at<float>(1, 0);

	}
	else
	{
		cout << "Lines are parallel, trying SVD" << endl;
		leastSquareFit(A, b, 1);
	}

	return intersectionVals;
}

vector<Point2d> findIntersections(vector<Point2d> &v1, vector<Point2d> &v2)
{
	vector<Point2d> intersections;

	for (unsigned int i = 0; i < v1.size(); i++)
	{
		intersections.push_back(intersectionVals(v1[i], v2[i]));
		//cout << "INTERSECTION " << i << ": " << intersections.at(intersections.size() - 1) << endl;
	}

	return intersections;
}

bool reconstructPoint(Point2d val, Point3d vec1, Point3d vec2, Point3d &result)
{
	Point3d Point1(0, 0, 0);
	Point3d Point2(0, 0, 0);

	bool valid = true;

	if (val.x < 0 || val.y < 0)
		valid = false;

	Point1 = START_POINT + val.x * vec1;
	Point2 = END_POINT + val.y * vec2;

	Point1.x = (Point1.x + Point2.x) / 2;
	Point1.y = (Point1.y + Point2.y) / 2;
	Point1.z = (Point1.z + Point2.z) / 2;

	result = Point1;

	return valid;
}

vector<Point3d> reconstructPoints(vector<Point2d> &val, vector<Point3d> &direction1, vector<Point3d> &direction2)
{
	vector<Point3d> reconstruction;

	for (unsigned int i = 0; i < direction1.size(); i++)
	{
		Point3d ipoint;
		cout << "R" << i << ": " << val[i].x << " " << val[i].y << endl;

		if (reconstructPoint(val[i], direction1[i], direction2[i], ipoint)){
			reconstruction.push_back(ipoint);
			//cout << "POINT: " << reconstruction.at(reconstruction.size() - 1) << endl;
		}
	}

	return reconstruction;
}

vector<Point3d> doVectorMath(vector<Point3d> &v1, vector<Point3d> &v2)
{
	vector<Point3d> result;
	vector<Point2d> projected1;
	vector<Point2d> projected2;
	vector<Point2d> intersections;

	projected1 = getProjections(v1);
	projected2 = getProjections(v2);

	intersections = findIntersections(projected1, projected2);

	result = reconstructPoints(intersections, v1, v2);

	return result;
}

Point3d localToGlobalPoint(Point3d pt, bool doTranslate = true)
{
	Point3d point(0,0,0);
	Mat transform(3, 3, CV_32F);

	transform.at<float>(0, 0) = (float)cos(SNOWFLAKE_STATE.angle * CV_PI / 180);
	transform.at<float>(0, 1) = 0;
	transform.at<float>(0, 2) = (float)sin(SNOWFLAKE_STATE.angle * CV_PI / 180);
	transform.at<float>(1, 0) = 0;
	transform.at<float>(1, 1) = 1;
	transform.at<float>(1, 2) = 0;
	transform.at<float>(2, 0) = -(float)sin(SNOWFLAKE_STATE.angle * CV_PI / 180);
	transform.at<float>(2, 1) = 0;
	transform.at<float>(2, 2) = (float)cos(SNOWFLAKE_STATE.angle * CV_PI / 180);

	point.x = transform.at<float>(0, 0) * pt.x + transform.at<float>(0, 1) * pt.y + transform.at<float>(0, 2) * pt.z;
	point.y = transform.at<float>(1, 0) * pt.x + transform.at<float>(1, 1) * pt.y + transform.at<float>(1, 2) * pt.z;
	point.z = transform.at<float>(2, 0) * pt.x + transform.at<float>(2, 1) * pt.y + transform.at<float>(2, 2) * pt.z;

	if (doTranslate) {
		point.x += SNOWFLAKE_STATE.x;
		point.z += SNOWFLAKE_STATE.z;
	}

	return point;
}

vector<Point3d> localToGlobal(vector<Point3d> &points, bool doTranslate = true)
{
	vector<Point3d> globals;

	for (unsigned int d = 0; d < points.size(); d++)
	{
		globals.push_back(localToGlobalPoint(points[d], doTranslate));
		//cout << "GLOBALS: " << globals.at(globals.size() - 1) << endl;
	}

	return globals;
}

vector<Point3d> removeFloorPoints(vector<Point3d> &points)
{
	vector<Point3d> notFloors;

	cout << "Points before floor removal" << points.size() << endl;

	for (unsigned int i = 0; i < points.size(); i++)
	{
		if (points[i].y <= 0)
		{
			notFloors.push_back(points[i]);
		}
	}

	cout << "Not floor points: " << notFloors.size() << endl;

	return notFloors;
}

Point2i pointToCell(Point3d point)
{
	Point2i pt;

	//pt.x = int((double(MAP_WIDTH) / 2.0) + (point.x / double(MAP_UNIT_DIST)));
	//pt.y = int((double(MAP_HEIGHT) / 2.0) + (point.z / double(MAP_UNIT_DIST)));

	pt.x = (point.x / double(MAP_UNIT_DIST));
	pt.y = (point.z / double(MAP_UNIT_DIST));

	//cout << "3D (" << point << ") --> 2D (" << pt << ")" << endl;

	return pt;
}

void drawPoints(vector<Point3d> &points)
{
	int x = 0;
	int y = 0;

	for (unsigned int i = 0; i < points.size(); i++)
	{
		x = points[i].x /10;
		y = points[i].z /10;

		if ((x < rawWorldMap.cols) && (x >= 0) && (y >= 0) && (y < rawWorldMap.rows))
		{
			circle(rawWorldMap, Point2i(x, y), 3, Scalar(255, 0, 255), -1);
			//rawWorldMap.at<Vec3b>(y, x) = Vec3b(255,0,255);
		}
	}

	return;
}

void drawLines(vector<Point3d> &preV1, vector<Point3d> &preV2)
{
	vector<Point3d> startAndEnd;
	vector<Point3d> v1, v2;

	//cout << "BEFORE: " << preV1.at(0).x << " " << preV1.at(0).z << endl;

	v1 = localToGlobal(preV1, false);
	v2 = localToGlobal(preV2, false);

	//cout << "AFTER: " << v1.at(0).x << " " << v1.at(0).z << endl;
	
	float scale = 1000.0;
	
	startAndEnd.push_back(Point3d(START_POINT));
	startAndEnd.push_back(Point3d(END_POINT));

	startAndEnd = localToGlobal(startAndEnd);



	for (int i = 0; i < v1.size(); i++)
	{
		line(rawWorldMap, Point2i(startAndEnd[0].x / 10.0, startAndEnd[0].z / 10.0), Point2i(startAndEnd[0].x / 10.0 + scale * v1[i].x / 10.0, startAndEnd[0].z / 10.0 + scale * v1[i].z / 10.0), Scalar(0, 0, 255));
	

		cout << "LINE " << i << ": ";

		cout << startAndEnd[0].x / 10.0 << " " << startAndEnd[0].z / 10.0 << " --> ";
		cout << (startAndEnd[0].x / 10.0 + scale * v1[i].x / 10.0) << " " << (startAndEnd[0].z / 10.0 + scale * v1[i].z / 10.0);
		cout << endl;

	}

	for (int i = 0; i < v2.size(); i++)
	{
		line(rawWorldMap, Point2i(startAndEnd[1].x / 10.0, startAndEnd[1].z / 10.0), Point2i(startAndEnd[1].x / 10.0 + scale * v2[i].x / 10.0, startAndEnd[1].z / 10.0 + scale * v2[i].z / 10.0), Scalar(0, 255, 0));
	}

	return;
}

vector<Point2i> pointsToMapCells(vector<Point3d> &points)
{
	vector<Point2i> mapCells;

	drawPoints(points);

	for (unsigned int i = 0; i < points.size(); i++)
	{
		mapCells.push_back(pointToCell(points[i]));
	}

	return mapCells;
}

void addToMap(vector<Point2i> mapPoints)
{
	for (unsigned int i = 0; i < mapPoints.size(); i++)
	{
		if (mapPoints[i].x > 0 && mapPoints[i].x < MAP_WIDTH)
		{
			if (mapPoints[i].y > 0 && mapPoints[i].y < MAP_HEIGHT)
			{
				worldMap->set_imp(mapPoints[i].y, mapPoints[i].x);
			}
		}
	}

	return;
}

void discoveryZone(ArRobot &robot, Camera &cam)
{
	Mat webcamFrame1;
	Mat webcamFrame2;
	Mat webcamFrame1Gray;
	Mat webcamFrame2Gray;
	Mat descriptors1;
	Mat descriptors2;
	Mat imageMatches;

	vector<DMatch> matches;
	vector<KeyPoint> keyPoints1;
	vector<KeyPoint> keyPoints2;
	vector<Point2f> pointList1;
	vector<Point2f> pointList2;
	vector<Point3d> directionVectors1;
	vector<Point3d> directionVectors2;
	vector<Point2i> mapPoints;

	FlannBasedMatcher matcher;
	//ORB orb(300, 1.2f, 16, 93, 0, 2, ORB::HARRIS_SCORE, 93);



	Ptr<ORB> orb = ORB::create(300, 1.2f, 16, 93, 0, 2, ORB::HARRIS_SCORE, 93);



	bool failed = false;

	cam.waitOnCondition();
	cam.lockMutex();
	cam.getImage().copyTo(webcamFrame1);
	cam.unlockMutex();
	
	imshow("Frame 1", webcamFrame1);
	waitKey(1);

	cout << "\tDiscovery Zone: first image" << endl;

	long starttime, endtime;
	starttime = clock();
	
	robot.lock();
	robot.setVel(200);
	robot.unlock();
	ArUtil::sleep(int(1000 * (MAP_UNIT_DIST/(double(200)))));

	endtime = clock();
	cout << "Slept " << (endtime - starttime) << endl;
	cout << "DONE MOVING" << endl;

	robot.lock();
	robot.setVel(0);
	robot.unlock();
	
	switch (SNOWFLAKE_STATE.angle)
	{
	case 0:
		SNOWFLAKE_STATE.x += MAP_UNIT_DIST;
		break;
	case 90:
		SNOWFLAKE_STATE.z += MAP_UNIT_DIST;
		break;
	case 180:
		SNOWFLAKE_STATE.x -= MAP_UNIT_DIST;
		break;
	case 270:
		SNOWFLAKE_STATE.z -= MAP_UNIT_DIST;
		break;
	}

	worldMap->setPos(SNOWFLAKE_STATE.z / MAP_UNIT_DIST, SNOWFLAKE_STATE.x / MAP_UNIT_DIST);//this_thread::sleep_for(chrono::seconds(2)); //See if necessary maybe via clock
	ArUtil::sleep(3000);

	cam.waitOnCondition();
	cam.lockMutex();
	cam.getImage().copyTo(webcamFrame2);
	cam.unlockMutex();
	
	imshow("Frame 2", webcamFrame2);
	waitKey(1);

	cout << "\tDiscovery Zone: second image" << endl;
	
	cout << "Discovery Zone: Camera Image Set Acquired" << endl;

	cvtColor(webcamFrame1, webcamFrame1Gray, COLOR_BGR2GRAY);
	cvtColor(webcamFrame2, webcamFrame2Gray, COLOR_BGR2GRAY);

	try
	{
		orb->detectAndCompute(webcamFrame1Gray, noArray(), keyPoints1, descriptors1, false);
		orb->detectAndCompute(webcamFrame2Gray, noArray(), keyPoints2, descriptors2, false);
	}
	catch (...)
	{
		failed = true;
		cout << "Error in ORB matching." << endl;
	}

	if (!failed)
	{
		descriptors1.convertTo(descriptors1, CV_32F);
		descriptors2.convertTo(descriptors2, CV_32F);

		matcher.match(descriptors1, descriptors2, matches);

		sort(matches.begin(), matches.end(), compareDist);

		cout << "Matches: " << matches.size() << endl;

		
		while (matches.size() > 150)
		{
			matches.pop_back();
		}
		

		cout << "Matches left: " << matches.size() << endl;

		//drawMatches(webcamFrame1, keyPoints1, webcamFrame2, keyPoints2, matches, imageMatches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

		//imshow("Image Matches", imageMatches);

		//waitKey(1);
	}

	for (unsigned int d = 0; d < matches.size(); d++)
	{
		pointList1.push_back(keyPoints1[matches[d].queryIdx].pt);
		pointList2.push_back(keyPoints2[matches[d].trainIdx].pt);
	}

	cout << "Point list 1 size: " << pointList1.size() << endl;
	cout << "Point list 2 size: " << pointList2.size() << endl;

	convertKeyPoints(pointList1, directionVectors1);
	convertKeyPoints(pointList2, directionVectors2);
	
	drawLines(directionVectors1, directionVectors2);

	mapPoints = pointsToMapCells(removeFloorPoints(localToGlobal(doVectorMath(directionVectors1, directionVectors2))));
	
	//cout << "Point list: " << endl;

	//for (int b = 0; b < mapPoints.size(); b++)
	//{
	//	cout << "( " << mapPoints[b].x << " , " << mapPoints[b].y << " )" << endl;
	//}

	addToMap(mapPoints);

	destroyAllWindows();

	namedWindow("Map", WINDOW_NORMAL);
	imshow("Map", rawWorldMap);
	waitKey(1);

	return;
}

void explore(ArRobot &robot, Camera &cam, Speech &sp)
{	
	sp.Talk("Exploring");

	for (int i = 0; i < 4; i++)//4
	{
		cout << "DIRECTION " << i << endl;
		robot.lock();
		robot.setDeltaHeading(90);
		robot.unlock();
		ArUtil::sleep(4000);

		SNOWFLAKE_STATE.angle += 90;
		SNOWFLAKE_STATE.angle %= 360;

		switch (i)
		{

		case 0:
			SNOWFLAKE_STATE.x += MAP_UNIT_DIST;
			break;
		case 1:
			SNOWFLAKE_STATE.z += MAP_UNIT_DIST;
			break;
		case 2:
			SNOWFLAKE_STATE.x -= MAP_UNIT_DIST;
			break;
		case 3:
			SNOWFLAKE_STATE.z -= MAP_UNIT_DIST;
			break;
		default:
			cout << "i must be in [0,3], i is: " << i << endl;
			break;
		}

		worldMap->setPos(SNOWFLAKE_STATE.z / MAP_UNIT_DIST, SNOWFLAKE_STATE.x / MAP_UNIT_DIST);

		ArUtil::sleep(2000);
		discoveryZone(robot, cam);

		//imwrite("c:/users/user/desktop/RawMap.bmp", rawWorldMap);
		imwrite("RawMap.bmp", rawWorldMap);

		ArUtil::sleep(2000);
	}

	sp.Talk("Done");

	return;
}

int main(int argc, char **argv)
{
	if (FAILED(::CoInitialize(NULL)))
		cout << "CoInitialize failed" << endl;

	Speech sp;
	sp.Talk("Hello, my name is Snowflake");

	Aria::init();

	worldMap = new Map(MAP_HEIGHT, MAP_WIDTH, 2, 1);

	worldMap->setPos(MAP_HEIGHT/2, MAP_WIDTH/2);
	int mapX, mapY;
	worldMap->getPos(mapX, mapY);

	SNOWFLAKE_STATE.angle = 90;
	SNOWFLAKE_STATE.x = mapX*MAP_UNIT_DIST;
	SNOWFLAKE_STATE.z = mapY*MAP_UNIT_DIST;

	Camera cam;

	ArArgumentParser argParser(&argc, argv);
	ArRobot robot;
	ArRobotConnector con(&argParser, &robot);
	ConnHandler ch(&robot);
	//ArSonarDevice sonar;
	ArKeyHandler keyHandler;

	int safe = 0;

	//namedWindow("Image Matches", WINDOW_AUTOSIZE);

	argParser.loadDefaultArguments();

	if (!Aria::parseArgs())
	{
		Aria::logOptions();
		Aria::exit(EXIT_FAILURE);
		return EXIT_FAILURE;
	}

	if (!con.connectRobot())
	{
		ArLog::log(ArLog::Normal, "Could not connect to the robot.");

		if (argParser.checkHelpAndWarnUnparsed())
		{
			Aria::logOptions();
		}
		Aria::exit(EXIT_FAILURE);
		return EXIT_FAILURE;
	}

	ArLog::log(ArLog::Normal, "Connected to robot.");

	if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		Aria::exit(EXIT_FAILURE);
	}

	Aria::setKeyHandler(&keyHandler);
	
	robot.attachKeyHandler(&keyHandler);
	
	cout << "Press esc to exit." << endl;

	cam.runAsync();
	robot.runAsync(true);

	cout << "Threads started." << endl;

	robot.clearDirectMotion();

	robot.lock();
	robot.setVel(0);
	robot.unlock();

	// MAIN PART STUFF AND THINGS

	//explore(robot, cam, sp);

	VoiceRecog VR;
	VR.runAsync();

	ExecuteCommands *commander = new ExecuteCommands(&robot, MAP_UNIT_DIST, 200);
	string command;
	
	cout << "DEBUG EXPLORE" << endl;

	explore(robot, cam, sp);

	while(true)
	{
		//Do the thing	
		VR.lockMutex();
		VR.getLastCommand(command);
		
		if (command.length() != 0)
		{
			if (!commandMode) {
				if (command == "snowflake"){
					commandMode = true;
					sp.Talk("Yes?");
				}
			}
			else 
			{
				if (command == "yes")
				{
					//just in case.
				}
				if (command == "set a")
				{
					commandMode = false;
					sp.Talk("Setting location A");
					worldMap->setLoc("a", mapX, mapY);	
					
				}
				if (command == "set b")
				{
					commandMode = false;
					sp.Talk("Setting location B");
					worldMap->setLoc("b", mapX, mapY);
					
				}
				if (command == "set c")
				{
					commandMode = false;
					sp.Talk("Setting location C");
					worldMap->setLoc("c", mapX, mapY);
				}
				if (command == "explore")
				{
					commandMode = false;
					explore(robot, cam, sp);
				}
				if (command == "goto a")
				{
					commandMode = false;
					sp.Talk("Going to A");
					worldMap->BFS("a");
					commander->execute(Map::Guide(worldMap));
				}
				if (command == "goto b")
				{
					commandMode = false;
					sp.Talk("Going to B");
					worldMap->BFS("b");
					commander->execute(Map::Guide(worldMap));
				}
				if (command == "goto c")
				{
					commandMode = false;
					sp.Talk("Going to C");
					worldMap->BFS("c");
					commander->execute(Map::Guide(worldMap));
				}
				if (command == "rotate north")
				{
					commandMode = false;
					sp.Talk("Turning North.");
					commander->moveNorth(0);	
				}
				if (command == "rotate south")
				{
					commandMode = false;
					sp.Talk("Turning South.");
					commander->moveSouth(0);
				}
				if (command == "rotate east")
				{
					commandMode = false;
					sp.Talk("Turning east.");
					commander->moveEast(0);
				}
				if (command == "rotate west")
				{
					commandMode = false;
					sp.Talk("Turning west.");
					commander->moveWest(0);
				}
				if (command == "move forward one")
				{
					commandMode = false;
					sp.Talk("Rolling.");
					cout << "Rolling rolling rolling...RAWHIDE." << endl;
					commander->moveForward(1);
				}
				if (command == "move forward two")
				{
					commandMode = false;
					sp.Talk("Rolling.");
					cout << "Rolling rolling rolling...RAWHIDE." << endl;
					commander->moveForward(2);
				}
				if (command == "move forward three")
				{
					commandMode = false;
					sp.Talk("Rolling.");
					cout << "Rolling rolling rolling...RAWHIDE." << endl;
					commander->moveForward(3);
				}
				if (command == "move forward four")
				{
					commandMode = false;
					sp.Talk("Rolling.");
					cout << "Rolling rolling rolling...RAWHIDE." << endl;
					commander->moveForward(4);
				}
				if (command == "move forward five")
				{
					commandMode = false;
					sp.Talk("Rolling.");
					cout << "Rolling rolling rolling...RAWHIDE." << endl;
					commander->moveForward(5);
				}
			}
		}

		//ArUtil::sleep(500);

		VR.unlockMutex();

	}
	
	worldMap->SaveImage(Map::CreateImage(worldMap), "C:/users/user/desktop/MAP.bmp");
	

	robot.waitForRunExit();
	
	ArLog::log(ArLog::Normal, "Main thread: Waiting for the secondary threads to exit, then exiting the program.");

	cam.stopRunning();

	do
	{
		ArUtil::sleep(150);
	}while(cam.getRunningWithLock());

	VR.stopRunning();

	do
	{
		ArUtil::sleep(150);
	} while (VR.getRunningWithLock());

	Aria::exit(EXIT_SUCCESS);

	::CoUninitialize();

	return EXIT_SUCCESS;
}
