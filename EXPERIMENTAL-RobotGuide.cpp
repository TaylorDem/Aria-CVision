#include <Aria.h>
#include <opencv2\opencv.hpp>
#include <opencv2\features2d\features2d.hpp>
#include "videoInput.h"
#include <math.h>
#include <chrono>
#include <thread>
#include "Map_BFS.h"
#include "Command.h"

using namespace std;
using namespace cv;

struct RobotPosition
{
	double x;
	double z;
	int angle;
};

const unsigned int MAP_UNIT_DIST = 508;
const double FLOOR_HEIGHT = -279.4;
const double CAMERA_Z_OFFSET = -241.3;
const int MAP_WIDTH = 32;
const int MAP_HEIGHT = 32;

const Point3i NORMAL_VECTOR(0, 1, 0);
const Point3d START_POINT(0, 0, 0);
const Point3d END_POINT(0, 0, MAP_UNIT_DIST);

RobotPosition SNOWFLAKE_STATE;
Map *worldMap;

class executeCommands
{
	enum { FOREWARD = 0, RIGHT = 1, LEFT = 2, BACKWARD = 3 };
	ArRobot *myRobot;
	vector<Command> commandList;
	unsigned int unit_dist;
	unsigned int velocity;
	unsigned int heading_secs = 3000;
	unsigned int sleep_secs;
	unsigned int facing = FOREWARD;

public:
	executeCommands(ArRobot *robot, unsigned int map_unit_dist, unsigned int vel, vector<Command> list) : myRobot(robot), unit_dist(map_unit_dist), velocity(vel)
	{
		sleep_secs = int ( 1000 * ( ( map_unit_dist ) / ( double( vel ) ) ) );
		commandList = list;
	}

	void execute()
	{
		for (unsigned int i = 0; i < commandList.size(); i++)
		{
			switch (commandList[i].heading)
			{
			case FOREWARD:
				moveForeward(commandList[i].steps);
				break;
			case RIGHT:
				moveRight(commandList[i].steps);
				break;
			case LEFT:
				moveLeft(commandList[i].steps);
				break;
			case BACKWARD:
				moveBackward(commandList[i].steps);
				break;
			default:
				cout << "Invalid heading, skipping command." << endl;
				break;
			}
		}

		return;
	}

	void moveForeward(int mapSteps)
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

		move(mapSteps);

		return;
	}

	void moveForeward(int mapSteps, unsigned int vel)
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

		move(mapSteps, vel);

		return;
	}

	void moveBackward(int mapSteps)
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

		move(mapSteps);

		return;
	}

	void moveBackward(int mapSteps, unsigned int vel)
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

		move(mapSteps, vel);

		return;
	}

	void moveLeft(int mapSteps)
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

		move(mapSteps);

		return;
	}

	void moveLeft(int mapSteps, unsigned int vel)
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

		move(mapSteps, vel);

		return;
	}

	void moveRight(int mapSteps)
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

		move(mapSteps);

		return;
	}

	void moveRight(int mapSteps, unsigned int vel)
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

		move(mapSteps, vel);
		
		return;
	}

	void move(int mapSteps)
	{
		myRobot->lock();
		myRobot->setVel(velocity);
		myRobot->unlock();
		ArUtil::sleep(1000 * sleep_secs * mapSteps);

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
		myRobot->comInt(ArCommands::SONAR, 1);
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

int checkSonar(ArRobot &robot)
{
	int safe = 1;

	robot.lock();

	for (int i = 1; i < 15; i++)
	{
		if ((i != 7) || (i != 8))
		{
			if (robot.getSonarRange(i) < MAP_UNIT_DIST)
			{
				safe = 0;
				break;
			}
		}
	}

	robot.unlock();

	return safe;
}

bool isMoving(ArRobot &robot)
{
	double values = 0;
	bool isMoving = false;

	robot.lock();
	values = robot.getVel() + robot.getRotVel();
	robot.unlock();

	if ( values > 0 )
	{
		isMoving = true;
	}

	return isMoving;
}

int compareDist(const DMatch a, const DMatch b)
{
	return (float)(a.distance < b.distance);
}

Point3d convertPoint(Point2f pixel) 
{
	Point3d point;

	double imageScale = 3.0;
	double ox = 329.222*imageScale;
	double oz = 245.245*imageScale;
	double fsx = 667.416*imageScale;
	double fsz = 663.209*imageScale;
	double skew = 0;
	double vx = pixel.x;
	double vy = 1.0;
	double vz = pixel.y;

	// Subtract the center
	vx -= ox;
	vz -= oz;

	// Scale accordingly
	vx /= fsx;
	vz /= fsz;

	// Undo skewing
	vx -= skew*vz;

	point.x = vx;
	point.y = vy;
	point.z = vz;

	return point;
}

void convertKeyPoints(vector<Point2f> keyPointsIn, vector<Point3d> keyPointsOut)
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

	temp = in - ((in.dot(NORMAL_VECTOR) / norm(NORMAL_VECTOR))*(Point3d)NORMAL_VECTOR);

	out.x = temp.x;
	out.y = temp.z;

	return out;
}

vector<Point2d> getProjections(vector<Point3d> in)
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
	A.at<float>(0, 1) = slope2.x;
	A.at<float>(1, 0) = slope1.y;
	A.at<float>(1, 1) = slope2.y;

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

	b.x = START_POINT.x - END_POINT.x;
	b.y = START_POINT.z - END_POINT.z;

	if (determinant(A))
	{
		leastSquareFit(A, b);
	}
	else
	{
		cout << "Lines are parallel, trying SVD" << endl;
		leastSquareFit(A, b, 1);
	}

	return intersectionVals;
}

vector<Point2d> findIntersections(vector<Point2d> v1, vector<Point2d> v2)
{
	vector<Point2d> intersections;

	for (unsigned int i = 0; i < v1.size(); i++)
	{
		intersections.push_back(intersectionVals(v1[i], v2[i]));
	}

	return intersections;
}

Point3d reconstructPoint(Point2d val, Point3d vec1, Point3d vec2)
{
	Point3d Point1(0,0,0);
	Point3d Point2(0,0,0);

	Point1 = START_POINT + val.x * vec1;
	Point2 = END_POINT + val.y * vec2;

	Point1.x = (Point1.x + Point2.x) / 2;
	Point1.y = (Point1.y + Point2.y) / 2;
	Point1.z = (Point1.z + Point2.z) / 2;

	return Point1;
}

vector<Point3d> reconstructPoints(vector<Point2d> val, vector<Point3d> direction1, vector<Point3d> direction2)
{
	vector<Point3d> reconstruction;

	for (unsigned int i = 0; i < direction1.size(); i++)
	{
		reconstruction.push_back(reconstructPoint(val[i], direction1[i], direction2[i]));
	}

	return reconstruction;
}

vector<Point3d> doVectorMath(vector<Point3d> v1, vector<Point3d> v2)
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

Point3d localToGlobalPoint(Point3d pt)
{
	Point3d point(0,0,0);

	switch (SNOWFLAKE_STATE.angle)
	{
	case 0:
		point.x = SNOWFLAKE_STATE.x + pt.z;
		point.z = SNOWFLAKE_STATE.z - pt.x;
		break;
	case 90:
		point.x = SNOWFLAKE_STATE.x + pt.x;
		point.z = SNOWFLAKE_STATE.z + pt.z;
		break;
	case 180:
		point.x = SNOWFLAKE_STATE.x - pt.z;
		point.z = SNOWFLAKE_STATE.z + pt.x;
		break;
	case 270:
		point.x = SNOWFLAKE_STATE.x + pt.x;
		point.y = SNOWFLAKE_STATE.z - pt.z;
		break;
	}

	return point;
}

vector<Point3d> localToGlobal(vector<Point3d> points)
{
	vector<Point3d> globals;

	for (unsigned int d = 0; d < points.size(); d++)
	{
		globals.push_back(localToGlobalPoint(points[d]));
	}

	return globals;
}

vector<Point3d> removeFloorPoints(vector<Point3d> points)
{
	vector<Point3d> notFloors;

	for (unsigned int i = 0; i < points.size(); i++)
	{
		if (points[i].y > FLOOR_HEIGHT)
		{
			notFloors.push_back(points[i]);
		}
	}

	return notFloors;
}

Point2i pointToCell(Point3d point)
{
	Point2i pt;

	pt.x = int((MAP_WIDTH / 2) + (point.x / MAP_UNIT_DIST));
	pt.y = int((MAP_HEIGHT / 2) + (point.z / MAP_UNIT_DIST));

	return pt;
}

vector<Point2i> pointsToMapCells(vector<Point3d> points)
{
	vector<Point2i> mapCells;

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
		worldMap->set_imp(mapPoints[i].y, mapPoints[i].x);
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

	FlannBasedMatcher matcher;
	//ORB orb(300, 1.2f, 16, 93, 0, 2, ORB::HARRIS_SCORE, 93);

	bool failed = false;

	cam.waitOnCondition();
	cam.lockMutex();
	cam.getImage().copyTo(webcamFrame1);
	cam.unlockMutex();

	robot.lock();
	robot.setVel(200);
	robot.unlock();
	ArUtil::sleep(1000 * int(MAP_UNIT_DIST/(double(200))));

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

	this_thread::sleep_for(chrono::seconds(2)); //Time to see if necessary maybe via clock

	cam.waitOnCondition();
	cam.lockMutex();
	cam.getImage().copyTo(webcamFrame2);
	cam.unlockMutex();

	cvtColor(webcamFrame1, webcamFrame1Gray, COLOR_BGR2GRAY);
	cvtColor(webcamFrame2, webcamFrame2Gray, COLOR_BGR2GRAY);

	try
	{
		//orb(webcamFrame1Gray, noArray(), keyPoints1, descriptors1, false);
		//orb(webcamFrame2Gray, noArray(), keyPoints2, descriptors2, false);
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

		while (matches.size() > 30)
		{
			matches.pop_back();
		}

		//drawMatches(webcamFrame1, keyPoints1, webcamFrame2, keyPoints2, matches, imageMatches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

		//imshow("Image Matches", imageMatches);

		//waitKey(1);
	}

	for (unsigned int d = 0; d < matches.size(); d++)
	{
		pointList1.push_back(keyPoints1[matches[d].queryIdx].pt);
		pointList2.push_back(keyPoints2[matches[d].trainIdx].pt);
	}

	convertKeyPoints(pointList1, directionVectors1);
	convertKeyPoints(pointList2, directionVectors2);
	
	addToMap(pointsToMapCells(removeFloorPoints(localToGlobal(doVectorMath(directionVectors1, directionVectors2)))));

	return;
}

int main(int argc, char **argv)
{
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
	ArSonarDevice sonar;
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

	for (int i = 0; i < 4; i++)
	{
		robot.lock();
		robot.setDeltaHeading(90);
		robot.unlock();
		ArUtil::sleep(1200);

		SNOWFLAKE_STATE.angle += 90;

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

		discoveryZone(robot, cam);
	}

	worldMap->SaveImage(Map::CreateImage(worldMap), "C:/users/user/desktop/MAP.bmp");

	/*
	while(true)
	{
		//Do Thing	

	}
	*/

	robot.waitForRunExit();

	cam.stopRunning();
  
	ArLog::log(ArLog::Normal, "Main thread: Waiting for the camera thread to exit, then exiting the program.");

	do
	{
		ArUtil::sleep(150);
	}while(cam.getRunningWithLock());

	Aria::exit(EXIT_SUCCESS);

	return EXIT_SUCCESS;
}
