#include "Voice.h"
#include <Aria.h>
#include <opencv2\opencv.hpp>
#include <iostream>
#include "Map_BFS.h"
#include <vector>
#include <ctime>
#include "Speech.cpp"

using namespace cv;
using namespace std;

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



int main11(int argc, char** argv)
{
	if (FAILED(::CoInitialize(NULL)))
		cout << "CoInitialize failed" << endl;

	Speech sp;
	sp.Talk("Hello, my name is Snowflake");


	

	Aria::init();
	ArArgumentParser argParser(&argc, argv);
	ArRobot robot;
	ArRobotConnector con(&argParser, &robot);
	ConnHandler ch(&robot);
	ArSonarDevice sonar;
	ArKeyHandler keyHandler;
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
	VoiceRecog VR;
	VR.runAsync();
	sp.Talk("I'm listening, feel free to give me a command.");	

	Mat image;
	image = imread("C:/Users/Taylor/Desktop/SUNYIT/Capstone/cs-capstone-robotics/BitMap.bmp", IMREAD_COLOR);   // Read the file

	if (!image.data)                              // Check for invalid input
	{
		cout << "Could not open the image" << std::endl;
		return -1;
	}
	//construct vector of map pointers with 2 known (start and end)
	vector<Map*> M;
	Map temp(image.rows, image.cols, 2, 1);
	M.push_back(&temp);
	int num_maps = 2;


	int i;
	//set points based on color on map
	for (i = 0; i< M.front()->rows; i++)
	{
		for (int j = 0; j<M.front()->columns; j++)
		{
			//white == passible
			if (image.at<Vec3b>(i, j)[0] == 255 && image.at<Vec3b>(i, j)[1] == 255 && image.at<Vec3b>(i, j)[2] == 255) M.front()->set_pass(i, j);

			//black == impassible
			if (image.at<Vec3b>(i, j)[0] == 0 && image.at<Vec3b>(i, j)[1] == 0 && image.at<Vec3b>(i, j)[2] == 0) M.front()->set_imp(i, j);

			//green ==start, known[0]
			if (image.at<Vec3b>(i, j)[0] == 0 && image.at<Vec3b>(i, j)[1] == 255 && image.at<Vec3b>(i, j)[2] == 0)
			{
				M.front()->Known[0].rPos = i;		//I think I should only need position saved...
				M.front()->Known[0].cPos = j;
				M.front()->set_pass(i, j);
			}
			//red == end, known[1]
			if (image.at<Vec3b>(i, j)[0] == 0 && image.at<Vec3b>(i, j)[1] == 0 && image.at<Vec3b>(i, j)[2] == 255)
			{
				M.front()->Known[1].rPos = i;
				M.front()->Known[1].cPos = j;
				M.front()->set_pass(i, j);
			}
		}
	}

	//higher level maps initialized
	Map *Higher;
	for (i = 0; i < num_maps - 1; i++)		//5 levels of maps in total. first being highest resolution.
	{
		Higher = Map::HL_Map(M.at(i));
		M.push_back(Higher);
	}
	//create images based on maps
	vector<Mat> images; // (num_maps);
	for (i = 0; i < num_maps; i++)
	{
		Mat Itemp = Map::CreateImage(M.at(i));
		images.push_back(Itemp);
	}
	bool Flag = false;		//holds if possible



	clock_t Timer = clock();
	Flag = M.front()->BFS(M.front()->Known[0], M.front()->Known[1]);	//perform BFS from node representing green => red.
	//Flag = Map::FindPath(M, M.front()->Known[0].rPos / 2, M.front()->Known[0].cPos / 2, M.front()->Known[1].rPos / 2, M.front()->Known[1].cPos / 2, num_maps);
	
	Timer = clock() - Timer;
	cout << "Time to execute BFS: " << Timer << endl;

	Mat distanceImage = image.clone();
	Vec3b pixel;
	double dist;
	int k, m;
	for (k = 0; k < M.front()->rows; k++) {
		for (m = 0; m < M.front()->columns; m++) {
			dist = M.front()->m[k][m].distance;
			dist /= (image.cols*1.0);
			dist *= 255.0;
			pixel.val[0] = pixel.val[1] = pixel.val[2] = dist;
			distanceImage.at<Vec3b>(k, m) = pixel;
		}
	}


	
	//M.BFS_path[] = the latest known path
	if (Flag == true) 		//we have success!
	{
		int a;
		int b;
		for (i = 0; i< M.front()->path_count; i++)
		{
			//at each node found by BFS()
			a = M.front()->BFS_path[i].rPos;
			b = M.front()->BFS_path[i].cPos;

			//make color of path blue
			image.at<Vec3b>(a, b)[0] = 255;		//blue
			image.at<Vec3b>(a, b)[1] = 0;		//green
			image.at<Vec3b>(a, b)[2] = 0;		//red
		}
	}	//end of if success
	
	else cout << "Breadth First Search has failed to find a path." << endl;
	//print out the image regardless to verify
	namedWindow("Display window", WINDOW_AUTOSIZE);// Create a window for display.
	imshow("Display window", image);                   // Show our image inside it.

	namedWindow("Distance Image");
	imshow("Distance Image", distanceImage);
	char name[80];

	for (i = 0; i < num_maps; i++)
	{
		sprintf(name, "Reso window #%d", i + 1);
		namedWindow(name, WINDOW_AUTOSIZE);// Create windows for all maps.
		imshow(name , images.at(i));
	}     
	waitKey(0);                                          // Wait for a keystroke in the window
	
	do
	{
		ArUtil::sleep(150);
	} while (VR.getRunningWithLock());

	Aria::exit(EXIT_SUCCESS);
	::CoUninitialize();
	return 0;
}
