#include <Aria.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
	Aria::init();

	ArArgumentParser parser(&argc, argv);

	parser.loadDefaultArguments();

	ArRobot robot;

	ArRobotConnector robotConnector(&parser, &robot);

	if (!robotConnector.connectRobot())
	{
		if (!parser.checkHelpAndWarnUnparsed())
		{
			ArLog::log(ArLog::Terse, "Unable to connect.");
		}
		else
		{
			ArLog::log(ArLog::Terse, "Could not connect.");
			Aria::logOptions();
			Aria::exit(1);
			return EXIT_FAILURE;
		}
	}

	if (!robot.isConnected())
	{
		ArLog::log(ArLog::Terse, "Connection to robot was interrupted.");
	}

	if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		Aria::exit(1);
		return EXIT_FAILURE;
	}

	ArKeyHandler keyHandler;
	Aria::setKeyHandler(&keyHandler);
	robot.attachKeyHandler(&keyHandler);
	printf("Press esc to exit.");

	robot.runAsync(true);

	cout << "Moving 250 mm" << endl;

	robot.lock();
	robot.move(250);
	robot.unlock();
	ArUtil::sleep(2000);

	cout << "Turning +90 degrees" << endl;

	robot.lock();
	robot.setDeltaHeading(90);
	robot.unlock();
	ArUtil::sleep(3000);

	robot.waitForRunExit();

	Aria::exit(0);
	return EXIT_SUCCESS;
}
