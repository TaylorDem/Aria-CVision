#ifndef _GETLKHAOUTE
#define _GETLKHAOUTE

/*
 This class was created using c++ by Taylor DeMarche.
 Currently in Stage 1.0!

 It creates a struct called node and a class Map which is a
 grid of the nodes and has methods to change points
 to multiple levels of difficulty.
 Node holds:
	 the value and its position in the grid.
	 the previous node's coordinates in a path.
	 distance from start point of BFS
 Contains array Known, which just holds specific nodes
 such as start or end nodes that we would like to remember.

 Grid Assumes bottom row is 0 and leftmost column is 0.
 
 Includes *BFS_path to hold the last known path and 
 path_count to know amount of nodes traversed.

 Now contains CreateImage method which will allow the map
 to be displayed in a color coded fashion

 contains code to build command vector from BFS path.


 current problems:
    Also created a higher resolution constructor that builds
    the higher resolution from an accumulation of knowledge from
    lower resolution.

	Test_BFS when run is immediately exited. something is wrong
	with the creation and display of the images relating to 
	our maps.

	require the higher level as the BFS test is far slower
	than ever hoped.
 */

#include <queue>
#include <vector>
#include <cmath>
#include <opencv2\opencv.hpp>
#include "Command.h"		//holds Command struct
#include <string>
#include <cmath>
using namespace std;
using namespace cv;

enum {
	PASS,
	UNK,
	IMP,
	DIFF,
};

const bool diag = false;

struct Node{
	int value;			//1-4 depending on status passable, unknown, impassible, difficult.
	int rPos;			//row position.
	int cPos;			//column position.
	double distance;
	int prevR;
	int prevC;
	int level;
};

struct Location{
	int rPos;
	int cPos;
	string name;
};

class Map
{
public:	//make everything public to help alleviate errors until further tested.


	Node **m, Position;
	vector<Location> Known;
	vector<Node> BFS_path;
	int rows, columns, path_count;


	//constructors
	Map();
	Map(int,int, int);	//set square size, second parameter is number of known locations.
	Map(int,int,int, int);	//set rectangle with height and width, third is number of known.
	static Map* HL_Map(Map*);	//creates a new map using old one that is half the size.

	//destructors
	~Map();

	//set current position and get current position functions
	void setPos(int, int);
	void getPos(int&, int&);
	

	//set points to PASS
	void set_pass(int,int);

	//set points to UNK
	void set_unk(int,int);
	
	//set points to IMP
	void set_imp(int,int);
	
	//set points to DIFF
	void set_diff(int, int);
	
	//set known Nodes
	int findLoc(string);
	void setLoc(string, int, int);
	bool getLoc(string, int&, int&);

	//Access value of point
	int check(int,int);

	//BFS methods
	bool BFS(Node,Node);		//just returns if found or not for now.
	bool BFS(Node);
	bool BFS(string);
	static bool FindPath(vector<Map*>, int, int, int, int, int);	//not sure how to pass start and end yet...

	//openCV test methods
	static Mat CreateImage(Map*);
	static void SaveImage(Mat, string);

	//Method to build Command vector
	static vector<Command> Guide(Map*);

	//function to look for simpler path, if none found it will run BFS
	static vector<Command> MasterPath(Map*, Node, Node);

};




//CONSTRUCTORS
Map::Map(void)			//Irrelevant constructor I assume, but left in for completeness
{
}

Map::Map(int size,int K, int lev)		//Square Matrix constructor
{
	m = new Node*[size];	//should default to false
	for (int i = 0; i < size; i++)
	{
		m[i] = new Node[size];
	}
	rows = columns = size;

	//for all nodes, make distance = -1 to show "unvisited" during BFS
	for(int i = 0; i<size; i++)
	{
		for(int j = 0; j<size; j++)
		{
			m[i][j].value = PASS;
			m[i][j].distance = -1.0;
			m[i][j].prevC = -1;
			m[i][j].rPos = i;
			m[i][j].cPos = j;
			m[i][j].level = lev;
		}
	}
}


Map::Map(int r,int c,int K, int lev)	//Rectangle sized Map
{
	m = new Node*[r];
	for (int i = 0; i < r; i++)
	{
		m[i] = new Node[c];
	}
	rows = r;
	columns = c;

	//for all nodes, make distance = -1 to show "unvisited" during BFS
	for(int i = 0; i<r; i++)
	{
		for(int j = 0; j<c; j++)
		{
			m[i][j].value = PASS;
			m[i][j].distance = -1;	//unvisited for BFS
			m[i][j].prevC = -1;		//"end point" for retracing path (BFS)
			m[i][j].rPos = i;
			m[i][j].cPos = j;
			m[i][j].level = lev;
		}
	}
}	



/* 
	I should make it so that any known nodes on old map are transcribed into new map
	to do this I can probably just divide the position of old known by 2 and round down
	to get position of new known node that corresponds with old one.
*/

Map* Map::HL_Map(Map *Lower)
{
	Map *higher = new Map((int)Lower->rows / 2, (int)Lower->columns / 2, 2, Lower->m[0][0].level + 1);		//2 known, half size of lower map
	int i, j, v[4];
	bool HAS_PASS, HAS_UNK, HAS_IMP, HAS_DIFF;
	for (i = 0; i < higher->rows; i++)
	{
		for (j = 0; j < higher->columns; j++)
		{
			//fill in higher with accumulated info
			v[0] = Lower->m[i * 2][j * 2].value;
			v[1] = Lower->m[(i * 2) + 1][j * 2].value;
			v[2] = Lower->m[i * 2][(j * 2) + 1].value;
			v[3] = Lower->m[(i * 2) + 1][(j * 2) + 1].value;
			
			//all same, higher = value
			if (v[0] == v[1] && v[1] == v[2] && v[2] == v[3])
			{
				higher->m[i][j].value = v[0];
				continue;
			}

			//check to see whats inside
			HAS_PASS = HAS_UNK = HAS_IMP = HAS_DIFF = false;
			for (int k = 0; k <= 3; k++)
			{
				if (v[k] == PASS) HAS_PASS = true;
				if (v[k] == UNK) HAS_UNK = true;
				if (v[k] == IMP) HAS_IMP = true;
				if (v[k] == DIFF) HAS_DIFF = true;
			}

			//pass + * = diff
			if (HAS_PASS) higher->m[i][j].value = DIFF;
			
			//diff + * = diff
			else if (HAS_DIFF) higher->m[i][j].value = DIFF;
			
			//unk + imp = imp
			else higher->m[i][j].value = IMP;
		}
	}

	return higher;
}



//Destructors
Map::~Map()
{
	Known.clear();
	for (int i = 0; i < rows; i++)
	{
		delete[] m[i];
	}
	delete[] m;
}


void Map::setPos(int a, int b)
{
	Position.rPos = a;
	Position.cPos = b;
}


void Map::getPos(int& x, int& y)
{
	x = Position.rPos;
	y = Position.cPos;
}


//MUTATOR TO PASSABLE
void Map::set_pass(int row,int column)	//set a specific point to 1(passable)
{
	m[row][column].value = PASS;
}


//MUTATOR TO UNKNOWN
void Map::set_unk(int row,int column)	//set a specific point to unknown(2)
{
	m[row][column].value = UNK;
}


//MUTATOR TO IMPASSIBLE
void Map::set_imp(int row,int column)	//set a specific point to impassible(3)
{
	m[row][column].value = IMP;
}


//MUTATOR TO DIFFICULT
void Map::set_diff(int row, int column)
{
	m[row][column].value = DIFF;
}



//ADD NODE TO KNOWN ARRAY METHOD
int Map::findLoc(string n)
{
	int i;

	for (i = 0; i < Known.size(); i++)
	{
		if (!Known.at(i).name.compare(n))	//if found
		{
			return i;
		}
	}
	return -1;
}

void Map::setLoc(string n, int row, int column)
{
	int flag = findLoc(n);
	if (flag == -1)
	{
		Location temp;
		temp.cPos = column;
		temp.rPos = row;
		temp.name = n;
		Known.push_back(temp);
	}
	else
	{
		Known.at(flag).cPos = column;
		Known.at(flag).rPos = row;
	}
}

bool Map::getLoc(string name, int &row, int &column)
{
	int flag = findLoc(name);
	if (flag == -1)
	{
			return false;
	}
	row = Known.at(flag).rPos;
	column = Known.at(flag).cPos;
	return true;
}


//ACCESSOR METHODS
int Map::check(int r,int c)	//get value of point
{
	return m[r][c].value;
}


//BFS METHOD
bool Map::BFS(Node S, Node E)   	//node S is start point; node E is end point.
{
	/*
	1. Place the starting node on the queue.
	2. If the queue is empty, return failure and stop.
	3. If the first element on the queue is a goal node, return success and stop.
	4. Remove and expand the first element from the queue and place all the children at the end of the queue in any order.
	5. Return to step 2.
	*/
	queue<Node> Q;
	int i,j;

	//1
	m[S.rPos][S.cPos].distance = 0.0;
	S.distance = 0.0;
	Q.push(m[S.rPos][S.cPos]);
	path_count = 0;
	while (!Q.empty())
	{	
		//check if end is already found and return before continuing on.
		if (m[E.rPos][E.cPos].distance != -1.0)
		{
			std::cout << "DONE AND EVERYTHING IS HAPPY" << endl;
			BFS_path.clear();	//clear path
			//add all prev nodes to the *path to trace back
			BFS_path.push_back(m[E.rPos][E.cPos]);
			i = 0;
			while (BFS_path[i].prevC != -1)	//while there is still a way back.
			{
				BFS_path.push_back(m[BFS_path[i].prevR][BFS_path[i].prevC]);	//next in array is prev node.
				i++;
			}
			BFS_path.push_back(m[S.rPos][S.cPos]);
			path_count = BFS_path.size();
			return true; //success
		}

		//cout << Q.front().rPos << ", " << Q.front().cPos << endl;
		int a = -2;
		for(i=(Q.front().rPos - 1);i<=(Q.front().rPos + 1);i++)							
		{	
			a++;
			int b = -2;
			if (i<0 || i>=rows) continue;	//stay within border of image
			for(j=(Q.front().cPos - 1);j<=(Q.front().cPos + 1);j++)		//for 3 by 3 grid around node.
			{
				b++;
				if (j<0 || j>=columns) continue;							//stay within border of image
				//if((i == Q.front().rPos) && (j == Q.front().cPos)) continue;	//exclude center node in grid.
				double DA = 1.0;
				if (a != 0 && b != 0 && diag) DA = sqrt(2);	//if diagonal, distance ~~ rad(2)
				if ((a != 0 && b != 0) && !diag) continue;	//ignore diagonals when the bool is false
				
				if (m[i][j].value == PASS && (m[i][j].distance > Q.front().distance + DA || m[i][j].distance == -1))			//push all other nodes if passable and distance = -1.
				{
					m[i][j].distance = Q.front().distance + DA;	//set distance based on current position.
					m[i][j].prevR = Q.front().rPos;				//set current position to "prevR/C" to back track from new node.
					m[i][j].prevC = Q.front().cPos;
					Q.push(m[i][j]);
				} 
			}
		}
		Q.pop();	//get rid of center node from Queue.

	//5
	}

	//only will return false if queue empties but end node wasn't given a distance.
	return false;
}


bool Map::BFS(Node E)   	//node S is start point; node E is end point.
{
	return BFS(Position, E);
}


bool Map::BFS(string destination)
{
	int row;
	int column;
	if (getLoc(destination, row, column))
	{
		return BFS(m[row][column]);
	}
	return false;
}




//modify old BFS to not use diagonals and exit when end is found.



bool Map::FindPath(vector<Map*> maps, int start_r, int start_c, int end_r, int end_c, int level)
{
	queue<Node> Q;
	int i, j;
	bool test;

	if (level > maps.size() || level == 0)
	{
		std::cout << "wrong level input";
		return false;
	}


	//1
	maps.at(level)->m[start_r][start_c].distance = 0.0;
	Q.push(maps.at(level)->m[start_r][start_c]);
	maps.at(level)->path_count = 0;
	while (!Q.empty())
	{

		//cout << Q.front().rPos << ", " << Q.front().cPos << endl;
		int a = -2;
		for (i = (Q.front().rPos - 1); i <= (Q.front().rPos + 1); i++)
		{
			a++;
			int b = -2;
			if (i<0 || i >= maps.at(level)->rows) continue;	//stay within border of image
			for (j = (Q.front().cPos - 1); j <= (Q.front().cPos + 1); j++)		//for 3 by 3 grid around node.
			{
				b++;
				if (j<0 || j >= maps.at(level)->columns) continue;							//stay within border of image
				//if((i == Q.front().rPos) && (j == Q.front().cPos)) continue;	//exclude center node in grid.
				double DA = 1.0;
				if ((a != 0 && b != 0) && diag) DA = sqrt(2);	//if diagonal, distance ~~ rad(2)
				if ((a != 0 && b != 0) && !diag) continue;	//ignore diagonals when the bool is false

				if (maps.at(level)->m[i][j].value == PASS && (maps.at(level)->m[i][j].distance > Q.front().distance + DA || maps.at(level)->m[i][j].distance == -1))			//push all other nodes if passable and distance = -1.
				{
					maps.at(level)->m[i][j].distance = Q.front().distance + DA;	//set distance based on current position.
					maps.at(level)->m[i][j].prevR = Q.front().rPos;				//set current position to "prevR/C" to back track from new node.
					maps.at(level)->m[i][j].prevC = Q.front().cPos;
					Q.push(maps.at(level)->m[i][j]);
				}
				if (level != 1 &&(maps.at(level)->m[i][j].value == DIFF && (maps.at(level)->m[i][j].distance > Q.front().distance + DA || maps.at(level)->m[i][j].distance == -1)))
				{
					//something like this I imagine...
					test = FindPath(maps, (int) i*2, (int) j*2, (int)(i+1)*2, (int)(j+1)*2, level-1);
					if (test)
					{
						maps.at(level)->m[i][j].distance = Q.front().distance + DA;	//set distance based on current position.
						maps.at(level)->m[i][j].prevR = Q.front().rPos;				//set current position to "prevR/C" to back track from new node.
						maps.at(level)->m[i][j].prevC = Q.front().cPos;
						Q.push(maps.at(level)->m[i][j]);
					}
				}
			}
		}
		Q.pop();	//get rid of center node from Queue.

		//5
	}
	//if (Q.front().rPos == E.rPos && Q.front().cPos == E.cPos)
	if (maps.at(level)->m[end_r][end_c].distance != -1.0)
	{
		std::cout << "DONE AND EVERYTHING IS HAPPY" << endl;
		maps.at(level)->BFS_path.clear();	//clear path
		//add all prev nodes to the *path to trace back
		maps.at(level)->BFS_path.push_back(maps.back()->m[end_r][end_c]);
		i = 0;
		while (maps.at(level)->BFS_path[i].prevC != -1)	//while there is still a way back.
		{
			maps.at(level)->BFS_path.push_back(maps.at(level)->m[maps.at(level)->BFS_path[i].prevR][maps.at(level)->BFS_path[i].prevC]);	//next in array is prev node.
			i++;
		}
		maps.at(level)->path_count = maps.at(level)->BFS_path.size();
		return true; //success
	}
	else return false;
}


Mat Map::CreateImage(Map *M)
{
	Mat image = Mat::zeros(M->rows, M->columns, CV_8UC3);
	Vec3b pixel;
	for (int i = 0; i < M->rows; i++)
	{
		for (int j = 0; j < M->columns; j++)
		{
			pixel.val[0] = pixel.val[1] = pixel.val[2] = 0;
			if (M->m[i][j].value == PASS)	//pass is green
			{
				pixel.val[1] = 255;
			}
			if (M->m[i][j].value == UNK)		//unknown set to black
			{
				pixel.val[0] = pixel.val[1] = pixel.val[2] = 255;
			}
			if (M->m[i][j].value == IMP)		//imp set to red
			{
				pixel.val[2] = 255;
			}
			if (M->m[i][j].value == DIFF)	//diff set to blue
			{
				pixel.val[0] = 255;  
			}
			image.at<Vec3b>(i, j) = pixel;
		}
	}
	return image;
}


void Map::SaveImage(Mat image, string filename)
{
	imwrite(filename, image);
}



/*
this method uses the path vector to build a list of commands to
be fed to the robot. i.e. up x2, left x4.
*/

vector<Command> Map::Guide(Map* p)
{
	vector<Command> guide;		//create command list
	bool up, down, left, right;		//holds directions
	int i;
	Command temp;	//forward =0, right =1, left =2, backward =3 with how pauls stuff works... i believe

	for (i = (p->path_count-1); i >= 0; i--)		//going backwards because we start path at end point
	{
		up = down = left = right = false;

		//what if i-1 == NULL???
		if (i == 0) continue;	//just ignore for now... not sure how to handle this
		if (p->BFS_path[i].cPos < p->BFS_path[i - 1].cPos) right = true;	//each direction should be correct
		if (p->BFS_path[i].cPos > p->BFS_path[i - 1].cPos) left = true;	
		if (p->BFS_path[i].rPos < p->BFS_path[i - 1].rPos) up = true;		
		if (p->BFS_path[i].rPos > p->BFS_path[i - 1].rPos) down = true;		

		/* I handled direction this way to possibly make adding diagonals easier in the future. */


		//now assign direction and single step to guide commands
		if (up)
		{
			temp.heading = 0;
			temp.steps = 1;
			guide.push_back(temp);
		}
		if (left)
		{
			temp.heading = 2;
			temp.steps = 1;
			guide.push_back(temp);
		}
		if (right)
		{
			temp.heading = 1;
			temp.steps = 1;
			guide.push_back(temp);
		}
		if (down)
		{
			temp.heading = 3;
			temp.steps = 1;
			guide.push_back(temp);
		}
	}

	cout << "BEFORE GUIDE STEPS: " << guide.size() << endl;

	//now to condense the commands to include less commands for consecutive calls
	int gsize = guide.size();

	//for (i = 0; i < (gsize - 1); i++)
	for (i = (gsize - 2); i >= 0; i--)
	{
		//while (guide.at(i).heading == guide.at(i + 1).heading)		//if next is the same command direction
		if (guide.at(i).heading == guide.at(i + 1).heading)
		{
			guide.at(i).steps = guide.at(i + 1).steps;
			guide.at(i).steps++;
			guide.erase(guide.begin() + i + 1);		//erase command we already accounted for by adding a step to last command
		}
		//check for diagonals amongst the path here.

	}

	cout << "GUIDE STEPS: " << guide.size() << endl;

	return guide;
}


vector<Command> Map::MasterPath(Map* A, Node S, Node E)
{
	int i;
	vector<Command> path;
	bool Bres = true;

	if (diag)
	{
		vector<Node> Bres_list;
		double error = 0;
		double delError = abs((E.rPos - S.rPos) / (E.cPos - S.cPos));
		int j = S.rPos;
		i = S.cPos;
		while (i != E.cPos)		//while not on endpoint 
		{
			if (A->m[j][i].value != PASS) Bres = false;
			if (A->m[j][i].value == PASS) Bres_list.push_back(A->m[j][i]);
			error += delError;
			while (error >= 0.5)
			{
				if (A->m[j][i].value != PASS) Bres = false;
				if (A->m[j][i].value == PASS) Bres_list.push_back(A->m[j][i]);
				if (E.rPos - S.rPos > 0) j++;
				if (E.rPos - S.rPos < 0) j--;
				error--;
			}
			if (E.cPos - S.cPos > 0) i++;
			if (E.cPos - S.cPos < 0) i--;
		}

		if (Bres) //diagonal path found
		{
			bool up = false, down = false, left = false, right = false;
			Command temp;
			for (i = 0; i < Bres_list.size(); i++)
			{
				if (Bres_list.at(i).cPos < Bres_list.at(i + 1).cPos) right = true;
				if (Bres_list.at(i).cPos > Bres_list.at(i + 1).cPos) left = true;
				if (Bres_list.at(i).rPos < Bres_list.at(i + 1).rPos) up = true;
				if (Bres_list.at(i).rPos > Bres_list.at(i + 1).rPos) down = true;

				/* 
				A big problem is that the Command struct isn't designed to have diagonals.
				We need to fix that pronto.
				*/

				if (right && up)	
				{
					temp.heading = 0;	//up then right commands?
					temp.steps = 1;
					path.push_back(temp);
					temp.heading = 1;
					temp.steps = 1;
					path.push_back(temp);
				}
				else if (right && down)
				{
					temp.heading = 3;	//down then right commands?
					temp.steps = 1;
					path.push_back(temp);
					temp.heading = 1;
					temp.steps = 1;
					path.push_back(temp);
				}
				else if (left && up)
				{
					temp.heading = 0;	//up then left commands?
					temp.steps = 1;
					path.push_back(temp);
					temp.heading = 2;
					temp.steps = 1;
					path.push_back(temp);
				}
				else if (left && down)
				{
					temp.heading = 3;	//down then left commands?
					temp.steps = 1;
					path.push_back(temp);
					temp.heading = 2;
					temp.steps = 1;
					path.push_back(temp);
				}
				//singular direction cases
				else if (up)
				{
					temp.heading = 0;
					temp.steps = 1;
					path.push_back(temp);
				}
				else if (down)
				{
					temp.heading = 3;
					temp.steps = 1;
					path.push_back(temp);
				}
				else if (left)
				{
					temp.heading = 2;
					temp.steps = 1;
					path.push_back(temp);
				}
				else if (right)
				{
					temp.heading = 1;
					temp.steps = 1;
					path.push_back(temp);
				}
			}//end of for to build list of commands

			for (i = 0; i < path.size(); i++)
			{
				while (path.at(i).heading == path.at(i + 1).heading)		//if next is the same command direction
				{
					path.at(i).steps++;
					path.erase(path.begin() + i);		//erase command we already accounted for by adding a step to last command
				}
			}
			return path;
		}
	}


	//if that fails or diagonals are set off, check for direct "simple" path
	bool firstD = true, secondD = true;
	bool startUD = true;

	if (S.rPos <= E.rPos)
		for (i = S.rPos; i <= E.rPos; i++)
			if (A->m[i][S.cPos].value != PASS) firstD = false;

	if (S.rPos >= E.rPos)
		for (i = S.rPos; i >= E.rPos; i--)
			if (A->m[i][S.cPos].value != PASS) firstD = false;		//does up/down first then left/right. 

	
	if (S.cPos <= E.cPos && firstD)
		for (i = S.cPos; i <= E.cPos; i++)
			if (A->m[E.rPos][i].value != PASS) secondD = false;	

	if (S.cPos >= E.cPos && firstD)
		for (i = S.cPos; i >= E.cPos; i--)
			if (A->m[E.rPos][i].value != PASS) secondD = false;	

	if (!firstD || !secondD)	//meaning one path at least was stopped.
	{
		startUD = false;	//acknowledge that the up/down path didn't work for later
		//same thing (reset), but starting with left/right before up/down
		firstD = secondD = true;
		if (S.cPos <= E.cPos)
			for (i = S.cPos; i <= E.cPos; i++)
				if (A->m[S.rPos][i].value != PASS) firstD = false;

		if (S.cPos >= E.cPos)
			for (i = S.cPos; i >= E.cPos; i--)
				if (A->m[S.rPos][i].value != PASS) firstD = false; 


		if (S.rPos <= E.rPos && firstD)
			for (i = S.rPos; i <= E.rPos; i++)
				if (A->m[i][E.cPos].value != PASS) secondD = false;

		if (S.rPos >= E.rPos && firstD)
			for (i = S.rPos; i >= E.rPos; i--)
				if (A->m[i][E.cPos].value != PASS) secondD = false;
	}

	
	if (firstD && secondD)		//nothing was impassible
	{
		//create command vector directly for the simple path
		if (startUD)
		{
			path.at(0).steps = (int)abs(E.rPos - S.rPos);
			if (S.rPos < E.rPos)
				path.at(0).heading = 0; //up
			if (S.rPos > E.rPos)
				path.at(0).heading = 3; //down

			path.at(1).steps = (int)abs(E.cPos - S.cPos);
			if (S.cPos < E.cPos)
				path.at(1).heading = 2; //left
			if (S.cPos > E.cPos)
				path.at(1).heading = 1; //right
		}
		if (!startUD)
		{
			path.at(0).steps = (int)abs(E.cPos - S.cPos);
			if (S.cPos < E.cPos)
				path.at(0).heading = 2; //left
			if (S.cPos > E.cPos)
				path.at(0).heading = 1; //right

			path.at(1).steps = (int)abs(E.rPos - S.rPos);
			if (S.rPos < E.rPos)
				path.at(1).heading = 2; //up
			if (S.rPos > E.rPos)
				path.at(1).heading = 1; //down
		}

		return path;
	}	//end of "simple path"

	//if that fails, run BFS
	bool bfs_check = A->BFS(S, E);
	if (bfs_check) return Guide(A);
	else
	{
		cout << "Couldn't Find A Single Path." << endl;
		return path;
	}
}
#endif