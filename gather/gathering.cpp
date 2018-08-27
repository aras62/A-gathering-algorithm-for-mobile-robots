
#include "gathering.h"
#include <ctime>
using namespace cv;
using namespace std;

void Gather::disclaimer() 

{
	cout << "###########################################################################\n"
		 << "##                               Gather Bots                             ##\n"
		 << "## A Gathering robot algorithm based on the paper 'Fault-Tolerant        ##\n"
		 << "## Gathering Algorithms for Autonomous Mobile Robots' by N. Agmon and    ##\n"
		 << "## D. Peleg.                                                             ##\n"
		 << "## Designed by: Amir Rasouli                                             ##\n"
		 << "###########################################################################\n\n\n";
}
void Gather::subset(std::vector<agent> arr, int size, int k, int index, std::vector<cv::Point> &subVec, std::vector<std::vector<cv::Point>> &subCombo){
 
	if (k == 0)
	{
		subCombo.push_back(subVec);
		return;
	}
	
	for (int i = index; i < size; i++)
	{
		subVec.push_back(arr[i].position);
		subset(arr, size, k-1, i+1, subVec, subCombo);
		subVec.pop_back();
	}
};   
void Gather::getInput(int &nRob,int &faul, int &sMin, int &sMax, int &delay, bool &showHull, int &width, int &height){
    string m;
	int numRob;
	do{
		cout << "Input the number of robots:  ";
		cin >> nRob;
		if (cin.fail())
		{
			cout << "Wrong Input, please enter an Integer \n";
			cin.clear();
			cin.ignore();
		}else{
			break;
		}
	}while(true);

	faul = (nRob-1)/3;
	cout << "Select the number of Faulty Robots?(y/n):   ";
	cin >> m;
	if (m == "y" || m =="Y")
	{
		do{
		cout << "Input number of Faulty Robots(No more than " << faul <<" ):  " ;
		cin >> numRob;
		
		if (cin.fail() || numRob > faul)
		{
			cout << "Wrong Input, please enter a valid Integer \n";
			cin.clear();
			cin.ignore();
		}else{
			faul = numRob;
			break;
		}
	}while(true);
		

	}else
	{
		cout << "Maximum Number of Faulty Robots:  " << faul <<"\n" ;
	}
	cout << "Show convex Hulls? (y/n):  ";
		cin >> m;
		if (m == "y" || m == "Y")
		{
			showHull = true;
		}
	cout << "Use Default Parameters ? (y/n):   ";
	cin >> m;
	
	//********Setup***********
	if (m == "n" || m =="N")
	{
	  cout<<" ************************ Parameters ************************\n"  
		   <<"FiledWidth and FieldHeight: size of the environment (in pixels)" 
		   << "Smin: minimum distance traveled by each robot in each cycle (in pixels).\n"
		   << "Smax: maximum distance traveled by each robot in each cycle (in pixels).\n"
		   << "delay: delay between consequtive frames(in milliseconds)\n"
		  <<" ***********************************************************\n\n\n" ;
		
	  do{
		cout << "FieldWidth(greater than 50):  ";
		cin >> numRob;
		if (cin.fail()|| numRob<50)
		{
			cout << "Wrong Input, please enter a valid integer \n";
			cin.clear();
			cin.ignore();
		}else{
			width = numRob;
			break;
		}
			}while(true);

		do{
		cout << "FieldLength(greater than 50):  ";
		cin >> numRob;
		if (cin.fail()|| numRob <50)
		{
			cout << "Wrong Input, please enter a valid integer \n";
			cin.clear();
			cin.ignore();
		}else{
			height = numRob;
			break;
		}
		}while(true);

		
			do{
		cout << "Smin( >1 ):  ";
		cin >> numRob;
		if (cin.fail()|| numRob <1)
		{
			cout << "Wrong Input, please enter a valid integer \n";
			cin.clear();
			cin.ignore();
		}else{
			sMin = numRob;
			break;
		}
		}while(true);
				do{
		cout << "Smax( >"<< sMin <<" ):  ";
		cin >> numRob;
		if (cin.fail()|| numRob <= sMin)
		{
			cout << "Wrong Input, please enter a valid integer \n";
			cin.clear();
			cin.ignore();
		}else{
			sMax = numRob;
			break;
		}
		}while(true);
		do{
		cout << "Delay:  ";
		cin >> numRob;
		if (cin.fail())
		{
			cout << "Wrong Input, please enter an integer \n";
			cin.clear();
			cin.ignore();
		}else{
			delay = numRob;
			break;
		}
		}while(true);
	}
}
void Gather::setUpField(Mat &field, vector<agent> &robs, int numRobs, int fault, int width, int height){
int count = 0;
Scalar color = Scalar(0,0,0);
for (int i = 0; i < numRobs; i++)
	{
		int x = rand()%width;
		int y = rand()%height;
		agent robot;
		robot.position = Point(x,y);
		if (rand() % 2 == 0 && count < fault)
		{
			robot.status = false;
			count++;
			color = Scalar(0,0,255);
		}else
		{
			robot.status = true;
			color = Scalar(255,0,0);
		}

		robs.push_back(robot);
		
		circle(field, Point(x,y), 12,color, -1);
		putText(field,std::to_string((long long)i+1), Point(x-10,y+5),CV_FONT_HERSHEY_COMPLEX, 0.5,Scalar(255,255,255),1);
	}

}
Mat Gather::convexIntersect(Mat &field, vector<vector<Point>> &list, bool dLines){
Mat pAnd = Mat(field.rows,field.cols, CV_8UC3,Scalar(0,255,0));
vector<Point> hull;
for (int j = 0; j < list.size(); j++)
	{
		Mat p = Mat(field.rows,field.cols, CV_8UC3,Scalar(0,0,0));
		convexHull(Mat(list[j]), hull, true, false);
		int hullCount = (int)hull.size();
		Point* pt0 = &hull[0];
		fillConvexPoly(p, pt0, (int)hull.size(),Scalar(0,255,0),8);
		bitwise_and(pAnd,p,pAnd);
		Point point0= hull[hullCount-1];
		if(dLines == true)
		{
			
			for( int i = 0; i < hullCount; i++ )
			{
				Point pt = hull[i];
				line(field, point0, pt, Scalar(0, 255, 255), 1, CV_AA);
				point0 = pt;
			}
		}
	}
	
	return pAnd;
}
Point Gather::calcCenterGrav(Mat hullInter,Mat &field ){
	Mat cGrav;
	Point cenGrav;
	cvtColor(hullInter, cGrav, CV_RGB2GRAY);
	int x = 0, y = 0, count = 0;
	for(int yc = 0; yc < cGrav.rows; yc++){
		for(int xc =0; xc < cGrav.cols; xc++){
			if (cGrav.at<uchar>(yc,xc) > 0)
			{
				x+=xc;
				y+=yc;
				count++;
			}
		}
	}
	if (count == 0)
	{ 
		cenGrav = Point((int)(cGrav.cols/2), (int)(cGrav.rows/2));}
	else{
	 cenGrav = Point((int)(x/count), (int)(y/count));
	}
	circle(field, cenGrav, 4,Scalar(255, 0, 255), -1);
	return cenGrav;
}
void Gather::drawBots(vector<agent> bots, Mat &robots, Mat &hull, bool draw)
{
	Scalar col = Scalar(0,0,0);
	for (int i = 0; i < bots.size(); i++)
	{
		if (bots[i].status == false)
			{
			   col = Scalar(50,50,255);
			}else
		{
			col = Scalar(255,50,20);
			}
	        circle(robots, bots[i].position, 12,col, -1);
			putText(robots,std::to_string((long long)i+1), Point(bots[i].position.x-10,bots[i].position.y+5),CV_FONT_HERSHEY_COMPLEX, 0.5,Scalar(255,255,255),1);
			if(draw == true)
			{
			   circle(hull, bots[i].position, 12,col, -1);
			   putText(hull,std::to_string((long long)i+1), Point(bots[i].position.x-10,bots[i].position.y+5),CV_FONT_HERSHEY_COMPLEX, 0.5,Scalar(255,255,255),1);
			}
	}

}
void Gather::performGathering()
{
	disclaimer();
	const double PI = 3.14159265;
	Mat final;
	vector<agent> robots;
	vector<Point> junk1;
	vector<vector<Point>> subList1; 
	int Smin = 5;
	int Smax = 10;
	int delay = 100;
	int width = 400;
	int height = 400;
	bool drawLines = true;// false;
	int numRobot = 4; // greater than 3
	int faulty = 1; // should be less than  numRobot/3 
	int cnt = 0;
	Scalar s = Scalar(200,200,200);
	//getInput(numRobot, faulty, Smin, Smax,delay,drawLines, width, height); // if want to get input from the user uncomment it
	
	
	Mat pImg = Mat(height, width, CV_8UC3,s);
	Mat pHull = Mat(height, width, CV_8UC3,s);
	Mat intersect = Mat(pImg.rows, pImg.cols,pImg.type(),s);
	setUpField(pImg, robots,numRobot, faulty, width, height);
		std::vector<int> params;
		params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		params.push_back(100);
		while(true)
		{
	   // clock_t start = clock(); // for timing measure
		Mat robPos= Mat(pImg.rows, pImg.cols, pImg.type(), s);
	    Mat pHull= Mat(pImg.rows, pImg.cols, pImg.type(), s);
		vector<Point> junk;
		vector<vector<Point>> subList; 
		subset(robots, robots.size(), robots.size()-faulty, 0,junk, subList);
		intersect = convexIntersect(pHull,subList, drawLines);

		//	cout << (double)(clock()-start)/CLOCKS_PER_SEC << endl;
		Point center = calcCenterGrav(intersect, pHull);
		circle(robPos, center, 6,Scalar(255, 0, 255), -1);
		drawBots(robots,robPos,pHull, drawLines);
		if(drawLines == true)
		{
			pHull+=intersect;
			imshow("convexHulls", pHull);
			waitKey(delay);
		
		std::stringstream tmp1;
		tmp1 << "../images/" <<"hull_"<< cnt <<".png";
	
		//imwrite(tmp1.str(), pHull,params);
		}
		imshow("robots", robPos);
		waitKey(delay);
		//waitKey(0);
		std::stringstream tmp;
		tmp << "../images/" <<cnt<<".png";
		//imwrite(tmp.str(), robPos,params);
		cnt++;	
				
		bool done = true;
		for(int i = 0 ; i < robots.size(); i++)
		{

			if(abs((double)robots[i].position.x - (double)center.x) < 5 && abs((double)robots[i].position.y - (double)center.y) <5)
			{
				continue;
			}
			if (robots[i].status == true)
			{
				done = false;
			}
			if (robots[i].status == false)
			{
				int x = rand() % width;
				int y = rand() % height;
				robots[i].position = Point(x,y);
			}else{
				Point2d dispLace = Point2d(robots[i].position.x - center.x, robots[i].position.y - center.y);
				int pgDist= sqrt(SQR(dispLace.y) + SQR(dispLace.x));
				double angle= atan2((double) robots[i].position.y - center.y, (double) robots[i].position.x - center.x );
				int distance = Smin + rand()%Smax;
				distance = (distance > pgDist) ? pgDist : distance;
				int xDist = cos(angle)*distance;
				int yDist = sin(angle)*distance;
				robots[i].position -= Point(xDist,yDist);
			}
		
		}
	if (done ==true)
		{
			break;
		}
	}
	
	waitKey(0);
	
}