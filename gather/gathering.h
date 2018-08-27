#include <iostream>
#include <opencv2/opencv.hpp>
#define SQR(X) X*X

struct agent{
	cv::Point position;
	bool status;
};
class Gather
{
public:
void disclaimer();
void subset(std::vector<agent> arr, int size, int k, int index, std::vector<cv::Point> &subVec, std::vector<std::vector<cv::Point>> &subCombo);
void getInput(int &nRob,int &faul,int &sMin, int &sMax, int &delay, bool &showHull, int &width, int &height);
void setUpField(cv::Mat &field, std::vector<agent> &robs, int numRobs, int fault, int width, int height);
cv::Mat convexIntersect(cv::Mat &field,  std::vector<std::vector<cv::Point>> &list, bool dLines);
cv::Point calcCenterGrav(cv::Mat hullInter,cv::Mat &field );
void drawBots(std::vector<agent> bots, cv::Mat &robots, cv::Mat &hull, bool draw);
void performGathering();
};