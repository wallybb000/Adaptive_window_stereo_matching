// Adaptive_window_stereo_matching.cpp: 主要專案檔。

#include "stdafx.h"
#include <iostream>


using namespace System;
using namespace cv; 
using namespace std;

#define ERROR_LEVEL 255
#define INIT_LEVEL 254

#define SUPPERT_LENGTH 30
#define REFINE_BLOCK_LENGTH 30

#define DISPARITY 15

#define WINDOW_THRESHOLD 10
#define SUBTRACT_THRESHOLD 10

#define SMax 250
#define SMin 30


void disparityRefinement(Mat & DisparityMap, Mat& ColorImage)
{

	for (int X = 0; X < DisparityMap.cols;X++)
	for (int Y = 0; Y < DisparityMap.rows; Y++)
	{

		int diff = 2000000;
		Point2i MinDiffPoint(0, 0);
		for (int Wx = -1 * REFINE_BLOCK_LENGTH / 2; Wx <= REFINE_BLOCK_LENGTH / 2;Wx++)
		for (int Wy = -1 * REFINE_BLOCK_LENGTH / 2; Wy <= REFINE_BLOCK_LENGTH / 2;Wy++)
		{
			int Ix = X + Wx;
			int Iy = Y + Wy;
			if (Ix < 0 || Ix >= DisparityMap.cols || Iy < 0 || Iy >= DisparityMap.rows)
				continue;
			if (Wx == 0 && Wy == 0)
				continue;

			Vec3b center = ColorImage.at<Vec3b>(Y,X);
			Vec3b neighbor = ColorImage.at<Vec3b>(Iy, Ix);

			double dL = Math::Pow(center[0] - neighbor[0], 2);
			double da = Math::Pow(center[1] - neighbor[1], 2);
			double db = Math::Pow(center[2] - neighbor[2], 2);
			double distance = Math::Sqrt(dL + da + db);
			if (distance < diff)
			{
				diff = distance;
				MinDiffPoint.x = Ix;
				MinDiffPoint.y = Iy;
			}
		}
		uchar a = DisparityMap.at<uchar>(Y, X);
		uchar b = DisparityMap.at<uchar>(MinDiffPoint);
		DisparityMap.at<uchar>(Y, X) = b;


	}

}


bool setDisparityToWindowRegion(Mat & DisparityMap, int Disparity, std::vector<Point2i> & Window)
{
	int X = Window[0].x;
	int Y = Window[0].y;
	if (X < 0 || X >= DisparityMap.cols || Y < 0 || Y >= DisparityMap.rows)return false;
	//DisparityMap.at<uchar>(Y, X)=Disparity;



	
	for (int index = 1; index < Window.size(); index++)
	{
		int Temp = DisparityMap.at<uchar>(Y + Window[index].y, X + Window[index].x);

		if (Temp == INIT_LEVEL) DisparityMap.at<uchar>(Y + Window[index].y, X + Window[index].x) = Disparity;
		else if (abs(Disparity - Temp)>1)
			DisparityMap.at<uchar>(Y + Window[index].y, X + Window[index].x) = ERROR_LEVEL;
	}
	



};


int WindowSubtraction(Mat &Left, Mat & Right, int Disparity, std::vector<Point2i> & Window)
{
	int X = Window[0].x;
	int Y = Window[0].y;
	if (X < 0 || X >= Left.cols || Y < 0 || Y >= Left.rows)return -1;

	int Coin = 0;

	for (int index = 1; index < Window.size(); index++)
	{
		Vec3b LeftPixel = Left.at<Vec3b>(Y + Window[index].y, X + Window[index].x);
		Vec3b RightPixel = Right.at<Vec3b>(Y + Window[index].y, X + Window[index].x + Disparity);

		double dL = Math::Pow(LeftPixel[0] - RightPixel[0], 2);
		double da = Math::Pow(LeftPixel[1] - RightPixel[1], 2);
		double db = Math::Pow(LeftPixel[2] - RightPixel[2], 2);
		
		if (Math::Sqrt(dL + da + db)<SUBTRACT_THRESHOLD)
			Coin++;
	}
	return Coin;
}



bool getAdaptiveWindow(Mat & InImage, Point Position, std::vector<Point2i> & Window)
{
	if (Position.x < 0 || Position.x >= InImage.cols || Position.y < 0 || Position.y >= InImage.rows)
		return false;

	Window.clear();

	Window.push_back(Point2i(Position.x, Position.y));

	for (int Wx = -1* SUPPERT_LENGTH / 2; Wx <= SUPPERT_LENGTH / 2; Wx++)
	for (int Wy = -1 * SUPPERT_LENGTH / 2; Wy <= SUPPERT_LENGTH / 2; Wy++)
	{
		int Ix = Position.x + Wx;
		int Iy = Position.y + Wy;

		if (Ix < 0 || Ix >= InImage.cols || Iy < 0 || Iy >= InImage.rows)
			continue;

		Vec3b center = InImage.at<Vec3b>(Position.y, Position.x);
		Vec3b neighbor = InImage.at<Vec3b>(Iy, Ix);

		double dL = Math::Pow(center[0] - neighbor[0], 2);
		double da = Math::Pow(center[1] - neighbor[1], 2);
		double db = Math::Pow(center[2] - neighbor[2], 2);

		if (Math::Sqrt(dL + da + db) < WINDOW_THRESHOLD)
			Window.push_back(Point2i(Wx, Wy));
		
	}
	
	return true;

};


int main(array<System::String ^> ^args)
{


	Mat Right = imread("C:\\Users\\wally\\Desktop\\tsukuba\\scene1.row3.col3.jpg", 1);
	Mat Left = imread("C:\\Users\\wally\\Desktop\\tsukuba\\scene1.row3.col2.jpg");
	Mat DisparityMap(Right.size(), CV_8UC1);

	std::vector<Point2i> Window;

	cvtColor(Right, Right,CV_RGB2Lab);
	cvtColor(Left, Left, CV_RGB2Lab);
	DisparityMap.setTo(INIT_LEVEL);
	for (int X = 0; X < DisparityMap.cols;X++)
	for (int Y = 0; Y < DisparityMap.rows; Y++)
	{
		if (DisparityMap.at<uchar>(Y, X) != INIT_LEVEL)continue;

		
		int MaxDisparity=0;
		int MaxTemp = -1000;

		getAdaptiveWindow(Left, Point2d(X, Y), Window);
		cout << "window:"<< Window.size() - 1;
		for (int D = -1*DISPARITY; D <= DISPARITY; D++)
		{
			int Value = WindowSubtraction(Left, Right, D, Window);
			if (Value> MaxTemp)
			{
				MaxTemp = Value;
				MaxDisparity = abs(D);
			}
		}
		cout << "   " << MaxTemp;
		cout << "   " << MaxDisparity << endl;
		setDisparityToWindowRegion(DisparityMap, MaxDisparity, Window);
	}
	


	disparityRefinement(DisparityMap, Left);

	//medianBlur(DisparityMap, DisparityMap, 3);

	int Max = -1000;
	int Min = 20000;

	for (int X = 0; X < DisparityMap.cols; X++)
	for (int Y = 0; Y < DisparityMap.rows; Y++)
	{

		uchar temp = DisparityMap.at<uchar>(Y, X);
		if  (temp== ERROR_LEVEL)continue;

		if (temp > Max)Max = temp;
		if (temp < Min)Min = temp;
	}


	for (int X = 0; X < DisparityMap.cols; X++)
	for (int Y = 0; Y < DisparityMap.rows; Y++)
	{

		uchar temp = DisparityMap.at<uchar>(Y, X);
		if (temp == ERROR_LEVEL){
			
			DisparityMap.at<uchar>(Y, X) = 0;
			
		}
		else if (temp == INIT_LEVEL)
		{
			cout << "errrrrrrrrr";
		}
		else
		{
			double dtemp = (double)temp;
			dtemp = dtemp / (Max - Min)*(SMax - SMin) + SMin;
			DisparityMap.at<uchar>(Y, X) =static_cast<uchar>(dtemp);
		}
	}
	
	imshow("see", DisparityMap);

	waitKey(0);
    Console::WriteLine(L"Hello World");
    return 0;
}
