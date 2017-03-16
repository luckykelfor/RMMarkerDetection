// import the necessary packages
//import time
//import cv2
//import numpy as np
//
////Difference Variable
//minDiff = 10000
//minSquareArea = 5000
//match = -1
//
////Frame width & Height
//w = 640
//h = 480
#include<opencv.hpp>
using namespace cv;
#define REF_IMAGE_NUM 3
char ReferenceImagesPaths[REF_IMAGE_NUM][20] = { "D:/ArrowUp.jpg", "D:/RM.jpg", "D:/Circle.jpg" };
char ReferenceImagesTitle[REF_IMAGE_NUM][20] = { "ArrowU", "RM", "Circle" };
const int minSquareArea = 5000;
const int minDiff = 10000;
const int match = -1;
const int w = 640;
const int h = 480;
extern double fx;// = 1151.1; // / 1194.61;
extern double fy;// = 1150.8;///;// 1196.98;
extern double u0;// = 627.29108;// 233.0957;// 302.9064;// 634.075;
extern double v0;// = 369.8137;// 300.378;// 235.2960;// 504.842;
double max(double a, double b)
{
	return a > b ? a : b;
}
double min(double a, double b)
{
	return a < b ? a : b;
}


//class Symbol
//{
//public:
//	char *name;
//	Mat image;
//};
//Symbol referenceImages[REF_IMAGE_NUM];
//
//
//
//void readRefImages()
//{
//	for (int i = 0; i < REF_IMAGE_NUM; i++)
//	{
//		::referenceImages[i].image = imread(ReferenceImagesPaths[i]);
//		cvtColor(::referenceImages[i].image, ::referenceImages[i].image, CV_BGR2GRAY);
//		::referenceImages[i].name = ReferenceImagesTitle[i];
//		cv::resize(::referenceImages[i].image, ::referenceImages[i].image, cv::Size(w / 2, h / 2));
//
//	}
//
//}
bool isAnticlock(vector<Point2f> &vertexes)
{
	Point v1 = vertexes[1] - vertexes[0];
	Point v2 = vertexes[2] - vertexes[0];

	//trace a line between the first and second point.
	//if the thrid point is at the right side, then the points are anti-clockwise
	//从代码推测，marker中的点集本来就两种序列：顺时针和逆时针，这里要把顺时针的序列改成逆时针


	//行列式的几何意义是什么呢？有两个解释：一个解释是行列式就是行列式中的行或列向量所构成的超平行多面体的有向面积或有向体积；另一个解释是矩阵A的行列式detA就是线性变换A下的图形面积或体积的伸缩因子。
	double o = (v1.x * v2.y) - (v1.y * v2.x);
	return o > 0.0;
}

double median(cv::Mat channel)
{
	double m = (channel.rows*channel.cols) / 2;
	int bin = 0;
	double med = -1.0;

	int histSize = 256;
	float range[] = { 0, 256 };
	const float* histRange = { range };
	bool uniform = true;
	bool accumulate = false;
	cv::Mat hist;
	cv::calcHist(&channel, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

	for (int i = 0; i < histSize && med < 0.0; ++i)
	{
		bin += cvRound(hist.at< float >(i));
		if (bin > m && med < 0.0)
			med = i;
	}

	return med;
}

Mat auto_canny(Mat &image, float sigma = 0.33)
{


	double v = median(image);

	// apply automatic Canny edge detection using the computed median
	int lower = int(max(0, (1.0 - sigma) * v));
	int upper = int(min(255, (1.0 + sigma) * v));
	Mat edged;
	Canny(image, edged, lower, upper);

	// return the edged image
	return edged;
}
// compute the median of the single channel pixel intensities

void getFourVertexes(Mat & OriginalFrame, vector<Point2f> & vertexes,Point & center)
{


	Mat gray;
	cvtColor(OriginalFrame, gray, CV_BGR2GRAY);
	Mat blurred;
	GaussianBlur(gray, blurred, cv::Size(3, 3), 0);

	//Detecting Edges

	Mat edges = auto_canny(blurred);
	//split(OriginalFrame)
	//Contour Detection & checking for squares based on the square area

	std::vector<std::vector<cv::Point>> contours;
	vector<vector<Point> >poly;


	vector<Mat> splits;
	Mat hsv;
	cvtColor(OriginalFrame, hsv, CV_BGR2HSV);

	split(hsv, splits);
	

	cv::findContours(edges, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
	poly.resize(contours.size());
 

	for (int i = 0; i < contours.size(); i++)
	{
		approxPolyDP(Mat(contours[i]), poly[i], 3, true);// 0.01*arcLength(Mat(contours[i]), true), true);
		//	vector<vector<Point> >poly(contours.size());
		if (poly[i].size() ==12)
		{
			double area = contourArea(poly[i]);
			//std::cout << "GOT YA" << std::endl;
			 
			if (area > minSquareArea)
			{
				//std::cout << "HAHA" << std::endl;
				drawContours(OriginalFrame, poly, i, (0,255, 255), 8);

				double line21 = norm(poly[i][2] - poly[i][1]);
				double line32 = norm(poly[i][3] - poly[i][2]);
				Point center_ = poly[i][0] + poly[i][3] + poly[i][6] + poly[i][9];
				//center.x = (int)(center.x*0.25);
				//center.y = (int)(center.y*0.25);
				center_ = center_ *0.25;
			//	cout << center << endl;		
				//cout << fx*(center.x - u0) / 1000<<" "<<fy*(center.y-v0)/1000<<endl;
				unsigned char *p = splits[0].ptr<unsigned char>(center_.y);
				if (abs(p[center_.x] - 98) < 50) continue;
				center = center_;

				//cout << (int)(p[center.x]) << endl;

			//	if(OriginalFrame.at<Vec3b>(center)[0]>50)//if (p[center.x]>10)
			//	{
			////		 cout << (int)(OriginalFrame.at<Vec3b>(center)[0]) << endl; // cout << (int)p[center.x] << endl;
			//		continue;
			//	}
				//cout << (int)(OriginalFrame.at<Vec3b>(center)[0]) << endl; // cout << (int)p[center.x] << endl;
				vector<Point> Vertex;
				if (3* line21 < line32)// && line32 < line32 * 12)
				{
									
										
					
					vertexes.push_back(poly[i][1]);
					vertexes.push_back(poly[i][4]);
					vertexes.push_back(poly[i][7]);
	             vertexes.push_back(poly[i][10]);
				// cout << "Begin 1:";


				
				}
				else if (line32 *3 <line21)// && line32<12 *line21)
				{
										
										
					vertexes.push_back(poly[i][0]);
					vertexes.push_back(poly[i][3]);
					vertexes.push_back(poly[i][6]);
					vertexes.push_back(poly[i][9]);
				//cout << "Begin 0:";

				}
				else
				{		
					
					vertexes.push_back(poly[i][2]);
					vertexes.push_back(poly[i][5]);
					vertexes.push_back(poly[i][8]);
					vertexes.push_back(poly[i][11]);
	
					//cout << "Begein 2:";
				}
				if (isAnticlock(vertexes))
				{
					Point2f temp = vertexes[1];
					vertexes[1] = vertexes[3];
					vertexes[3] = temp;

				}

				//line(OriginalFrame, poly[i][0], poly[i][3], Scalar(0, 0, 255), 6);

				//line(OriginalFrame, poly[i][3], poly[i][6], Scalar(0, 0, 255), 6);
				arrowedLine(OriginalFrame, vertexes[0], vertexes[1], Scalar(0, 0, 255),6);
			 		//cout << vertexes[0] <<" "<<vertexes[1]<<" "<<vertexes[2]<<" "<<vertexes[3]<< endl;
		 

				//line(OriginalFrame, poly[i][6], poly[i][9], Scalar(0, 0, 255), 6);


				//line(OriginalFrame, poly[i][9], poly[i][0], Scalar(0, 0, 255), 6);

			}
		}
	}



}
