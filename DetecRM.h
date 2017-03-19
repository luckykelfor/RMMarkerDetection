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
//include<opencv.hpp>
using namespace cv;
#define REF_IMAGE_NUM 3
 
const int minSquareArea = 5000;
const int minDiff = 10000;
const int match = -1;
 
extern double fx;// = 1151.1; // / 1194.61;
extern double fy;// = 1150.8;///;// 1196.98;
extern double u0;// = 627.29108;// 233.0957;// 302.9064;// 634.075;
extern double v0;// = 369.8137;// 300.378;// 235.2960;// 504.842;
extern int PATTERN_SIZE;
double max(double a, double b)
{
	return a > b ? a : b;
}
double min(double a, double b)
{
	return a < b ? a : b;
}

extern Mat patt_RM;

extern Mat patt_Arrow;

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

void resize_and_threshold_warped(Mat & warppedimage)
{
//Resize the corrected image to proper size & convert it to grayscale
//warped_new = cv2.resize(image, (w / 2, h / 2))
//warped_new_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

//Smoothing Out Image
//blur = cv2.GaussianBlur(warppedimage, (5, 5), 0)
 
	GaussianBlur(warppedimage, warppedimage, Size(3, 3), 0);
//Calculate the maximum pixel and minimum pixel value & compute threshold
	double min_val, max_val,min_loc,max_loc;
	minMaxLoc(warppedimage, &min_val, &max_val);

 
	double thresh = (min_val + max_val) / 2;

//Threshold the image
	cv::threshold(warppedimage, warppedimage, thresh, 255, CV_THRESH_BINARY);



}


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

void getFourVertexes(Mat & OriginalFrame, vector<Point2f> & vertexes,Point & center, bool & isRM)
{

	vector<Mat> splits;

	split(OriginalFrame, splits);
	Mat gray = splits[0].clone();
	//cvtColor(OriginalFrame, gray, CV_BGR2GRAY);
	Mat blurred;
	GaussianBlur(gray, blurred, cv::Size(3, 3), 0);

	//Detecting Edges

	Mat edges = auto_canny(blurred);

	std::vector<std::vector<cv::Point>> contours;
	vector<vector<Point> >poly;


	cv::findContours(edges, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
	poly.resize(contours.size());

	vector<Point2f> Vertex_tmp;
	int index = 0;
	for (int i = 0; i < contours.size(); i++)
	{
		approxPolyDP(Mat(contours[i]), poly[i], 3, true);// 0.01*arcLength(Mat(contours[i]), true), true);
		double area;
		 
		if (poly[i].size() ==12)
		{
			 area = contourArea(poly[i]);

			if (area > minSquareArea)
			{
				//std::cout << "HAHA" << std::endl;
				drawContours(OriginalFrame, poly, i, (0,255, 255), 8);

				double line21 = norm(poly[i][2] - poly[i][1]);
				double line32 = norm(poly[i][3] - poly[i][2]);
				Point center_ = poly[i][0] + poly[i][3] + poly[i][6] + poly[i][9];
				center_ = center_ *0.25;
				unsigned char *p = splits[1].ptr<unsigned char>(center_.y);
				cout<<abs(p[center_.x] - 98)<<endl;
				if (abs(p[center_.x] - 98) > 50)
					continue;
					
				isRM = true;
				cout << abs(p[center_.x] - 98) << endl;//Trick here!
				center = center_;
				Point2f Vertex[4];

				//根据长短边确定选择四个内角点
				if (3* line21 < line32)// && line32 < line32 * 12)
				{				
					
					Vertex_tmp.push_back(poly[i][1]);
					Vertex_tmp.push_back(poly[i][4]);
					Vertex_tmp.push_back(poly[i][7]);
					Vertex_tmp.push_back(poly[i][10]);
				
				}
				else if (line32 *3 <line21)// && line32<12 *line21)
				{
										
										
					Vertex_tmp.push_back(poly[i][0]);
					Vertex_tmp.push_back(poly[i][3]);
					Vertex_tmp.push_back(poly[i][6]);
					Vertex_tmp.push_back(poly[i][9]);

				}
				else
				{		
					Vertex_tmp.push_back(poly[i][2]);
					Vertex_tmp.push_back(poly[i][5]);
					Vertex_tmp.push_back(poly[i][8]);
					Vertex_tmp.push_back(poly[i][11]);
				}
				if (isAnticlock(Vertex_tmp))//确保是逆时针顺序排列角点
				{
					Point2f temp = Vertex_tmp[1];
					Vertex_tmp[1] = Vertex_tmp[3];
					Vertex_tmp[3] = temp;

				}

               Mat warpped;
               Mat res;
			   Mat dst;
				

			   //Compute the diffImage to determine for finding the right order.
				int diff[4];
				for (int i = 0; i < 4; i++)
				{
					Vertex[(0+i)%4] = Point2f(0, 0);
					Vertex[(1 + i) % 4] = Point2f(0, PATTERN_SIZE);
					Vertex[(2 + i) % 4] =Point2f(PATTERN_SIZE, PATTERN_SIZE);
					Vertex[(3 + i) % 4] = (Point2f(PATTERN_SIZE, 0));
                       				
					res = getPerspectiveTransform(&Vertex_tmp[0], Vertex);
					warpPerspective(gray, warpped, res, cv::Size(PATTERN_SIZE, PATTERN_SIZE));
					resize_and_threshold_warped(warpped);

					bitwise_xor(warpped, patt_RM, dst);
					//else
					//   bitwise_xor(warpped, patt_Arrow, dst);
					//imshow("warpped", warpped); waitKey();
					diff[i] = countNonZero(dst);
					
				}
				
	
				index = 0;
				int index_M = 0;
				for (int i = 1; i < 4; i++)
				{
					if (diff[i]<diff[index])
					{
						index = i;
					}
					if (diff[i]>diff[index_M])
					{
						index_M = i;
					}

				}
			//	cout << (float)diff[index_M] / (PATTERN_SIZE*PATTERN_SIZE) << endl;//Another hint for determining RM or Arrow.
 

				// shift to fix the order.
				vertexes.push_back(Vertex_tmp[(0 + index) % 4]);
				vertexes.push_back(Vertex_tmp[(1 + index) % 4]);
				vertexes.push_back(Vertex_tmp[(2 + index) % 4]);
				vertexes.push_back(Vertex_tmp[(3 + index) % 4]);
                //index = 0;;// if (!isRM && index >2)index += 1;
				arrowedLine(OriginalFrame, vertexes[0], vertexes[1], Scalar(0, 0, 255),6);
				return; //Here, remember to return for the current frame!


			}
		}
	}



}
