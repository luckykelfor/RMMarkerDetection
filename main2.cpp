#include "stdafx.h"

#include <opencv2\opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "..\include\PNPSolver.h"
#include"DetecRM.h"
using namespace std;
using namespace cv;



double fx = 1164.1; // / 1194.61;
double fy = 1163.8;///;// 1196.98;
double u0 = 631.2303;// 233.0957;// 302.9064;// 634.075;
double v0 = 373.7216;// 300.378;// 235.2960;// 504.842;
//镜头畸变参数
double k1 = 0.0356;;
double k2 = -0.0737;
double p1 = -0.00099;
double p2 = -0.0030;
double k3 =0.00;

Mat patt_RM;
Mat patt_Arrow;
int PATTERN_SIZE = 200;
 
int main()
{
	//初始化相机，此处替换为你的相机驱动程序
	
	cv::VideoCapture vcap;
	vcap.open(0);
	cv::Mat frame;
	cv::namedWindow("CamPos", 0);


	patt_RM = imread("D:/RM.jpg");
	cvtColor(patt_RM, patt_RM, CV_BGR2GRAY);
	resize(patt_RM, patt_RM, Size(PATTERN_SIZE, PATTERN_SIZE));
	threshold(patt_RM, patt_RM, 100, 255, CV_THRESH_BINARY);
	imshow("Patt", patt_RM);


	//patt_Arrow = imread("D:/Arrow.jpg");
	//cvtColor(patt_Arrow, patt_Arrow, CV_BGR2GRAY);
	//resize(patt_Arrow, patt_Arrow, Size(PATTERN_SIZE, PATTERN_SIZE));
	//threshold(patt_Arrow, patt_Arrow, 100, 255, CV_THRESH_BINARY);
	//imshow("Arrow", patt_Arrow);

		//初始化位姿估计类
		PNPSolver p4psolver;

		//初始化相机参数
		p4psolver.SetCameraMatrix(fx, fy, u0, v0);
		//设置畸变参数
		p4psolver.SetDistortionCoefficients(k1, k2, p1, p2, k3);

		//p4psolver.Points3D.push_back(cv::Point3f(100, 60, 0));		//P1三维坐标的单位是毫米
		//p4psolver.Points3D.push_back(cv::Point3f(83, 72, 0));	//P2
		//p4psolver.Points3D.push_back(cv::Point3f(116, 72, 0));	//P3
		//p4psolver.Points3D.push_back(cv::Point3f(90, 114, 0));	//P4

		Point center;




		//将点重投影回图像，检验投影点是否正确
		vector<cv::Point3f> r;
		r.push_back(cv::Point3f(-100, -100, 0));//重投影点事件坐标
		r.push_back(cv::Point3f(100, -100, 0));//重投影点事件坐标
		r.push_back(cv::Point3f(100, 100, 0));//重投影点事件坐标
		r.push_back(cv::Point3f(-100, 100, 0));//重投影点事件坐标

		r.push_back(cv::Point3f(-100, -100, 200));//重投影点事件坐标
		r.push_back(cv::Point3f(100, -100, 200));//重投影点事件坐标
		r.push_back(cv::Point3f(100, 100, 200));//重投影点事件坐标
		r.push_back(cv::Point3f(-100, 100, 200));//重投影点事件坐标


		vcap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
		vcap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
		vcap.set(CV_CAP_PROP_FPS, 60);




	while(vcap.isOpened())
	{
		vcap.read(frame);
		cv::Mat paintBoard = cv::Mat::zeros(frame.size(), CV_8UC3);//新建一个Mat，用于存储绘制的东西
		bool isRM;
		cv::imshow("CamPos", (frame - paintBoard) + paintBoard + paintBoard);
		getFourVertexes(frame, p4psolver.Points2D, center,isRM);
		//cout << frame.cols << " "<<frame.rows<<endl;
		if (p4psolver.Points2D.size() == 4)
		{

			if (isRM)
			{
				//将特征点的世界坐标添加进去
				p4psolver.Points3D.push_back(cv::Point3f(-60, -60, 0));		//P1三维坐标的单位是毫米

				p4psolver.Points3D.push_back(cv::Point3f(-60, 60, 0));	//
				p4psolver.Points3D.push_back(cv::Point3f(60, 60, 0));	//P3
				p4psolver.Points3D.push_back(cv::Point3f(60, -60, 0));	//P2


			}
			else
			{
				p4psolver.Points3D.push_back(cv::Point3f(-65, -75, 0));		//P1三维坐标的单位是毫米

				p4psolver.Points3D.push_back(cv::Point3f(-65, 75, 0));	//
				p4psolver.Points3D.push_back(cv::Point3f(65, 75, 0));	//P3
				p4psolver.Points3D.push_back(cv::Point3f(65, -75, 0));	//P2

			}

			////解位姿
			p4psolver.Solve(PNPSolver::METHOD::CV_ITERATIVE);

			vector<cv::Point2f>	ps = p4psolver.WordFrame2ImageFrame(r);
 

			//Draw the reconstructed cube
			cv::line(frame, ps[4], ps[5], cv::Scalar(255, 255, 0), 2);
			cv::line(frame, ps[5], ps[6], cv::Scalar(255, 255, 0), 2);
			cv::line(frame, ps[6], ps[7], cv::Scalar(255, 255, 0), 2);
			cv::line(frame, ps[7], ps[4], cv::Scalar(255, 255, 0), 2);



			cv::line(frame, ps[0], ps[1], cv::Scalar(0, 255, 0), 3);
			cv::line(frame, ps[1], ps[2], cv::Scalar(0, 255, 0),3);
			cv::line(frame, ps[2], ps[3], cv::Scalar(0, 255, 0), 3);
			cv::line(frame, ps[3], ps[0], cv::Scalar(0, 255, 0),3);

			cv::line(frame, ps[0], ps[4], cv::Scalar(255, 0, 255), 3);
			cv::line(frame, ps[1], ps[5], cv::Scalar(255, 0, 255), 3);
			cv::line(frame, ps[2], ps[6], cv::Scalar(255, 0, 255), 3);
			cv::line(frame, ps[3], ps[7], cv::Scalar(255, 0, 255), 3);


			

			//输出位姿信息
			cout << "pitch:"<<p4psolver.Theta_W2C.x << " roll:" << p4psolver.Theta_W2C.y << " yaw:" << p4psolver.Theta_W2C.z << endl;
			//It semms the z component from solvPNP is not that trusty.
			cout << "x;" << p4psolver.Position_OwInC.x << " y:" << p4psolver.Position_OwInC.y << " z:" << p4psolver.Position_OwInC.z << endl;

			 
		}
		p4psolver.Points2D.clear();
		p4psolver.Points3D.clear();
		cv::imshow("CamPos", (frame - paintBoard) + paintBoard + paintBoard);
		if( 27 == cv::waitKey(2))
			break;
	}
	vcap.release();//else
	//	std::cout << "相机初始化失败！" << endl;
	return 0;
}