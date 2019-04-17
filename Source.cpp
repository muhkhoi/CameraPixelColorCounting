#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "SerialPort.h"
#include <fstream>
#include <cstring>
#include <ctime>
#include <limits.h>

using namespace std;
using namespace cv;

char* portName = "\\\\.\\COM5";

#define MAX_DATA_LENGTH 255
SerialPort *arduino;

#define H           0
#define S           1
#define V           2

#define ul			4

#define erSize      0
#define diSize      1

#define RECT        0
#define CROSS       1
#define ELIPSE      2
int countkirim = 0;
#define BIRU        1
#define TERANG		2
#define GELAP       3

int BMin[3], BMax[3], Bed[2];
int TMin[3], TMax[3], Ted[2];
int GMin[3], GMax[3], Ged[2];
int rMin[3] = {0,0,0};
int rMax[3] = {255,255,255};
int rED[2]  = {0,0};


char file[]={"data0.ul"};

Mat frameColor;

Mat eleMor = getStructuringElement(MORPH_RECT,Size(11,11));
Mat se1 = getStructuringElement(MORPH_RECT, Size(5, 5));
Mat se2 = getStructuringElement(MORPH_RECT, Size(2, 2));
int biruX = 0;
int biruY = 0;
int countimg = 0;

void importData()
{
	ifstream buka(file);
	string line;
	int j=0, buf=0, data=0;
	if (buka.is_open()){
		while (buka.good())
		{
			char buffer[5] = {};
			getline(buka,line);
			for(size_t i=0; i<line.length(); i++)
			{
				buffer[j] = line[i];
				if(line[i]==';')
				{
					j = 0;
					buf = atoi(buffer);
					if(data<3){
						rMin[data] = buf;//printf("wMin : %d\n", rMin[data]);
					}else if(data<6){
						rMax[data%3] = buf;//printf("wMax : %d\n", rMax[data%3]);
					}else if(data<8){
						rED[data%6] = buf;//printf("ED : %d\n", rED[data%6]);
					}
					data++;
					continue;
				}
				j++;
			}
		}
		buka.close();
	}
}

void initColors(void){
	file[ul] = '0';
	importData();
	BMin[H] = rMin[H];	BMax[H] = rMax[H];	Bed[erSize] = rED[erSize];			//biru
	BMin[S] = rMin[S];	BMax[S] = rMax[S];	Bed[diSize] = rED[diSize];
	BMin[V] = rMin[V];	BMax[V] = rMax[V];

	file[ul] = '1';
	importData();
	TMin[H] = rMin[H];	TMax[H] = rMax[H];	Ted[erSize] = rED[erSize];			//terang
	TMin[S] = rMin[S];	TMax[S] = rMax[S];	Ted[diSize] = rED[diSize];
	TMin[V] = rMin[V];	TMax[V] = rMax[V];

	file[ul] = '2';
	importData();
	GMin[H] = rMin[H];	GMax[H] = rMax[H];	Ged[erSize] = rED[erSize];			//gelap
	GMin[S] = rMin[S];	GMax[S] = rMax[S];	Ged[diSize] = rED[diSize];
	GMin[V] = rMin[V];	GMax[V] = rMax[V];
}

Mat GetThresImage(Mat img, int mode, int colorMode)	//Data InitColor
{
	Mat imHSV, thres;
	
	cvtColor(img, imHSV, COLOR_BGR2HSV);
	if(mode == 1)
	{ 
		inRange(imHSV, Scalar(BMin[H], BMin[S], BMin[V]), Scalar(BMax[H], BMax[S], BMax[V]), thres);
	}
	else if(mode == 2)
	{ 
		inRange(imHSV, Scalar(TMin[H], TMin[S], TMin[V]), Scalar(TMax[H], TMax[S], TMax[V]), thres);
	}
	else if(mode == 3)
	{ 
		inRange(imHSV, Scalar(GMin[H], GMin[S], GMin[V]), Scalar(GMax[H], GMax[S], GMax[V]), thres);
	}
	imHSV.release();
	return thres;
}


void pixelCek(Mat img,Mat img2, Mat frame)
{
		
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		vector<vector<Point> > contours2;
		vector<Vec4i> hierarchy2;
		Mat frame2;
		Mat dst;
		Mat dstBin;
		Mat result;
		Mat imgCpy;
		Mat img2Cpy;
		frame.copyTo(frame2);
		img.copyTo(imgCpy);
		img2.copyTo(img2Cpy);
		int pixel = 0;
		int counterB = 0;
		int counterP = 0;
		int tanda = 0;
		int biruThr = 15500; //jumlah pixel biru batas minimum (25%) +-60000 pixel (ketutup teleskop) * 0.25 = 15000
        for( int y = 0; y < frame.rows; y++ ) 
        {
            for( int x = 0; x < frame.cols; x++ ) 
            {
				pixel = (int)img.at<uchar>(cv::Point2i(x,y));
				if(pixel== 255)
				{	
					counterB++;
					
				}
            }
			
        }
		for( int y = 0; y < frame.rows; y++ ) 
        {
            for( int x = 0; x < frame.cols; x++ ) 
            {
				pixel = (int)img2.at<uchar>(cv::Point2i(x,y));
				if(pixel== 255)
				{	
					counterP++;
					
				}
            }
			
        }

		findContours( img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		findContours( img2, contours2, hierarchy2, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		
		for( int i = 0; i< contours.size(); i++ )
		{
			Scalar color = CV_RGB(0, 0, 0);
			drawContours( frame, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
		}
		for( int i = 0; i< contours2.size(); i++ )
		{
			Scalar color = CV_RGB(255, 0, 0);
			drawContours( frame2, contours2, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
		}
		hconcat(frame, frame2, dst);
		hconcat(imgCpy, img2Cpy, dstBin);
		//hconcat(dst,dstBin,result);
		if(counterB >= 25600) //lebih dari 35% biru == terang
		{
			cv::putText(dst, "Cerah!", cv::Point(10, 40), cv::FONT_HERSHEY_DUPLEX,1.0,  CV_RGB(118, 118, 0),2);	
			if(countkirim == 0)
			{
			
			std::string input_string = "A";
			char *c_string = new char[input_string.size() + 1];
			//copying the std::string to c string
			std::copy(input_string.begin(), input_string.end(), c_string);
			//Adding the delimiter
			c_string[input_string.size()] = '\n';
			arduino->writeSerialPort(c_string,2);
			}
			countkirim++;
			if(countkirim == 50){countkirim = 0;}

		}
		else if(counterB >= counterP && counterB >= biruThr)	//biru lebih banyak dari putih dan nilai pixel lebih banyak dari ketnetuan
		{
			cv::putText(dst, "Cerah!", cv::Point(10, 40), cv::FONT_HERSHEY_DUPLEX,1.0,  CV_RGB(118, 118, 0),2);
			if(countkirim == 0)
			{
				
			std::string input_string = "A";
			char *c_string = new char[input_string.size() + 1];
			//copying the std::string to c string
			std::copy(input_string.begin(), input_string.end(), c_string);
			//Adding the delimiter
			c_string[input_string.size()] = '\n';
			arduino->writeSerialPort(c_string,2);
			}
			countkirim++;
			if(countkirim == 50){countkirim = 0;}
		}
		else if(counterB >= counterP && counterB < biruThr)	//biru lebih banyak dari putih tapi nilai jumlah pixel kurang dari ketentuan
		{
			cv::putText(dst, "Agak Mendung", cv::Point(10, 40), cv::FONT_HERSHEY_DUPLEX,1.0,  CV_RGB(118, 50, 0),2);	
			if(countkirim == 0)
			{
			
			std::string input_string = "B";
			char *c_string = new char[input_string.size() + 1];
			//copying the std::string to c string
			std::copy(input_string.begin(), input_string.end(), c_string);
			//Adding the delimiter
			c_string[input_string.size()] = '\n';
			arduino->writeSerialPort(c_string,2);
			}
			countkirim++;
			if(countkirim == 50){countkirim = 0;}
		}
		else if(counterP >= counterB && counterB >= biruThr)	//putih lebih banyak dari biru tapi jumlah pixel biru lebih banyak dari ketentuan
		{
			cv::putText(dst, "Cerah!", cv::Point(10, 40), cv::FONT_HERSHEY_DUPLEX,1.0,  CV_RGB(118, 118, 0),2);	
			if(countkirim == 0)
			{
			
			
			std::string input_string = "A";
			char *c_string = new char[input_string.size() + 1];
			//copying the std::string to c string
			std::copy(input_string.begin(), input_string.end(), c_string);
			//Adding the delimiter
			c_string[input_string.size()] = '\n';
			arduino->writeSerialPort(c_string,2);
			}
			countkirim++;
			if(countkirim == 50){countkirim = 0;}
		}
		else if(counterP >= counterB && counterB < biruThr && counterB >= 5000)	//putih lebih banyak dari biru dan jumlah pixel biru kurang dari ketentuan
		{
			cv::putText(dst, "Agak Mendung", cv::Point(10, 40), cv::FONT_HERSHEY_DUPLEX,1.0,  CV_RGB(118, 50, 0),2);	
			if(countkirim == 0)
			{
			
			std::string input_string = "B";
			char *c_string = new char[input_string.size() + 1];
			//copying the std::string to c string
			std::copy(input_string.begin(), input_string.end(), c_string);
			//Adding the delimiter
			c_string[input_string.size()] = '\n';
			arduino->writeSerialPort(c_string,2);
			}
			countkirim++;
			if(countkirim == 50){countkirim = 0;}
		}

		else if((counterB >= 0 && counterB <= 5000) || counterB == 0)		//biru hampir tidak ada = gelap
		{
			cv::putText(dst, "Gelap", cv::Point(10, 40), cv::FONT_HERSHEY_DUPLEX,1.0,  CV_RGB(118, 0, 0),2);
			if(countkirim == 0){
				
			std::string input_string = "C";
			char *c_string = new char[input_string.size() + 1];
			//copying the std::string to c string
			std::copy(input_string.begin(), input_string.end(), c_string);
			//Adding the delimiter
			c_string[input_string.size()] = '\n';
			arduino->writeSerialPort(c_string,2);
			}
			countkirim++;
			if(countkirim == 50){countkirim = 0;}
		}
		printf("biru %d putih %d\n",counterB,counterP);
		imshow("Bin",dstBin);
		imshow("Img",dst);
}
/*
int pixelPutih(Mat img, Mat frame)
{
		
		int pixel = 0;
		int counter = 0;
		int tanda = 0;
		
        
		if(counter >= 30000)			//lebih dari 50% awan putih = berawan
		{
			//printf("Berawan\n");
			tanda = 1;
		}
		else if(counter >= 15000 && counter < 30000)			//lebih dari 30% awan putih = agak berawan
		{
			//printf("agak berawan\n");
			tanda = 2;
		}
		else if(counter < 15000)			//kurang dari 2% awan putih = tidak berawan (kemungkinan gelap / cerah)
		{
			//printf("tidak berawan\n");
			tanda = 3;
		}
		printf("%d\n",counter);
		return tanda;
			
}
*/
void cekThres(Mat frame, Mat frameP){

	Mat thresB;
	Mat thresP;
	Mat imHSVB;
	Mat imHSVP;

	cvtColor(frame, imHSVB, COLOR_BGR2HSV);
	cvtColor(frameP, imHSVP, COLOR_BGR2HSV);
	
	thresB = GetThresImage(frame, BIRU,1);
	thresP = GetThresImage(frameP, TERANG,1);

	//imshow("thrB",thresB);
	//imshow("thrP",thresP);
 	pixelCek(thresB,thresP,frame);
}

int main()
{
		initColors();

		VideoCapture capture(0);
		//capture.set(CV_CAP_PROP_EXPOSURE, 20);
		capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
		capture.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
		

		if(!capture.isOpened())
		{
			
			fprintf(stderr,"Frame null!\n");
			fprintf(stderr,"Cek kabel kamera!\n");
		}
		
		VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'),10, Size(320,240),true);
		VideoCapture capturess("out.avi");
		arduino = new SerialPort(portName);

		if (arduino->isConnected()){
			std::cout << "Udah Nyambung" << portName << endl;
		}
		Mat frame;
		Mat frame2;
		Mat framePutih;
		Mat frameGelap;

		while(1){
			capture >> frame;
			
			//frame = imread("ikiset.jpg",IMREAD_ANYCOLOR | IMREAD_ANYDEPTH);
			frame.copyTo(framePutih);
			imshow("Capture",frame);
			video.write(frame);
			
			cekThres(frame,framePutih);
			capturess >> frame2;
			//imshow("view",frame2);

			char serialKirim;
 			//printf("%d %d %d\n",tandaBiru,tandaPutih,tandaGelap);
			//case cerah
			
			//delete[] c_string;
			
			
			//delete[] c_string;
			if(cvWaitKey(30)=='q')	exit(1);
		}
		
		cvDestroyAllWindows();
		return 0;

}




