#include <iostream>
#include <getopt.h>
#include <fstream>
#include <sstream>
#include "aruco.h"
using namespace cv;
using namespace aruco;

string TheInputVideo;
string TheIntrinsicFile;
bool The3DInfoAvailable=false;
float TheMarkerSize=-1;
ArMarkerDetector MDetector;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
Mat TheInputImage,TheInputImageCopy;
Mat TheIntriscCameraMatrix,TheDistorsionCameraParams;
void cvTackBarEvents(int pos,void*);
bool readIntrinsicFile(string TheIntrinsicFile,Mat & TheIntriscCameraMatrix,Mat &TheDistorsionCameraParams,Size size);
void readArguments ( int argc,char **argv );
void usage();
void draw3dMarkerAxis(Mat &Image,Marker &m,const Mat& cameraMatrix, const Mat& distCoeffs);
void draw3dMarkerCube(Mat &Image,Marker &m,const Mat& cameraMatrix, const Mat& distCoeffs);
pair<double,double> AvrgTime(0,0) ;//determines the average time required for detection
double ThresParam1,ThresParam2;
int iThresParam1,iThresParam2;
/************************************
 *
 *
 *
 *
 ************************************/
int main(int argc,char **argv)
{
	try
	{

	   

		if(argc==1) usage();
		//parse arguments
		readArguments (argc,argv);
		 //read from camera or from  file
		if (TheInputVideo=="") TheVideoCapturer.open(0);
		else TheVideoCapturer.open(TheInputVideo);
		//check video is open
		if (!TheVideoCapturer.isOpened()){
		  cerr<<"Could not open video"<<endl;
		  return -1;

		}

		//read first image to get the dimensions
		TheVideoCapturer>>TheInputImage;

		//read camera parameters if passed
		if (TheIntrinsicFile=="")
		{
			TheIntriscCameraMatrix=Mat();
			TheDistorsionCameraParams=Mat();
		}
		else
		{
			if (!readIntrinsicFile(TheIntrinsicFile,TheIntriscCameraMatrix,TheDistorsionCameraParams,TheInputImage.size()))
				{
				  cerr<<"could not open file "<<TheIntrinsicFile<<endl;
				  return -1;
				}
			The3DInfoAvailable=true;

		}
		//Create gui

		cv::namedWindow("thres",1);
		cv::namedWindow("in",1);
		MDetector.getThresholdParams( ThresParam1,ThresParam2);
		iThresParam1=ThresParam1;iThresParam2=ThresParam2;
		cv::createTrackbar("ThresParam1", "in",&iThresParam1, 13, cvTackBarEvents);
		cv::createTrackbar("ThresParam2", "in",&iThresParam2, 13, cvTackBarEvents);
		char key=0;
		int index=0;
		//capture until press ESC or until the end of the video
		while( key!=27 && TheVideoCapturer.grab())
		{
			TheVideoCapturer.retrieve( TheInputImage);
			//copy image

			index++; //number of images captured
			double tick = (double)getTickCount();//for checking the speed
			//Detection of markers in the image passed
			MDetector.detect(TheInputImage,TheMarkers,TheIntriscCameraMatrix,TheDistorsionCameraParams,TheMarkerSize);
			//chekc the speed by calculating the mean speed of all iterations
			AvrgTime.first+=((double)getTickCount()-tick)/getTickFrequency();
			AvrgTime.second++;
			cout<<"Time detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds"<<endl;

			//print marker info and draw the markers in image
			TheInputImage.copyTo(TheInputImageCopy);
			for(unsigned int i=0;i<TheMarkers.size();i++){
				cout<<TheMarkers[i]<<endl;
				TheMarkers[i].draw(TheInputImageCopy,Scalar(0,0,255),2);
			}

			//draw a 3d cube in each marker if there is 3d info
			if (The3DInfoAvailable)
			  for(unsigned int i=0;i<TheMarkers.size();i++){
 			    draw3dMarkerCube( TheInputImageCopy,TheMarkers[i],TheIntriscCameraMatrix,TheDistorsionCameraParams);
			    draw3dMarkerAxis( TheInputImageCopy,TheMarkers[i],TheIntriscCameraMatrix,TheDistorsionCameraParams);
			  }
			  //DONE! Easy, right?
			cout<<endl<<endl<<endl;
			//show input with augmented information and  the thresholded image
			cv::imshow("in",TheInputImageCopy);
			cv::imshow("thres",MDetector.getThresholdedImage());

			key=cv::waitKey(0);//wait for key to be pressed
		}
	}catch(std::exception &ex)

	{
		cout<<"Exception :"<<ex.what()<<endl;
	}

}
/************************************
 *
 *
 *
 *
 ************************************/

void cvTackBarEvents(int pos,void*)
{
if (iThresParam1<3) iThresParam1=3;
if (iThresParam1%2!=1) iThresParam1++;
if (ThresParam2<1) ThresParam2=1;
ThresParam1=iThresParam1;
ThresParam2=iThresParam2;
MDetector.setThresholdParams(ThresParam1,ThresParam2);
//recompute
MDetector.detect(TheInputImage,TheMarkers,TheIntriscCameraMatrix,TheDistorsionCameraParams,TheMarkerSize);
TheInputImage.copyTo(TheInputImageCopy);
for(unsigned int i=0;i<TheMarkers.size();i++)	TheMarkers[i].draw(TheInputImageCopy,Scalar(0,0,255),2);
//draw a 3d cube in each marker if there is 3d info
if (The3DInfoAvailable)
  for(unsigned int i=0;i<TheMarkers.size();i++)
    draw3dMarkerCube( TheInputImageCopy,TheMarkers[i],TheIntriscCameraMatrix,TheDistorsionCameraParams);

cv::imshow("in",TheInputImageCopy);
cv::imshow("thres",MDetector.getThresholdedImage());
}

/************************************
 *
 *
 *
 *
 ************************************/
void usage()
{
	cout<<"This program test the ArUco Library \n\n";
	cout<<"-i <video.avi>: specifies a input video file. If not, images from camera are captures"<<endl;
	cout<<"-f <file>: if you have calibrated your camera, pass calibration information here so as to be able to get 3D marker info"<<endl;
	cout<<"-s <size>: size of the marker's sides (expressed in meters!)"<<endl;
}

/************************************
 *
 *
 *
 *
 ************************************/
static const char short_options [] = "hi:f:s:";

static const struct option
long_options [] =
{
	{ "help",           no_argument,   NULL,                 'h' },
	{ "input",     required_argument,   NULL,           'i' },
	{ "intFile",     required_argument,   NULL,           'f' },
	{ "size",     required_argument,   NULL,           's' },

	{ 0, 0, 0, 0 }
};

/************************************
 *
 *
 *
 *
 ************************************/
void readArguments ( int argc,char **argv )
{
	for ( ;; )
	{
		int index;
		int c;
		c = getopt_long ( argc, argv,
			short_options, long_options,
			&index );

		if ( -1 == c )
			break;
		switch ( c )
		{
			case 0:
				break;
			case 'h':
				usage ();
				exit ( EXIT_SUCCESS );
				break;
			case 'i':
				TheInputVideo=optarg;
				break;
			case 'f':
				TheIntrinsicFile=optarg;
				break;
			case 's':
				TheMarkerSize=atof(optarg);
				break;
			default:
				usage ();
				exit ( EXIT_FAILURE );
		};
	}

}


/************************************
 *
 *
 *
 *
 ************************************/
bool readIntrinsicFile(string TheIntrinsicFile,Mat & TheIntriscCameraMatrix,Mat &TheDistorsionCameraParams,Size size)
{
	//open file
	ifstream InFile(TheIntrinsicFile.c_str());
	if (!InFile) return false;
	char line[1024];
	InFile.getline(line,1024);	 //skype first line that should contain only comments
	InFile.getline(line,1024);//read the line with real info

	//transfer to a proper container
	stringstream InLine;
	InLine<<line;
	//Create the matrices
	TheDistorsionCameraParams.create(4,1,CV_32FC1);
	TheIntriscCameraMatrix=Mat::eye(3,3,CV_32FC1);


	//read intrinsic matrix
	InLine>>TheIntriscCameraMatrix.at<float>(0,0);//fx
	InLine>>TheIntriscCameraMatrix.at<float>(1,1); //fy
	InLine>>TheIntriscCameraMatrix.at<float>(0,2); //cx
	InLine>>TheIntriscCameraMatrix.at<float>(1,2);//cy
	//read distorion parameters
	for(int i=0;i<4;i++) InLine>>TheDistorsionCameraParams.at<float>(i,0);

	//now, read the camera size
	float width,height;
	InLine>>width>>height;
	//resize the camera parameters to fit this image size
	float AxFactor= float(size.width)/ width;
	float AyFactor= float(size.height)/ height;
	TheIntriscCameraMatrix.at<float>(0,0)*=AxFactor;
	TheIntriscCameraMatrix.at<float>(0,2)*=AxFactor;
	TheIntriscCameraMatrix.at<float>(1,1)*=AyFactor;
	TheIntriscCameraMatrix.at<float>(1,2)*=AyFactor;

	//debug
	cout<<"fx="<<TheIntriscCameraMatrix.at<float>(0,0)<<endl;
	cout<<"fy="<<TheIntriscCameraMatrix.at<float>(1,1)<<endl;
	cout<<"cx="<<TheIntriscCameraMatrix.at<float>(0,2)<<endl;
	cout<<"cy="<<TheIntriscCameraMatrix.at<float>(1,2)<<endl;
	cout<<"k1="<<TheDistorsionCameraParams.at<float>(0,0)<<endl;
	cout<<"k2="<<TheDistorsionCameraParams.at<float>(1,0)<<endl;
	cout<<"p1="<<TheDistorsionCameraParams.at<float>(2,0)<<endl;
	cout<<"p2="<<TheDistorsionCameraParams.at<float>(3,0)<<endl;

	return true;
}
/************************************
 *
 *
 *
 *
 ************************************/
void draw3dMarkerAxis(Mat &Image,Marker &m,const Mat& cameraMatrix, const Mat& distCoeffs)
{
Mat objectPoints (4,3,CV_32FC1);
objectPoints.at<float>(0,0)=0;objectPoints.at<float>(0,1)=0;objectPoints.at<float>(0,2)=0;
objectPoints.at<float>(1,0)=m.ssize;objectPoints.at<float>(1,1)=0;objectPoints.at<float>(1,2)=0;
objectPoints.at<float>(2,0)=0;objectPoints.at<float>(2,1)=m.ssize;objectPoints.at<float>(2,2)=0;
objectPoints.at<float>(3,0)=0;objectPoints.at<float>(3,1)=0;objectPoints.at<float>(3,2)=m.ssize;

vector<Point2f> imagePoints;
projectPoints( objectPoints, m.Rvec,m.Tvec, cameraMatrix,  distCoeffs,   imagePoints);
//draw lines of different colours
cv::line(Image,imagePoints[0],imagePoints[1],Scalar(0,0,255),1,CV_AA);
cv::line(Image,imagePoints[0],imagePoints[2],Scalar(0,255,0),1,CV_AA);
cv::line(Image,imagePoints[0],imagePoints[3],Scalar(255,0,0),1,CV_AA);
putText(Image,"x", imagePoints[1],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255),2);
putText(Image,"y", imagePoints[2],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,0),2);
putText(Image,"z", imagePoints[3],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,0,0),2);
}

/************************************
 *
 *
 *
 *
 ************************************/
void draw3dMarkerCube(Mat &Image,Marker &m,const Mat& cameraMatrix, const Mat& distCoeffs)
{
Mat objectPoints (8,3,CV_32FC1);
double halfSize=m.ssize/2;
objectPoints.at<float>(0,0)=-halfSize;objectPoints.at<float>(0,1)=0;objectPoints.at<float>(0,2)=-halfSize;
objectPoints.at<float>(1,0)=halfSize;objectPoints.at<float>(1,1)=0;objectPoints.at<float>(1,2)=-halfSize;
objectPoints.at<float>(2,0)=halfSize;objectPoints.at<float>(2,1)=0;objectPoints.at<float>(2,2)=halfSize;
objectPoints.at<float>(3,0)=-halfSize;objectPoints.at<float>(3,1)=0;objectPoints.at<float>(3,2)=halfSize;

objectPoints.at<float>(4,0)=-halfSize;objectPoints.at<float>(4,1)=m.ssize;objectPoints.at<float>(4,2)=-halfSize;
objectPoints.at<float>(5,0)=halfSize;objectPoints.at<float>(5,1)=m.ssize;objectPoints.at<float>(5,2)=-halfSize;
objectPoints.at<float>(6,0)=halfSize;objectPoints.at<float>(6,1)=m.ssize;objectPoints.at<float>(6,2)=halfSize;
objectPoints.at<float>(7,0)=-halfSize;objectPoints.at<float>(7,1)=m.ssize;objectPoints.at<float>(7,2)=halfSize;

vector<Point2f> imagePoints;
projectPoints( objectPoints, m.Rvec,m.Tvec, cameraMatrix,  distCoeffs,   imagePoints);
//draw lines of different colours
for(int i=0;i<4;i++)
  cv::line(Image,imagePoints[i],imagePoints[(i+1)%4],Scalar(0,0,255),1,CV_AA);

for(int i=0;i<4;i++)
  cv::line(Image,imagePoints[i+4],imagePoints[4+(i+1)%4],Scalar(0,0,255),1,CV_AA);

for(int i=0;i<4;i++)
  cv::line(Image,imagePoints[i],imagePoints[i+4],Scalar(0,0,255),1,CV_AA);

/*for(unsigned int i=0;i<imagePoints.size();i++)
    cout<<imagePoints[i].x<<" "<<imagePoints[i].y<<endl;*/
}
