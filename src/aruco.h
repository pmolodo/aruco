#ifndef CV_ARUCO_H
#define CV_ARUCO_H
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cstdio>
#include <iostream>
using namespace std;
using namespace cv;

namespace aruco
{
	/**\brief This class represents a marker
	 *
	 */

	class Marker: public vector<Point2f>
	{
		public:
			//id of  the marker
			int id;
			//size of the markers sides in meters
			float ssize;
			//matrices of rotation and translation respect to the camera
			Mat Rvec,Tvec;

			/**
			 */
			Marker()
			{
				id=-1;
				ssize=-1;
				Rvec.create(3,1,CV_32FC1);
				Tvec.create(3,1,CV_32FC1);
				for(int i=0;i<3;i++)
					Tvec.at<float>(i,0)=Rvec.at<float>(i,0)=-999999;
			}
			/**
			 */
			Marker(const Marker &M):vector<Point2f>(M)
			{
				M.Rvec.copyTo(Rvec);
				M.Tvec.copyTo(Tvec);
				id=M.id;
				ssize=M.ssize;
			}
			/**
			 */
			~Marker(){}
			/**Draws this marker in the input image
			 */
			void draw(Mat &in, Scalar color, int lineWidth=1,bool writeId=true);
			
			/**Given the extrinsic camera parameters returns the GL_MODELVIEW matrix for opengl.
			 * Setting this matrix, the reference corrdinate system will be set in this marker
			 */
			void glGetModelViewMatrix(  double modelview_matrix[16])throw(cv::Exception);
 
			/**
			 */
			friend bool operator<(const Marker &M1,const Marker&M2)
			{
				return M1.id<M2.id;
			}
			/**
			 */
			friend ostream & operator<<(ostream &str,const Marker &M)
			{
				str<<M.id<<"=";
				for(int i=0;i<4;i++)
					str<<"("<<M[i].x<< ","<<M[i].y<<") ";
				str<<"Txyz=";
				for(int i=0;i<3;i++)
					str<<M.Tvec.at<float>(i,0)<<" ";
				str<<"Rxyz=";
				for(int i=0;i<3;i++)
					str<<M.Rvec.at<float>(i,0)<<" ";

				return str;
			}
			

	};
	
	/**\brief Main class for marker detection
	 *
	 */
	class ArMarkerDetector
	{
		public:

			/**
			 */
			ArMarkerDetector();

			/**
			 */
			~ArMarkerDetector();

			/**Detects the markers in the image passed
			 *
			 * If you provide information about the camera parameters and the size of the marker, then, the extrinsics of the markers are detected
			 *
			 * @param input input color image
			 * @param detectedMarkers output vector with the markers detected
			 * @param camMatrix intrinsic camera information.
			 * @param distCoeff camera distorsion coefficient. If set Mat() if is assumed no camera distorion
			 * @param markerSizeMeters size of the marker sides expressed in meters
			 */
			void detect(Mat &input,vector<Marker> &detectedMarkers,Mat camMatrix=Mat(),Mat distCoeff=Mat(),float markerSizeMeters=-1) throw (cv::Exception);

			  /**This set the type of thresholding methods available
			   */
			  
			enum ThresholdMethods{FIXED_THRES,ADPT_THRES,CANNY};
			
			/**Sets the threshold method
			 */
			void setThresholdMethod(ThresholdMethods m){_thresMethod=m;}
			/**Returns the current threshold method
			 */
			ThresholdMethods getThresholdMethod()const{return _thresMethod;}
			 /**
			  * Set the parameters of the threshold method
			  * We are currently using the Adptive threshold ee opencv doc of adaptiveThreshold for more info
			  *   @param param1: blockSize of the pixel neighborhood that is used to calculate a threshold value for the pixel
			  *   @param param2: The constant subtracted from the mean or weighted mean
			  */
			void setThresholdParams(double param1,double param2){_thresParam1=param1;_thresParam2=param2;}
			 /**
			  * Set the parameters of the threshold method
			  * We are currently using the Adptive threshold ee opencv doc of adaptiveThreshold for more info
			  *   param1: blockSize of the pixel neighborhood that is used to calculate a threshold value for the pixel
			  *   param2: The constant subtracted from the mean or weighted mean
			  */
			void getThresholdParams(double &param1,double &param2)const {param1=_thresParam1;param2=_thresParam2;}


			 /**Returns a reference to the internal image thresholded. It is for visualization purposes and to adjust manually 
			  * the parameters
			  */
			 Mat & getThresholdedImage(){return thres;}
			 
			 
			/**Given the intrinsic camera parameters returns the GL_PROJECTION matrix for opengl.
			 * PLease NOTE that when using OpenGL, it is assumed no camera distorsion! So, if it is not true, you should have
			 * undistor image
			 *
			 * @param CamMatrix intrinsic parameters of the camera specified.
			 * @param orgImgSize size of the original image
			 * @param size of the image/window where to render (can be different from the real camera image). Please not that it must be related to CamMatrix
			 * @param proj_matrix output projection matrix to give to opengl
			 * @param gnear,gfar: visible rendering range
			 * @param invert: indicates if the output projection matrix has to yield a horizontally inverted image because image data has not been stored in the order of glDrawPixels: bottom-to-top.
			 */
			static void glGetProjectionMatrix( Mat &  CamMatrix,Size orgImgSize, Size size,double proj_matrix[16],double gnear,double gfar,bool invert=false   )throw(cv::Exception);
	  private:

			Mat grey,thres,thres2;
			vector<vector<Point> > contours2;
			vector<Vec4i> hierarchy2;
			//Threshold parameters
			double _thresParam1,_thresParam2;
			
			ThresholdMethods _thresMethod;
			/**Given the iput image with markers, creates an output image with it in the canonical position
			 * @param in input image
			 * @param out image with the marker
			 * @param size of out
			 * @param points 4 corners of the marker in the image in
			 */
			void warp(Mat &in,Mat &out,Size size, vector<Point2f> points)throw (cv::Exception);

			int hammDistMarker(Mat  bits);
			int mat2id(Mat &bits);
			/**Correct errors in the markers
			 */
			bool correctHammMarker(Mat &bits);

			Mat rotate(Mat  in);

			/**
			 * @pram nRotations number of 90deg rotations in clowise direction needed to set the marker in correct position
			 */
			int getMarkerId(Mat &in,int &nRotations);

			void drawApproxCurve(Mat &in,vector<Point>  &approxCurve ,Scalar color);
			void drawContour(Mat &in,vector<Point>  &contour,Scalar  );
			void thresHold(int method,Mat &grey,Mat &out);

			void drawAllContours(Mat input);

			void draw(Mat out,const vector<Marker> &markers );
			/**
			 */
			bool isInto(Mat &contour,vector<Point2f> &b); 
			/**
			 */
			//bool isInto(vector<Point2f> &a,vector<Point2f> &b);
			/**
			 */
			int perimeter(vector<Point2f> &a);
			/**
			 */
			template<typename T>
				void printMat(Mat M,string info="")
			{

				cout<<info<<endl;
				for(int y=0;y<M.rows;y++)
				{
					for(int x=0;x<M.cols;x++)
					{
						if(sizeof(T)==1)
							cout<<(int) M.at<T>(y,x)<<" ";
						else cout<<  M.at<T>(y,x)<<" ";
					}
					cout<<endl;
				}
			}
			/**
			 */
			template<typename T>
				void printMat(CvMat   *M,string info="")
			{
				cout<<info<<endl;
				Mat MM(M);
				for(int y=0;y<MM.rows;y++)
				{
					for(int x=0;x<MM.cols;x++)
					{
						if(sizeof(T)==1)
							cout<<(int) MM.at<T>(y,x)<<" ";
						else cout<<  MM.at<T>(y,x)<<" ";
					}
					cout<<endl;
				}
			}

			//from ARToolKit

			static void argConvGLcpara2( double cparam[3][4], int width, int height, double gnear, double gfar, double m[16], bool invert )throw(cv::Exception);
			static int  arParamDecompMat( double source[3][4], double cpara[3][4], double trans[3][4] )throw(cv::Exception);
			static double norm( double a, double b, double c );
			static double dot(  double a1, double a2, double a3,
				double b1, double b2, double b3 );
			
			
			static void rotateXAxis(Mat &rotation);

	};


/**
 * Creates an ar marker with the id specified. hamming code is employed 
 * There are a total of 5 rows of 5 cols each
 * Each row encodes a total of 2 bits, so there are 2^10 bits:(0-1023) 
 * Hamming code is employed for error detection/correction
 * The least significative bytes are first (from left-up to to right-bottom)
 * Example: id = 110
 * bin code: 00 01 10 11 10
 * Marker (least significative bit is the leftmost)
 * Note: The first bit, is the inverse of the hamming parity. This avoids the 0 0 0 0 0 to be valid
 * 1st row encodes 00: 1 0 0 0 0 : hex 0x10
 * 2nd row encodes 01: 1 0 1 1 1 : hex 0x17
 * 3nd row encodes 10: 0 1 0 0 1 : hex 0x09
 * 4th row encodes 11: 0 1 1 1 0 : hex 0x0e
 * 5th row encodes 10: 0 1 0 0 1 : hex 0x09
 */
Mat createMarker(int id,int size) throw (cv::Exception); 


};
#endif
