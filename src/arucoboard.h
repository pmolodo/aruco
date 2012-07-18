#ifndef _Aruco_Board_
#define _Aruco_Board_
#include <opencv/cv.h>
#include "aruco.h"
using namespace cv;
namespace aruco
{
/**\brief This class defines a board with several markers.
* Board contains several markers so that they are more robustly detected
*/
class BoardConfiguration
{
public:
	/**
	*/
	BoardConfiguration();

	/**
	*/
	BoardConfiguration(const BoardConfiguration  &T);

	/**Saves the board info to a file
	*/
	void saveToFile(string sfile)throw (cv::Exception);
	/**Reads board info from a file
	*/
	void readFromFile(string sfile)throw (cv::Exception);

// private:
	Mat _markersId;// grid of marker ids. Represent the matrix of markers in the board
	int _markerSizePix, _markerDistancePix;//size in pixels of the marker side and the distance between markers
};
/**
*/
class Board:public vector<Marker>
{
	public:
	/**
	*/
	Board()
	{
	  Rvec.create(3,1,CV_32FC1);
	  Tvec.create(3,1,CV_32FC1);
	  for(int i=0;i<3;i++)
		  Tvec.at<float>(i,0)=Rvec.at<float>(i,0)=-999999;
	}

    /**Given the extrinsic camera parameters returns the GL_MODELVIEW matrix for opengl.
	* Setting this matrix, the reference corrdinate system will be set in this board
	 */
    void glGetModelViewMatrix(double modelview_matrix[16])throw(cv::Exception);

    
	BoardConfiguration conf;
	//matrices of rotation and translation respect to the camera
	Mat Rvec,Tvec;

	
};


/**\brief This class detects AR boards
*/
class ArBoardDetector
{
  public:

    /** Given the markers detected, determines if there is the board passed
    * @param detectedMarkers result provided by aruco::ArMarkerDetector
    * @param BConf the board you want to see if is present
    * @param Bdetected output information of the detected board
    * @param camMatrix camera matrix with intrinsics
    * @param distCoeff camera distorsion coeff
    * @param camMatrix intrinsic camera information.
    * @param distCoeff camera distorsion coefficient. If set Mat() if is assumed no camera distorion
    * @param markerSizeMeters size of the marker sides expressed in meters
    * @return value indicating  the  likelihood of having found the marker
    */
    float detect(const vector<Marker> &detectedMarkers,const  BoardConfiguration &BConf, Board &Bdetected, Mat camMatrix=Mat(),Mat distCoeff=Mat(), float markerSizeMeters=-1 )throw (cv::Exception);
  private:
    void rotateXAxis(Mat &rotation);
};



/**
 * Creates an ar board
 *
 */
Mat createBoard(Size gridSize,int MarkerSize,int MarkerDistance,unsigned int FirstMarkerID, BoardConfiguration& TInfo ) throw (cv::Exception);
};
#endif

