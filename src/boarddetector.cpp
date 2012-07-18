/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
#include "boarddetector.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cstdlib>
#include <ctime>
#include <cassert>
#include <fstream>
using namespace std;
using namespace cv;
namespace aruco
{
/**
*
*
*/
float BoardDetector::detect(const vector<Marker> &detectedMarkers,const  BoardConfiguration &BConf, Board &Bdetected,const CameraParameters &cp, float markerSizeMeters  )throw (cv::Exception)
{
    return detect(detectedMarkers, BConf,Bdetected,cp.CameraMatrix,cp.Distorsion,markerSizeMeters);
}
/**
*
*
*/
float BoardDetector::detect(const vector<Marker> &detectedMarkers,const  BoardConfiguration &BConf, Board &Bdetected, Mat camMatrix,Mat distCoeff,float markerSizeMeters)throw (cv::Exception)
{
// cout<<"markerSizeMeters="<<markerSizeMeters<<endl;
    ///find among detected markers these that belong to the board configuration
    Mat detected(BConf._markersId.size(),CV_32SC1); //stores the indices of the makers
    detected.setTo(Scalar(-1));//-1 mean not detected
    int nMarkInBoard=0;//total number of markers detected
    for (unsigned int i=0;i<detectedMarkers.size();i++) {
        bool found=false;
        int id=detectedMarkers[i].id;
        //find it
        for (  int j=0;j<detected.size().height && ! found;j++)
            for (  int k=0;k<detected.size().width && ! found;k++)
                if ( BConf._markersId.at<int>(j,k)==id) {
                    detected.at<int>(j,k)=i;
                    nMarkInBoard++;
                    found=true;
                    Bdetected.push_back(detectedMarkers[i]);
                    if (markerSizeMeters>0)
                        Bdetected.back().ssize=markerSizeMeters;
                }
    }
    Bdetected.conf=BConf;
    if (markerSizeMeters!=-1)
        Bdetected.markerSizeMeters=markerSizeMeters;
//calculate extrinsic if there is information for that
    if (camMatrix.rows!=0 && markerSizeMeters>0 && nMarkInBoard>=1) {

        // now, create the matrices for finding the extrinsics
        Mat objPoints(4*nMarkInBoard,3,CV_32FC1);
        Mat imagePoints(4*nMarkInBoard,2,CV_32FC1);
        //size in meters of inter-marker distance
        double markerDistanceMeters= double(BConf._markerDistancePix) * markerSizeMeters / double(BConf._markerSizePix);



        int currIndex=0;
        for (  int y=0;y<detected.size().height;y++)
            for (  int x=0;x<detected.size().width;x++) {
                if (  detected.at<int>(y,x)!=-1 ) {

                    vector<Point2f> points =detectedMarkers[ detected.at<int>(y,x) ];
                    //set first image points
                    for (int p=0;p<4;p++) {
                        imagePoints.at<float>(currIndex+p,0)=points[p].x;
                        imagePoints.at<float>(currIndex+p,1)=points[p].y;
                    }

                    //tranaltion to make the Ref System be in center
                    float TX=-(  ((detected.size().height-1)*(markerDistanceMeters+markerSizeMeters) +markerSizeMeters) /2) ;
                    float TY=-(  ((detected.size().width-1)*(markerDistanceMeters+markerSizeMeters) +markerSizeMeters)/2);
                    //points in real refernce system. We se the center in the bottom-left corner
                    float AY=x*(markerDistanceMeters+markerSizeMeters ) +TY;
                    float AX=y*(markerDistanceMeters+markerSizeMeters )+TX;
                    objPoints.at<float>( currIndex,0)= AX;
                    objPoints.at<float>( currIndex,1)= AY;
                    objPoints.at<float>( currIndex,2)= 0;
                    objPoints.at<float>( currIndex+1,0)= AX;
                    objPoints.at<float>( currIndex+1,1)= AY+markerSizeMeters;
                    objPoints.at<float>( currIndex+1,2)= 0;
                    objPoints.at<float>( currIndex+2,0)= AX+markerSizeMeters;
                    objPoints.at<float>( currIndex+2,1)= AY+markerSizeMeters;
                    objPoints.at<float>( currIndex+2,2)= 0;
                    objPoints.at<float>( currIndex+3,0)= AX+markerSizeMeters;
                    objPoints.at<float>( currIndex+3,1)= AY;
                    objPoints.at<float>( currIndex+3,2)= 0;
                    currIndex+=4;
                }
            }

        CvMat cvCamMatrix=camMatrix;
        CvMat cvDistCoeffs;
        Mat zeros=Mat::zeros(4,1,CV_32FC1);
        if (distCoeff.rows>=4)  cvDistCoeffs=distCoeff;
        else  cvDistCoeffs=zeros;
        CvMat cvImgPoints=imagePoints;
        CvMat cvObjPoints=objPoints;

        CvMat cvRvec=Bdetected.Rvec;
        CvMat cvTvec=Bdetected.Tvec;
        cvFindExtrinsicCameraParams2(&cvObjPoints, &cvImgPoints, &cvCamMatrix, &cvDistCoeffs,&cvRvec,&cvTvec);
        //now, rotate 90 deg in X so that Y axis points up
        rotateXAxis(Bdetected.Rvec);
        //cout<<Bdetected.Rvec.at<float>(0,0)<<" "<<Bdetected.Rvec.at<float>(1,0)<<Bdetected.Rvec.at<float>(2,0)<<endl;
        //cout<<Bdetected.Tvec.at<float>(0,0)<<" "<<Bdetected.Tvec.at<float>(1,0)<<Bdetected.Tvec.at<float>(2,0)<<endl;
    }
    return double(nMarkInBoard)/double( BConf._markersId.size().width*BConf._markersId.size().height);
}

void BoardDetector::rotateXAxis(Mat &rotation)
{
    cv::Mat R(3,3,CV_32FC1);
    Rodrigues(rotation, R);
    //create a rotation matrix for x axis
    cv::Mat RX=cv::Mat::eye(3,3,CV_32FC1);
    float angleRad=M_PI/2;
    RX.at<float>(1,1)=cos(angleRad);
    RX.at<float>(1,2)=-sin(angleRad);
    RX.at<float>(2,1)=sin(angleRad);
    RX.at<float>(2,2)=cos(angleRad);
    //now multiply
    R=R*RX;
    //finally, the the rodrigues back
    Rodrigues(R,rotation);

}


void Board::glGetModelViewMatrix(   double modelview_matrix[16])throw(cv::Exception)
{
    //check if paremeters are valid
    bool invalid=false;
    for (int i=0;i<3 && !invalid ;i++)
    {
        if (Tvec.at<float>(i,0)!=-999999) invalid|=false;
        if (Rvec.at<float>(i,0)!=-999999) invalid|=false;
    }
    if (invalid) throw cv::Exception(9002,"extrinsic parameters are not set","Marker::getModelViewMatrix",__FILE__,__LINE__);
    Mat Rot(3,3,CV_32FC1),Jacob;
    Rodrigues(Rvec, Rot, Jacob);

    double para[3][4];
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++) para[i][j]=Rot.at<float>(i,j);
    //now, add the translation
    para[0][3]=Tvec.at<float>(0,0);
    para[1][3]=Tvec.at<float>(1,0);
    para[2][3]=Tvec.at<float>(2,0);
    double scale=1;

    modelview_matrix[0 + 0*4] = para[0][0];
    // R1C2
    modelview_matrix[0 + 1*4] = para[0][1];
    modelview_matrix[0 + 2*4] = para[0][2];
    modelview_matrix[0 + 3*4] = para[0][3];
    // R2
    modelview_matrix[1 + 0*4] = para[1][0];
    modelview_matrix[1 + 1*4] = para[1][1];
    modelview_matrix[1 + 2*4] = para[1][2];
    modelview_matrix[1 + 3*4] = para[1][3];
    // R3
    modelview_matrix[2 + 0*4] = -para[2][0];
    modelview_matrix[2 + 1*4] = -para[2][1];
    modelview_matrix[2 + 2*4] = -para[2][2];
    modelview_matrix[2 + 3*4] = -para[2][3];
    modelview_matrix[3 + 0*4] = 0.0;
    modelview_matrix[3 + 1*4] = 0.0;
    modelview_matrix[3 + 2*4] = 0.0;
    modelview_matrix[3 + 3*4] = 1.0;
    if (scale != 0.0)
    {
        modelview_matrix[12] *= scale;
        modelview_matrix[13] *= scale;
        modelview_matrix[14] *= scale;
    }


}


/****
 * 
 */
void Board::OgreGetPoseParameters(double position[3], double orientation[4]) throw(cv::Exception)
{
    //check if paremeters are valid
    bool invalid=false;
    for (int i=0;i<3 && !invalid ;i++)
    {
        if (Tvec.at<float>(i,0)!=-999999) invalid|=false;
        if (Rvec.at<float>(i,0)!=-999999) invalid|=false;
    }
    if (invalid) throw cv::Exception(9003,"extrinsic parameters are not set","Marker::getModelViewMatrix",__FILE__,__LINE__);  
    
    // calculate position vector
    position[0] = -Tvec.ptr<float>(0)[0];
    position[1] = -Tvec.ptr<float>(0)[1];
    position[2] = +Tvec.ptr<float>(0)[2];
    
    // now calculare orientation quaternion
    cv::Mat Rot(3,3,CV_32FC1);
    cv::Rodrigues(Rvec, Rot);
    
    // calculate axes for quaternion
    double stAxes[3][3];
    // x axis
    stAxes[0][0] = -Rot.at<float>(0,0);
    stAxes[0][1] = -Rot.at<float>(1,0);
    stAxes[0][2] = +Rot.at<float>(2,0);
    // y axis
    stAxes[1][0] = -Rot.at<float>(0,1);
    stAxes[1][1] = -Rot.at<float>(1,1);
    stAxes[1][2] = +Rot.at<float>(2,1);    
    // for z axis, we use cross product
    stAxes[2][0] = stAxes[0][1]*stAxes[1][2] - stAxes[0][2]*stAxes[1][1];
    stAxes[2][1] = - stAxes[0][0]*stAxes[1][2] + stAxes[0][2]*stAxes[1][0];
    stAxes[2][2] = stAxes[0][0]*stAxes[1][1] - stAxes[0][1]*stAxes[1][0];
    
    // transposed matrix
    double axes[3][3];
    axes[0][0] = stAxes[0][0];
    axes[1][0] = stAxes[0][1];
    axes[2][0] = stAxes[0][2];
    
    axes[0][1] = stAxes[1][0];
    axes[1][1] = stAxes[1][1];
    axes[2][1] = stAxes[1][2];  
    
    axes[0][2] = stAxes[2][0];
    axes[1][2] = stAxes[2][1];
    axes[2][2] = stAxes[2][2];    
    
    // Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
    // article "Quaternion Calculus and Fast Animation".
    double fTrace = axes[0][0]+axes[1][1]+axes[2][2];
    double fRoot;
      
    if ( fTrace > 0.0 )
    {
	// |w| > 1/2, may as well choose w > 1/2
	fRoot = sqrt(fTrace + 1.0);  // 2w
	orientation[0] = 0.5*fRoot;
	fRoot = 0.5/fRoot;  // 1/(4w)
	orientation[1] = (axes[2][1]-axes[1][2])*fRoot;
	orientation[2] = (axes[0][2]-axes[2][0])*fRoot;
	orientation[3] = (axes[1][0]-axes[0][1])*fRoot;
    }
    else
    {
	// |w| <= 1/2
	static unsigned int s_iNext[3] = { 1, 2, 0 };
	unsigned int i = 0;
	if ( axes[1][1] > axes[0][0] )
	    i = 1;
	if ( axes[2][2] > axes[i][i] )
	    i = 2;
	unsigned int j = s_iNext[i];
	unsigned int k = s_iNext[j];

	fRoot = sqrt(axes[i][i]-axes[j][j]-axes[k][k] + 1.0);
	double* apkQuat[3] = { &orientation[1], &orientation[2], &orientation[3] };
	*apkQuat[i] = 0.5*fRoot;
	fRoot = 0.5/fRoot;
	orientation[0] = (axes[k][j]-axes[j][k])*fRoot;
	*apkQuat[j] = (axes[j][i]+axes[i][j])*fRoot;
	*apkQuat[k] = (axes[k][i]+axes[i][k])*fRoot;
    }
    
    
}

};

