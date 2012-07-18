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
#ifndef _Aruco_board_h
#define _Aruco_board_h
#include <opencv/cv.h>
#include <string>
#include <vector>
#include "exports.h"
#include "marker.h"
using namespace std;
namespace aruco{
/**\brief This class defines a board with several markers.
* Board contains several markers so that they are more robustly detected
*/
 
class ARUCO_EXPORTS  BoardConfiguration
{
public:
    cv::Mat _markersId;// grid of marker ids. Represent the matrix of (integers) markers in the board
    int _markerSizePix, _markerDistancePix;//size in pixels of the marker side and the distance between markers

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

};

/**
*/
class ARUCO_EXPORTS Board:public vector<Marker>
{
public:
    BoardConfiguration conf;
    //matrices of rotation and translation respect to the camera
    cv::Mat Rvec,Tvec;
    float markerSizeMeters;
    /**
    */
    Board()
    {
        Rvec.create(3,1,CV_32FC1);
        Tvec.create(3,1,CV_32FC1);
        for (int i=0;i<3;i++)
            Tvec.at<float>(i,0)=Rvec.at<float>(i,0)=-999999;
	markerSizeMeters=-1;
    }

    /**Given the extrinsic camera parameters returns the GL_MODELVIEW matrix for opengl.
    * Setting this matrix, the reference corrdinate system will be set in this board
     */
    void glGetModelViewMatrix(double modelview_matrix[16])throw(cv::Exception);
    
    /**
     * Returns position vector and orientation quaternion for an Ogre scene node or entity.
     * 	Use:
     * ...
     * Ogre::Vector3 ogrePos (position[0], position[1], position[2]);
     * Ogre::Quaternion  ogreOrient (orientation[0], orientation[1], orientation[2], orientation[3]);
     * mySceneNode->setPosition( ogrePos  );
     * mySceneNode->setOrientation( ogreOrient  );
     * ...
     */
    void OgreGetPoseParameters(  double position[3], double orientation[4] )throw(cv::Exception);    



};
}

#endif
