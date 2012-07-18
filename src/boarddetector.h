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
#ifndef _Aruco_BoardDetector_H
#define _Aruco_BoardDetector_H
#include <opencv/cv.h>
#include "exports.h"
#include "board.h"
#include "cameraparameters.h"
using namespace std;

namespace aruco
{

/**\brief This class detects AR boards
*/
class ARUCO_EXPORTS  BoardDetector
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
    float detect(const vector<Marker> &detectedMarkers,const  BoardConfiguration &BConf, Board &Bdetected, cv::Mat camMatrix=cv::Mat(),cv::Mat distCoeff=cv::Mat(), float markerSizeMeters=-1 )throw (cv::Exception);
    float detect(const vector<Marker> &detectedMarkers,const  BoardConfiguration &BConf, Board &Bdetected,const CameraParameters &cp, float markerSizeMeters=-1 )throw (cv::Exception);
private:
    void rotateXAxis(cv::Mat &rotation);
};

};
#endif

