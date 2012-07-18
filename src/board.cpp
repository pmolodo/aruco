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
#include "board.h"
#include <fstream>
using namespace std;
using namespace cv;
namespace aruco {

/**
*
*
*/
BoardConfiguration::BoardConfiguration()
{
    _markerSizePix=_markerDistancePix=-1;
}
/**
*
*
*/
BoardConfiguration::BoardConfiguration(const BoardConfiguration  &T)
{
    _markersId=T._markersId;
    _markerSizePix=T._markerSizePix;
    _markerDistancePix=T._markerDistancePix;

}
/**
*
*
*/
void BoardConfiguration::saveToFile(string sfile)throw (cv::Exception)
{
    ofstream file(sfile.c_str());
    if (!file) throw cv::Exception(9190,"File could not be opened for writing :"+sfile,"BoardConfiguration::saveToFile",__FILE__,__LINE__);
    file<<"ArucoBoard 1.0"<<endl;
    file<<_markersId.size().height<<" "<<_markersId.size().width<<" ";
    for (  int i=0;i<_markersId.size().height;i++) {
        for (  int j=0;j<_markersId.size().width;j++)
            file<<_markersId.at<int>(i,j)<<" ";
    }
    file<<endl;
    file<< _markerSizePix<<" "<<_markerDistancePix<<" "<<endl;

}
/**
*
*
*/
void BoardConfiguration::readFromFile(string sfile)throw (cv::Exception)
{
    ifstream file(sfile.c_str());
    if (!file)  throw cv::Exception(9191,"File could not be opened fir reading :"+sfile,"BoardConfiguration::readFromFile",__FILE__,__LINE__);
    string sig,ver;
    file>>sig>>ver;
    if (sig!="ArucoBoard") throw cv::Exception(9191,"Invalid file type :"+sfile,"BoardConfiguration::readFromFile",__FILE__,__LINE__);
    if (ver!="1.0") throw cv::Exception(9191,"Invalid file version :"+sfile,"BoardConfiguration::readFromFile",__FILE__,__LINE__);
    Size size;
    file>>size.height>>size.width;//read size
    _markersId.create(size,CV_32SC1);
    for (  int i=0;i<size.height;i++)
        for (  int j=0;j<size.width;j++)
            file>>_markersId.at<int>(i,j);

    file>> _markerSizePix>>_markerDistancePix;
}

}
