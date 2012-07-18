#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
using namespace cv;
using namespace std;
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
Mat createMarker(int id,int size) throw (cv::Exception)
{
  if (id>=1024) throw cv::Exception(9010,"id>=1024","createMarker",__FILE__,__LINE__);
  Mat marker(size,size, CV_8UC1);
  marker.setTo(Scalar(0));
  //for each line, create
  int swidth=size/7;
  int ids[4]={0x10,0x17,0x09,0x0e};
  for(int y=0;y<5;y++){
    int index=(id>>2*(4-y)) & 0x0003;
    int val=ids[index];
    for(int x=0;x<5;x++){
	Mat roi=marker(Rect((x+1)* swidth,(y+1)* swidth,swidth,swidth));
	if ( ( val>>4-x) & 0x0001 ) roi.setTo(Scalar(255));
	else roi.setTo(Scalar(0));
    }
  }
  return marker;
}
 
int main(int argc,char **argv)
{
try{
  if (argc!=4){
    cerr<<"Usage: <makerid(0:1023)> outfile.jpg sizeInPixels"<<endl;
    return -1;
  }
  Mat marker=createMarker(atoi(argv[1]),atoi(argv[3]));
  imwrite(argv[2],marker);
  
  
    
}
catch(std::exception &ex)
{
    cout<<ex.what()<<endl;
}

}
