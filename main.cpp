#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <time.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv )
{
	Mat frame;
	time_t start;
	time_t end;
	double seconds;
	VideoCapture cap(0);	// open the default camera
	if(!cap.isOpened()) {	// check if we succeeded
		return -1;
	}
	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
	cout << "Frame size : " << dWidth << " x " << dHeight << endl;

	namedWindow("MyImage", WINDOW_AUTOSIZE);
	for(int i = 0;i<100;i++)
	{
		bool bSuccess = cap.read(frame); // get a new frame from camera
		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}
		imshow("MyImage", frame);
		if (waitKey(10) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}

	cap.release();
	return 0;
}
