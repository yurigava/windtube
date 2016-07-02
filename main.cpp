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
	cap.set(CV_CAP_PROP_FPS, 150);

	time(&start);
	for(int i = 0;i<100;i++)
	{
		cap >> frame; // get a new frame from camera
		imshow("edges", frame);
	}
	time(&end);
	seconds = difftime(end, start);
	cout << "Tempo para 100: " << seconds << endl;
	cout << "Framerate: " << 100/seconds << endl;

	cap.release();
	return 0;
}
