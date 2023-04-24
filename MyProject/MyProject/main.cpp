#include <iostream>
#include <stdio.h>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(void)
{
	Mat img = imread("Lenna.png");
	imshow("img", img);
	waitKey(0);


	vector<Vec3f> circles;
	// https://diyver.tistory.com/102
	HoughCircles();
	return 0;
}