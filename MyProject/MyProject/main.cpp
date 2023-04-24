#include <iostream>
#include <stdio.h>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

using namespace std;
using namespace cv;

int main(void)
{
	Mat img = imread("Lenna.png");

	imshow("img", img);
	waitKey(0);
	return 0;
}