
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	if( argc != 2)
	{
	cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
	return -1;
	}

	Mat image;
	image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file

	if(! image.data )                              // Check for invalid input
	{
	cout <<  "Could not open or find the image" << std::endl ;
	return -1;
	}

/*	namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
	imshow( "Display window", image );                   // Show our image inside it.

	waitKey(0);                                          // Wait for a keystroke in the window
	return 0;*/
	
	cvtColor(image, image, COLOR_BGRA2BGR);
	//capturing image here
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PXM_BINARY);
	compression_params.push_back(1);

	//writing as ppm image
	imwrite("Image_1.ppm", image, compression_params);
	return 0;
	
	
}
