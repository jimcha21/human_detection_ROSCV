#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <iostream>
#include <string>

using namespace cv;
using namespace cv::ximgproc;
using namespace std;

int main(int argc, char const *argv[])
{
    printf("%d\n", argc);
    if( argc < 2)
    {
     cout <<" Usage: display_image left right ijamges" << endl;
     return -1;
    }

    Mat left,right;
    left = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file
    right = imread(argv[2], CV_LOAD_IMAGE_COLOR);   // Read the file

    if(!right.data  || !left.data)                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }    

    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", right );                   // Show our image inside it.
    namedWindow( "Display window2", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window2", left );                   // Show our image inside it.
    waitKey(0);

	printf("Hello world\n");
	return 0;
}