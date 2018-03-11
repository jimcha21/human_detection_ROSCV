#include <cv.h>
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <iostream>
#include <string>
#include <cstdlib>
#include <fstream>



using namespace cv;

/// Global Variables
const int alpha_slider_max = 100;
int alpha_slider;
double alpha;
double beta;

/// Matrices to store images
Mat src1;
Mat src2;
Mat dst;

/**
 * @function on_trackbar
 * @brief Callback for trackbar
 */
void on_trackbar( int, void* )
{
 alpha = (double) alpha_slider/alpha_slider_max ;
 beta = ( 1.0 - alpha );

 addWeighted( src1, alpha, src2, beta, 0.0, dst);

 imshow( "Linear Blend", dst );
}

int main( int argc, char** argv )
{
 /// Read image ( same size, same type )
 src1 = imread("agg_rl");
 src2 = imread("../../images/WindowsLogo.jpg");

 if( !src1.data ) { printf("Error loading src1 \n"); return -1; }
 if( !src2.data ) { printf("Error loading src2 \n"); return -1; }

 /// Initialize values
 alpha_slider = 0;

 /// Create Windows
 namedWindow("Linear Blend", 1);

 /// Create Trackbars
 char TrackbarName[50];
 sprintf( TrackbarName, "Alpha x %d", alpha_slider_max );

 createTrackbar( TrackbarName, "Linear Blend", &alpha_slider, alpha_slider_max, on_trackbar );

 /// Show some stuff
 on_trackbar( alpha_slider, 0 );

 /// Wait until user press some key
 waitKey(0);
 return 0;
}