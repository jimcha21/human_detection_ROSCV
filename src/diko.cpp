#include <opencv2/opencv.hpp>

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <iostream>     // std::cout
#include <algorithm>    // std::sort
#include <vector>       // std::vector
#include <time.h>
#include <math.h>

using namespace cv;
using namespace cv::ml;
using namespace std;

int main( int argc, char** argv )
{

    cv::CommandLineParser parser(argc, argv, "{help h|| show help message}"
            "{pd||pos_dir}{p||pos.lst}{nd||neg_dir}{n||neg.lst}");
    if (parser.has("help"))
    {
        parser.printMessage();
        exit(0);
    }
    vector< Mat > pos_lst;
    vector< Mat > full_neg_lst;
    vector< Mat > neg_lst;
    vector< Mat > gradient_lst;
    vector< int > labels;
/*    string pos_dir = "/media/jimcha/e90a1ec3-ed57-4c9c-8bff-a961bd2269f0/home/jimcha/trainHOG/INRIAPerson/";//parser.get<string>("pd");
    string pos = "Train/pos1.lst"; //parser.get<string>("p");
    string neg_dir = "/media/jimcha/e90a1ec3-ed57-4c9c-8bff-a961bd2269f0/home/jimcha/trainHOG/INRIAPerson/";//parser.get<string>("pd");
    string neg = "Train/neg1.lst";  //parser.get<string>("n");*/
    string pos_dir = "/home/jimcha/opencv/workspace/";//parser.get<string>("pd");
    string pos = "AerialDat/pos1.lst"; //parser.get<string>("p");
    string neg_dir = "/home/jimcha/opencv/workspace/";//parser.get<string>("pd");
    string neg = "AerialDat/neg1.lst";  //parser.get<string>("n");
    if( pos_dir.empty() || pos.empty() || neg_dir.empty() || neg.empty() )
    {
        cout << "Wrong number of parameters." << endl
            << "Usage: " << argv[0] << " --pd=pos_dir -p=pos.lst --nd=neg_dir -n=neg.lst" << endl
            << "example: " << argv[0] << " --pd=/INRIA_dataset/ -p=Train/pos.lst --nd=/INRIA_dataset/ -n=Train/neg.lst" << endl; 
        exit( -1 );
    }

}
