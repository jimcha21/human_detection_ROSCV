#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include <string>
#include <fstream>
#include <iostream>     // std::cout
#include <algorithm>    // std::sort
#include <vector>       // std::vector
#include <time.h>
#include <math.h>
#include <unistd.h>

#define GetCurrentDir getcwd

using namespace cv;
using namespace std;

void load_images( const string & prefix, const string & filename, vector< Mat > & img_lst)
{
	string line;
    ifstream file;

    file.open( (prefix+filename).c_str() );
    if( !file.is_open() )
    {
        cerr << "Unable to open the list of images from " << filename << " filename." << endl;
        exit( -1 );
    }

    bool end_of_parsing = false;
    while( !end_of_parsing )
    {
        getline( file, line );
        if( line.empty() ) // no more file to read
        {
            end_of_parsing = true;
            break;
        }
		Mat img;
		if(!filename.compare("pos.lst")){
			img = imread( (prefix+"pos/"+line).c_str() ); // load the image
       	}else{
			img = imread( (prefix+"neg/"+line).c_str() ); // load the image
      	}
        //cout << "mpe " << (prefix+line).c_str() << endl;
		if( img.empty() ) // invalid image, just skip it.
            continue;

/*
        imshow( "image", img );
        waitKey( 0 );
*/
		
        img_lst.push_back( img.clone() );
    }
}


void sample_neg( const vector< Mat > & full_neg_lst, vector< Mat > & neg_lst, const Size & size )
{
	cout<<"negs"<<endl;
    Rect box;
    box.width = size.width;
    box.height = size.height;

    const int size_x = box.width;
    const int size_y = box.height;

    srand( (unsigned int)time( NULL ) );

    vector< Mat >::const_iterator img = full_neg_lst.begin();
    vector< Mat >::const_iterator end = full_neg_lst.end();
    for( ; img != end ; ++img )
    {
		if(img->cols<= size.width && img->rows<= size.height){
			Mat roi = (*img);
			neg_lst.push_back( roi.clone() );
			imshow( "img", roi.clone() );
			waitKey(0 );
			continue;
		}//else
		
        box.x = rand() % (img->cols - size_x);
        box.y = rand() % (img->rows - size_y);
        Mat roi = (*img)(box);
        neg_lst.push_back( roi.clone() );


/*		imshow( "img", roi.clone() );
		waitKey(0);
*/
		
    }
}

int main( int argc,char *argv[],char *envp[]  )
{
	int count;  

	string current_dir=argv[0];	
	
	if(argc!=3){
		cout<<"Please insert dataset dirs. /$ .. <pos_dir> <neg_dir>"<<endl;
		return 0;
	}	
		
	vector< Mat > pos_lst;
	
	vector< Mat > full_neg_lst;
	vector< Mat > neg_lst;
	vector< Mat > gradient_lst;
	vector< int > labels;
	
    string pos_dir=argv[1];
	string neg_dir=argv[2];

	load_images(pos_dir,"pos.lst",pos_lst);
	labels.assign( pos_lst.size(), +1 );
	const unsigned int old = (unsigned int)labels.size();
	
	load_images( neg_dir, "neg.lst", full_neg_lst );
	sample_neg( full_neg_lst, neg_lst, Size( 64,128 ) );
    labels.insert( labels.end(), neg_lst.size(), -1 );
	printf("Negs %d %d\n",(int)pos_lst.size(),(int)neg_lst.size());
    
	//sample_neg( full_neg_lst, neg_lst, Size( 64,128 ) );
}


