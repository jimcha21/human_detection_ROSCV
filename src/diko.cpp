#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"

#include "opencv2/ml/ml.hpp"

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
//using namespace ml;
using namespace std;

void train_svm( const vector< Mat > & gradient_lst, const vector< int > & labels );


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
void convert_to_ml(const std::vector< cv::Mat > & train_samples, cv::Mat& trainData )
{
    //--Convert data
    const int rows = (int)train_samples.size();
    const int cols = (int)std::max( train_samples[0].cols, train_samples[0].rows );
    cv::Mat tmp(1, cols, CV_32FC1); //< used for transposition if needed
    trainData = cv::Mat(rows, cols, CV_32FC1 );
    vector< Mat >::const_iterator itr = train_samples.begin();
    vector< Mat >::const_iterator end = train_samples.end();
    for( int i = 0 ; itr != end ; ++itr, ++i )
    {
        CV_Assert( itr->cols == 1 ||
            itr->rows == 1 );
        if( itr->cols == 1 )
        {
            transpose( *(itr), tmp );
            tmp.copyTo( trainData.row( i ) );
        }
        else if( itr->rows == 1 )
        {
            itr->copyTo( trainData.row( i ) );
        }
    }
}

//checks the params pls --> http://docs.opencv.org/2.4.13/modules/ml/doc/support_vector_machines.html?highlight=cvsvm
	void train_svm( const vector< Mat > & gradient_lst, const vector< int > & labels )
{

    Mat train_data;
    convert_to_ml( gradient_lst, train_data );

    clog << "Start training...";
	CvSVM svm;
	
	
    //Ptr<cvSVM> svm = cvSVM::create();
	
	CvSVMParams params;
    params.svm_type=CvSVM::EPS_SVR;
	params.kernel_type=CvSVM::LINEAR;
	params.degree=3;
	params.gamma=0;
	params.coef0=0.0;
	params.C=0.01;
	params.nu=0.5;
	params.p=0.1;
	params.class_weights=0;
	params.term_crit=cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
	
	//svm->setCoef0(0.0);
    //svm->setDegree(3);
    //svm->setTermCriteria(TermCriteria( CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-3 ));
    //svm->setGamma(0);
    //svm->setKernel(SVM::LINEAR);
    //svm->setNu(0.5);
    //svm->setP(0.1); // for EPSILON_SVR, epsilon in loss function?
    //svm->setC(0.01); // From paper, soft classifier
    
	//svm->setType(SVM::EPS_SVR); // C_SVC; // EPSILON_SVR; // may be also NU_SVR; // do regression task
    
    svm.train(train_data, Mat(labels), Mat(), Mat(), params);
	//svm->train(train_data, ROW_SAMPLE, Mat(labels));
    
	clog << "...[done]" << endl;

	svm.save( "my_people_detector.yml" );


/*	GMLwriter gml_writer;   
gml_writer.write(argv[3], Users, edges);*/

	CV_WRAP CvSVM svm2;
	CvStatModel model;	
	svm2.load("my_people_detector.yml");

	//float* hog_detector;
	const float* float_mat =svm2.get_support_vector(0);
	cout<<float_mat[3779]<<" "<<float_mat[3780]<< endl;
	
	 //this is protected, but can access due to inheritance rules 
	 // const CvSVMDecisionFunc *dec = CvSVM::decision_func;

	//cout<<svm::decision_func.rho<<endl;
	vector<float> hog_detector;
	for(int i=0; i<svm2.get_var_count();i++){
		hog_detector.push_back(float_mat[i]);
	}
	//float rho=svm2.get_params().
	//hog_detector.push_back()
	cout<<hog_detector.size()<<endl;
	//cout<<"o arithmos einain "<<hog_detector[0]<<" "<<hog_detector[1]<< "kai to size einai "<<hog.detector.size()<<endl;
	
}



void compute_hog( const vector< Mat > & img_lst, vector< Mat > & gradient_lst, const Size & size )
{
    HOGDescriptor hog;
    hog.winSize = size;
    Mat gray;
    vector< Point > location;
    vector< float > descriptors;

    vector< Mat >::const_iterator img = img_lst.begin();
    vector< Mat >::const_iterator end = img_lst.end();
    int c=0;
    for( ; img != end ; ++img )
    {
        c++;
        cvtColor( *img, gray, COLOR_BGR2GRAY );
        hog.compute( gray, descriptors, Size( 8, 8 ), Size( 0, 0 ), location );
        gradient_lst.push_back( Mat( descriptors ).clone() );
/*
        imshow( "gradient", get_hogdescriptor_visu( img->clone(), descriptors, size ) );
        waitKey( 0 );*/

    }
    printf("%d\n", c);
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
/*			imshow( "img", roi.clone() );
			waitKey(0 );*/
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
	CV_Assert( old < labels.size() );
	
	compute_hog( pos_lst, gradient_lst, Size( 64,128 ) );
	compute_hog( neg_lst, gradient_lst, Size( 64,128 ) );
	train_svm( gradient_lst, labels );
	
}


