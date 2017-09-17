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

void get_svm_detector(const Ptr<SVM>& svm, vector< float > & hog_detector );
void convert_to_ml(const std::vector< cv::Mat > & train_samples, cv::Mat& trainData );
void load_images( const string & prefix, const string & filename, vector< Mat > & img_lst );
void sample_neg( const vector< Mat > & full_neg_lst, vector< Mat > & neg_lst, const Size & size );
Mat get_hogdescriptor_visu(const Mat& color_origImg, vector<float>& descriptorValues, const Size & size );
void compute_hog( const vector< Mat > & img_lst, vector< Mat > & gradient_lst, const Size & size );
void train_svm( const vector< Mat > & gradient_lst, const vector< int > & labels );
void draw_locations( Mat & img, const vector< Rect > & locations, const Scalar & color );
void test_it( const Size & size );
std::vector< std::vector<int> > people;
std::vector< std::vector<int> > paths;

void get_svm_detector(const Ptr<SVM>& svm, vector< float > & hog_detector )
{
    // get the support vectors
    Mat sv = svm->getSupportVectors();
    const int sv_total = sv.rows;
    // get the decision function
    Mat alpha, svidx;
    double rho = svm->getDecisionFunction(0, alpha, svidx);

    CV_Assert( alpha.total() == 1 && svidx.total() == 1 && sv_total == 1 );
    CV_Assert( (alpha.type() == CV_64F && alpha.at<double>(0) == 1.) ||
               (alpha.type() == CV_32F && alpha.at<float>(0) == 1.f) );
    CV_Assert( sv.type() == CV_32F );
    hog_detector.clear();

    hog_detector.resize(sv.cols + 1);
    memcpy(&hog_detector[0], sv.ptr(), sv.cols*sizeof(hog_detector[0]));
    hog_detector[sv.cols] = (float)-rho;
}


/*
* Convert training/testing set to be used by OpenCV Machine Learning algorithms.
* TrainData is a matrix of size (#samples x max(#cols,#rows) per samples), in 32FC1.
* Transposition of samples are made if needed.
*/
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

void load_images( const string & prefix, const string & filename, vector< Mat > & img_lst )
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
        Mat img = imread( (prefix+line).c_str() ); // load the image
        if( img.empty() ) // invalid image, just skip it.
            continue;
#ifdef _DEBUG
        imshow( "image", img );
        waitKey( 10 );
#endif
        img_lst.push_back( img.clone() );
    }
}

void sample_neg( const vector< Mat > & full_neg_lst, vector< Mat > & neg_lst, const Size & size )
{
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
        box.x = rand() % (img->cols - size_x);
        box.y = rand() % (img->rows - size_y);
        Mat roi = (*img)(box);
        neg_lst.push_back( roi.clone() );
#ifdef _DEBUG
        imshow( "img", roi.clone() );
        waitKey( 10 );
#endif
    }
}

// From http://www.juergenwiki.de/work/wiki/doku.php?id=public:hog_descriptor_computation_and_visualization
Mat get_hogdescriptor_visu(const Mat& color_origImg, vector<float>& descriptorValues, const Size & size )
{
    const int DIMX = size.width;
    const int DIMY = size.height;
    float zoomFac = 3;
    Mat visu;
    resize(color_origImg, visu, Size( (int)(color_origImg.cols*zoomFac), (int)(color_origImg.rows*zoomFac) ) );

    int cellSize        = 8;
    int gradientBinSize = 9;
    float radRangeForOneBin = (float)(CV_PI/(float)gradientBinSize); // dividing 180 into 9 bins, how large (in rad) is one bin?

    // prepare data structure: 9 orientation / gradient strenghts for each cell
    int cells_in_x_dir = DIMX / cellSize;
    int cells_in_y_dir = DIMY / cellSize;
    float*** gradientStrengths = new float**[cells_in_y_dir];
    int** cellUpdateCounter   = new int*[cells_in_y_dir];
    for (int y=0; y<cells_in_y_dir; y++)
    {
        gradientStrengths[y] = new float*[cells_in_x_dir];
        cellUpdateCounter[y] = new int[cells_in_x_dir];
        for (int x=0; x<cells_in_x_dir; x++)
        {
            gradientStrengths[y][x] = new float[gradientBinSize];
            cellUpdateCounter[y][x] = 0;

            for (int bin=0; bin<gradientBinSize; bin++)
                gradientStrengths[y][x][bin] = 0.0;
        }
    }

    // nr of blocks = nr of cells - 1
    // since there is a new block on each cell (overlapping blocks!) but the last one
    int blocks_in_x_dir = cells_in_x_dir - 1;
    int blocks_in_y_dir = cells_in_y_dir - 1;

    // compute gradient strengths per cell
    int descriptorDataIdx = 0;
    int cellx = 0;
    int celly = 0;

    for (int blockx=0; blockx<blocks_in_x_dir; blockx++)
    {
        for (int blocky=0; blocky<blocks_in_y_dir; blocky++)
        {
            // 4 cells per block ...
            for (int cellNr=0; cellNr<4; cellNr++)
            {
                // compute corresponding cell nr
                cellx = blockx;
                celly = blocky;
                if (cellNr==1) celly++;
                if (cellNr==2) cellx++;
                if (cellNr==3)
                {
                    cellx++;
                    celly++;
                }

                for (int bin=0; bin<gradientBinSize; bin++)
                {
                    float gradientStrength = descriptorValues[ descriptorDataIdx ];
                    descriptorDataIdx++;

                    gradientStrengths[celly][cellx][bin] += gradientStrength;

                } // for (all bins)


                // note: overlapping blocks lead to multiple updates of this sum!
                // we therefore keep track how often a cell was updated,
                // to compute average gradient strengths
                cellUpdateCounter[celly][cellx]++;

            } // for (all cells)


        } // for (all block x pos)
    } // for (all block y pos)


    // compute average gradient strengths
    for (celly=0; celly<cells_in_y_dir; celly++)
    {
        for (cellx=0; cellx<cells_in_x_dir; cellx++)
        {

            float NrUpdatesForThisCell = (float)cellUpdateCounter[celly][cellx];

            // compute average gradient strenghts for each gradient bin direction
            for (int bin=0; bin<gradientBinSize; bin++)
            {
                gradientStrengths[celly][cellx][bin] /= NrUpdatesForThisCell;
            }
        }
    }

    // draw cells
    for (celly=0; celly<cells_in_y_dir; celly++)
    {
        for (cellx=0; cellx<cells_in_x_dir; cellx++)
        {
            int drawX = cellx * cellSize;
            int drawY = celly * cellSize;

            int mx = drawX + cellSize/2;
            int my = drawY + cellSize/2;

            rectangle(visu, Point((int)(drawX*zoomFac), (int)(drawY*zoomFac)), Point((int)((drawX+cellSize)*zoomFac), (int)((drawY+cellSize)*zoomFac)), Scalar(100,100,100), 1);

            // draw in each cell all 9 gradient strengths
            for (int bin=0; bin<gradientBinSize; bin++)
            {
                float currentGradStrength = gradientStrengths[celly][cellx][bin];

                // no line to draw?
                if (currentGradStrength==0)
                    continue;

                float currRad = bin * radRangeForOneBin + radRangeForOneBin/2;

                float dirVecX = cos( currRad );
                float dirVecY = sin( currRad );
                float maxVecLen = (float)(cellSize/2.f);
                float scale = 2.5; // just a visualization scale, to see the lines better

                // compute line coordinates
                float x1 = mx - dirVecX * currentGradStrength * maxVecLen * scale;
                float y1 = my - dirVecY * currentGradStrength * maxVecLen * scale;
                float x2 = mx + dirVecX * currentGradStrength * maxVecLen * scale;
                float y2 = my + dirVecY * currentGradStrength * maxVecLen * scale;

                // draw gradient visualization
                line(visu, Point((int)(x1*zoomFac),(int)(y1*zoomFac)), Point((int)(x2*zoomFac),(int)(y2*zoomFac)), Scalar(0,255,0), 1);

            } // for (all bins)

        } // for (cellx)
    } // for (celly)


    // don't forget to free memory allocated by helper data structures!
    for (int y=0; y<cells_in_y_dir; y++)
    {
        for (int x=0; x<cells_in_x_dir; x++)
        {
            delete[] gradientStrengths[y][x];
        }
        delete[] gradientStrengths[y];
        delete[] cellUpdateCounter[y];
    }
    delete[] gradientStrengths;
    delete[] cellUpdateCounter;

    return visu;

} // get_hogdescriptor_visu

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

void train_svm( const vector< Mat > & gradient_lst, const vector< int > & labels )
{

    Mat train_data;
    convert_to_ml( gradient_lst, train_data );

    clog << "Start training...";
    Ptr<SVM> svm = SVM::create();
    /* Default values to train SVM */
    svm->setCoef0(0.0);
    svm->setDegree(3);
    svm->setTermCriteria(TermCriteria( CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-3 ));
    svm->setGamma(0);
    svm->setKernel(SVM::LINEAR);
    svm->setNu(0.5);
    svm->setP(0.1); // for EPSILON_SVR, epsilon in loss function?
    svm->setC(0.01); // From paper, soft classifier
    svm->setType(SVM::EPS_SVR); // C_SVC; // EPSILON_SVR; // may be also NU_SVR; // do regression task
    svm->train(train_data, ROW_SAMPLE, Mat(labels));
    clog << "...[done]" << endl;

    svm->save( "my_people_detector.yml" );
}

void draw_locations( Mat & img, const vector< Rect > & locations, const Scalar & color )
{
    if( !locations.empty() )
    {
        vector< Rect >::const_iterator loc = locations.begin();
        vector< Rect >::const_iterator end = locations.end();
        for( ; loc != end ; ++loc )
        {
            rectangle( img, *loc, color, 2 );
        }
    }
}

vector<Rect> nms(const vector< Rect > & boxes, float overlapThresh){

    if(boxes.size()==0)
        return boxes;

    std::vector<int> pick,x1,y1,x2,y2,area,idxs;

    if( !boxes.empty() )
    {
        vector< Rect >::const_iterator loc = boxes.begin();
        vector< Rect >::const_iterator end = boxes.end();
        int c=0;
        for( ; loc != end ; ++loc )
        {
            x1.push_back(loc->x);
            y1.push_back(loc->y);
            x2.push_back(loc->x+loc->width);
            y2.push_back(loc->y+loc->height);
            //printf("%d %d %d %d\n",x1[c],y1[c],x2[c],y2[c]);
            area.push_back((x2[c] - x1[c] + 1) * (y2[c] - y1[c] + 1));            
            c++;
        }

        std::vector<int> tmp;
        for (int i = 0; i < y2.size(); ++i)
        {
            //printf("%d ",y2[i] );
            tmp.push_back(y2[i]);
        }
        
        std::sort(tmp.begin(), tmp.end());

        for (int ii = 0; ii < tmp.size(); ++ii)
        {
            for (int jj = 0; jj < y2.size(); ++jj)
            {
                if(tmp[ii]==y2[jj]){

                    bool dupli=false;
                    for (int zz = 0; zz < idxs.size(); ++zz)
                    {
                        if(jj==idxs[zz]){
                            dupli=true;
                            break;
                        }
                    }

                    if(dupli){
                        continue;
                    }else{
                        idxs.push_back(jj);
                        break;
                    }
                }
            }
        }

        while(idxs.size()>0){
            //printf("idx size %d\n",idxs.size() );
            int last=idxs.size()-1;
           
            int i = idxs[last];
            pick.push_back(i);


            std::vector<int> suppress;
            suppress.push_back(last);


            for (int pos = 0; pos < last; ++pos)
            {

                int j = idxs[pos];


                int xx1 = max(x1[i], x1[j]);
                int yy1 = max(y1[i], y1[j]);
                int xx2 = min(x2[i], x2[j]);
                int yy2 = min(y2[i], y2[j]);

                int w = max(0, xx2 - xx1 + 1);
                int h = max(0, yy2 - yy1 + 1);

                float overlap = float(w * h) / (float)area[j];

               
                //printf("%f\n", overlap);
                if (overlap > overlapThresh)
                    suppress.push_back(pos);

            }

            std::vector<int> tmp;
            for (int i = 0; i < idxs.size(); ++i)
            {
            //printf("%d ",y2[i] );
                tmp.push_back(idxs[i]);
            }
            idxs.clear();        

            for (int ii = 0; ii < tmp.size(); ++ii)
            {
                bool ignoreit=false;
                for (int jj = 0; jj < suppress.size(); ++jj)
                {
                    if(ii==suppress[jj]){
                        ignoreit=true;
                        break;
                    }                
                }

                if(!ignoreit){
                    idxs.push_back(tmp[ii]);
                }
            }

            /*int iii;
            scanf ("%d",&iii);*/

        }

        vector< Rect > picked_bx;
        loc = boxes.begin();
        end = boxes.end();
        int a=0;
        for( ; loc != end ; ++loc )
        {
            for (int ii = 0; ii < pick.size(); ++ii)
            {
                if(a==pick[ii]){

                    Rect rec;
                    rec.x=loc->x;
                    rec.y=loc->y;
                    rec.width=loc->width;
                    rec.height=loc->height;
                    picked_bx.push_back(rec);
                    //printf("%d %d %d %d\n",rec.x,rec.y,rec.width,rec.height);
                }
            }
            a++;
        }
        return picked_bx;
    }
    return boxes;
}

Mat&  detect_movement(Mat  & img, vector<Rect> humans, float sensitivity){
    
    printf("hello world\n");

    div_t divresult;
    if( !humans.empty() )
    {

        vector< Rect >::const_iterator loc = humans.begin();
        vector< Rect >::const_iterator end = humans.end();
        int c=0;
        for( ; loc != end ; ++loc )
        {

            int mid_x=loc->x+div(loc->width,2).quot;
            int mid_y=loc->y+div(loc->height,2).quot;

            int nearest_idx=-1;
            float min=999.9;
            for (int i = 0; i < people.size(); ++i)
            {
                float dist=sqrt(pow(mid_x-people[i][0],2)+pow(mid_y-people[i][1],2));
                if(min>dist&&nearest_idx<sensitivity){
                    min=dist;
                    nearest_idx=i;
                }
            }

            if(nearest_idx!=-1){
                std::vector<int> point;
                point.push_back(mid_x);
                point.push_back(mid_y);

                people.push_back(point);  
            }else{
                //printf("brike same sto %d ta %d %d\n",c+1,mid_x,mid_y);
                std::vector<int> point;
                point.push_back(mid_x);
                point.push_back(mid_y);
                paths.push_back(point);

            }
            c++;

        }
    }

    for (int i = 0; i < paths.size(); ++i)
    {        
        for (int j = -1; j < 1; ++j)
        {
            for (int z = -1; z < 1; ++z)
            {
                if(paths[i][0]+j>img.rows || paths[i][0]+j<0 || paths[i][1]+z>img.cols || paths[i][1]+z<0 ){
                    continue;
                }else{
                    img.at<Vec3b>(Point(paths[i][0]+j, paths[i][1]+z)) = Vec3b(0,0,255);
                }
            }
        }
        
    }

    return img;
}

void test_it( const Size & size )
{
    printf("mesa1\n");
    char key = 27;
    Scalar reference( 0, 255, 0 );
    Scalar trained( 0, 0, 255 );
    Scalar nms_color( 255, 0, 0 );
    Mat img, draw;
    Ptr<SVM> svm;
    HOGDescriptor hog;
    HOGDescriptor my_hog;
    my_hog.winSize = size;
    vector< Rect > locations;

    // Load the trained SVM.
    svm = StatModel::load<SVM>( "my_people_detector.yml" );
    // Set the trained svm to my_hog
    vector< float > hog_detector;
    get_svm_detector( svm, hog_detector );
    my_hog.setSVMDetector( hog_detector );
    // Set the people detector.
    hog.setSVMDetector( hog.getDefaultPeopleDetector() );
    // Open the camera.
    //Mat imaze = imread("/home/jimcha/opencv/workspace/many6.jpg");
    Mat imaze = imread("/home/jimcha/opencv/workspace/AerialDat/neg2/dji2.png");
    VideoCapture video("earth3.mp4");
/*    imshow("nata ",imaze);
    waitKey(0);*/

    //video.open(0);
    if( !video.isOpened() )
    {
        cerr << "Unable to open the device 0" << endl;
        exit( -1 );
    }

    bool end_of_process = false;
    while( !end_of_process )
    {

        video >> img;
        if( img.empty() )
            break;
        Mat dst,tmp;
        int resize_scaler;
        if(false){//image{
            tmp=imaze.clone();
            resize_scaler=1;
        }else{
            tmp=img.clone();
            resize_scaler=2;
        }

        draw = img.clone();
        Size s;
        s.height=tmp.rows/resize_scaler;
        s.width=tmp.cols/resize_scaler;
        resize(tmp, dst, s, 0, 0, cv::INTER_LINEAR );
        draw = dst.clone();

        locations.clear();
/*        hog.detectMultiScale( imaze, locations );
        draw_locations( draw, locations, trained );*/
        locations.clear();
        my_hog.detectMultiScale( dst, locations );
        printf("TR people detected %d\n" ,(int)locations.size());
        
//for debugging..
/*
        vector<Rect> newone;
        Rect hell;


        hell.x=12;
        hell.y=84;
        hell.width=140-hell.x;
        hell.height=212-hell.y;
        newone.push_back(hell);

*/
        vector< Rect > nmsed=nms(locations,0.1);
        draw_locations(detect_movement(draw,nmsed,20),nmsed, nms_color );
        //draw_locations( draw, locations,  reference );        
        //draw_locations( draw, nms(locations,0.1), nms_color );
        //draw_locations( draw, nms(newone,0.3), nms_color );
        printf("# # #\n");
        imshow( "Video", draw );
        key = (char)waitKey( 10 );
        if( 27 == key )
            end_of_process = true;
    }
}

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
 
    load_images( pos_dir, pos, pos_lst );
    labels.assign( pos_lst.size(), +1 );
    const unsigned int old = (unsigned int)labels.size();
    load_images( neg_dir, neg, full_neg_lst );
    sample_neg( full_neg_lst, neg_lst, Size( 64,128 ) );
    labels.insert( labels.end(), neg_lst.size(), -1 );
    printf("%d %d\n",(int)pos_lst.size(),(int)neg_lst.size());
    CV_Assert( old < labels.size() );

    compute_hog( pos_lst, gradient_lst, Size( 64,128 ) );
    compute_hog( neg_lst, gradient_lst, Size( 64,128 ) );

    train_svm( gradient_lst, labels );

    test_it( Size( 64,128 ) ); // change with your parameters
    printf("ended\n");
    return 0;
}

