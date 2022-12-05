#include <iostream>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <vector>
#include <fstream>
#include <chrono>
using namespace cv;
using namespace std;



tuple<vector<double>,vector<double>,vector<double>> new_image_depth(cv::Mat& im,double y_c,double f){

    int channels = im.channels();

    int h = im.rows;
    int w = im.cols * channels;   

    vector<double> X;
    vector<double> Y;
    auto t_y = 0.;
    auto e_y = h*0.5*2.6e-4;
    auto e_x = w*2.6e-4;

    int i,j;
    uchar* p;
    for( i = 0; i < h; ++i)
    {
        p = im.ptr<uchar>(i);
        for ( j = 0; j < w; ++j)
        {
            if (p[j]==255){
                double y =  (y_c/(f*(i*2.6e-4-t_y)) )*(f*f+e_y*e_y+(i*2.6e-4-e_y)*(t_y-e_y));
                //cout <<i << " " << j<<endl;
                Y.push_back( y );
                X.push_back( (j*2.6e-4-e_x/2)*y/f);
               //cout << "1" <<endl; 
            }
        }
    }
    vector<double> Z(X.size(), 0.f);

    return tuple<vector<double>,vector<double>,vector<double>> (X,Y,Z);
}

int main(){
    // Read the image file as
    // imread("default.jpg");
    Mat image = imread("output.jpg",IMREAD_GRAYSCALE);
    /*
    // Error Handling
    if (image.empty()) {
        cout << "Image File "
             << "Not Found" << endl;
  
        // wait for any key press
        cin.get();
        return -1;
    }
  
    // Show Image inside a window with
    // the name provided
    imshow("Window Name", image);
  
    // Wait for any keystroke
    waitKey(0);
    */
    auto t1  = std::chrono::high_resolution_clock::now();
    // apply function
    tuple<vector<double>,vector<double>,vector<double>> XYZ =new_image_depth(image,1.0,78e-3);

    auto t2  = std::chrono::high_resolution_clock::now();
    auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);

    cout << ms_int.count()<< "ms";

    /*auto X =get<0>(XYZ);
    auto Y = get<1>(XYZ);
    auto Z = get<2>(XYZ);
    ofstream myfile;
    myfile.open ("example.txt");
    
    for (auto i =0; i<X.size();++i){
        //cout << X[i] <<endl;
        myfile << X[i] << " " << Y[i] << " " <<Z[i] << endl;
    }
    myfile.close();
    //cout << X.size() << Y.size() << endl;

    */
    return 0;
}