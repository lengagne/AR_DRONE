#include <stdio.h>

#include <time.h>
#include <unistd.h>

#define NBP img_->width*img_->height

#define PSI_ZERO -30

extern "C" {
  
  #include <RT_ARDrone/RT_ARDrone.h>
}

#include "aruco.h"
#include "cvdrawingutils.h"

#include "MoGS_Joystick.h"

#define NBAVERAGE 10

using namespace cv;
using namespace aruco;


int ThePyrDownLevel;
MarkerDetector MDetector;
vector<Marker> TheMarkers;

CameraParameters TheCameraParameters;
void cvTackBarEvents(int pos,void*);
bool readCameraParameters(string TheIntrinsicFile,CameraParameters &CP,Size size);

pair<double,double> AvrgTime(0,0) ;//determines the average time required for detection
double ThresParam1,ThresParam2;
int iThresParam1,iThresParam2;
int waitTime=0;

ARDrone* bob ;
RGB24Image* img_src_;

float gaz_, roll_, pitch_, yaw_;
int flag_;
int camera_;

void seb_landing( )
{
	flag_ = 0;
	//landing
	sleep(2);
	gaz_ = -0.01;
	ARDrone_land(bob);
}

void seb_take_off( )
{
	flag_ = 0;	
	ARDrone_takeoff(bob);
	sleep(5);
	flag_ = 1;
	printf("take off ended\n");
}

float sign( float a)
{
	if (a>0)
		return 1;
	else
		return -1;
}

void saturation( float * in, double max)
{
	if (*in > max)
		(*in) = max;
	if (*in < -max)
		(*in) = -max;	
}


int main ( int argc, char** argv, char** envv ) {

	
	if (argc != 2 )
	{
		std::cerr<<" error you should specify the *.yml file"<<std::endl;
		return -1;
	}
	
// 	img_ = RGB24Image_new(640,360);
	img_src_ = RGB24Image_new(640,360);
	bob = ARDrone_new( "192.168.10.1" ) ;

	ARDrone_connect( bob ) ;	
	ARDrone_trim ( bob ) ;
	// reinit drone
	ARDrone_reset_defaults( bob ) ;	
	ARDrone_trim ( bob ) ;
	
	camera_ = 0;

	
	// use bottom camera
	if (camera_ != 2)
	{
		ARDrone_zap_camera ( bob, 2 );
		camera_ = 2;
		sleep(2);
		int i;
		for (i=0;i<10;i++)	
			ARDrone_get_RGB24Image ( bob, img_src_ ) ;
	}
	
// 	pthread_t thread_affich;
// 	int idaffich = pthread_create( &thread_affich , NULL, affich_modif, NULL );	


	
	time_t start, end;
	start = time(NULL);
	int nbp = 5000;
	
	/** init aruco*/
	string TheInputVideo = "live";
	float TheMarkerSize = 64;
	string TheIntrinsicFile=argv[1];
	
	//set the dimensions of the image
	Mat TheInputImage = Mat::zeros(360,640,CV_8UC3 );
	Mat TheInputImageCopy = Mat::zeros(360,640, CV_8UC3);

        //read camera parameters if passed
        if (TheIntrinsicFile!="") {
            TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
            TheCameraParameters.resize(TheInputImage.size());
        }
        //Configure other parameters
        if (ThePyrDownLevel>0)
            MDetector.pyrDown(ThePyrDownLevel);


        //Create gui

        cv::namedWindow("thres",1);
        cv::namedWindow("in",1);
        MDetector.getThresholdParams( ThresParam1,ThresParam2);
        MDetector.setCornerRefinementMethod(MarkerDetector::LINES);
//         iThresParam1=ThresParam1;
//         iThresParam2=ThresParam2;
//         cv::createTrackbar("ThresParam1", "in",&iThresParam1, 13, cvTackBarEvents);
//         cv::createTrackbar("ThresParam2", "in",&iThresParam2, 13, cvTackBarEvents);
        iThresParam1=7;
        iThresParam2=7;
	
	int count = 0;
        char key=0;
        int index=0;
	
	
	bool inX, old_inX;
	bool inZ, old_inZ;
	float X,Y,Z,theta, oldX, oldZ, oldtheta;
	float dX,dZ, dtheta;
	int cptX = 0;
        
	    X = 0;
	    oldX = 0;
	    Z = 0;
	    oldZ = 0;	
	theta = 0;
	    oldtheta = 0;	
	    
	    
	
	//capture until press ESC or until the end of the video
        while ( key!=27)
        {
		// get the image
		ARDrone_get_RGB24Image ( bob, img_src_ ) ;
		
		// copy the image
		int cpt = 0;
		for (int j=0;j<360;j++)
			for (int i=0;i<640;i++)
			{				
				Point3_<uchar>* p = TheInputImage.ptr<Point3_<uchar> >(j,i);
				p->z = (uchar) img_src_->pixels[cpt++];	//R 
				p->y = (uchar) img_src_->pixels[cpt++];	//G
				p->x = (uchar) img_src_->pixels[cpt++];	//B 
			}
			
			
			
			
            index++; //number of images captured
            double tick = (double)getTickCount();//for checking the speed
            //Detection of markers in the image passed
            MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters,TheMarkerSize);
            //chekc the speed by calculating the mean speed of all iterations
            AvrgTime.first+=((double)getTickCount()-tick)/getTickFrequency();
            AvrgTime.second++;

            //print marker info and draw the markers in image
            TheInputImage.copyTo(TheInputImageCopy);
	    
    
            for (unsigned int i=0;i<TheMarkers.size();i++) {
//                 cout<<TheMarkers[i]<<endl;
                TheMarkers[i].draw(TheInputImageCopy,Scalar(0,0,255),1);
		
            }

            //draw a 3d cube in each marker if there is 3d info
            if (  TheCameraParameters.isValid())
	    {
                for (unsigned int i=0;i<TheMarkers.size();i++) {
                    CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
                    CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
                }
	    }
	    
	    pitch_ = 0;
	    roll_ = 0;
	    gaz_ = 0;
	    yaw_ = 0;
	    

	    
	    inX = false;
	    old_inX = false;
	    inZ = false;
	    old_inZ = false;
	    
	    
			if (TheMarkers.size()> 0)
			{
				X =  TheMarkers[0].Tvec.ptr<float>(0)[0] ;
				Y =  TheMarkers[0].Tvec.ptr<float>(0)[1] ;
				Z =  TheMarkers[0].Tvec.ptr<float>(0)[2] ;
				
				
				cv::Mat_<float> rotMat(3, 3), tmp(3,3);
				
				tmp(0,0) = 0;	tmp(0,1) = -1;	tmp(0,2) = 0;
				tmp(1,0) = 0;	tmp(1,1) = 0;	tmp(1,2) = -1;
				tmp(2,0) = 1;	tmp(2,1) = 0;	tmp(2,2) = 0;
				cv::Rodrigues(TheMarkers[0].Rvec, rotMat);
				
				rotMat = tmp * rotMat ;
				float theta  = atan2(rotMat(2,1), rotMat(1,1)) ;
						
				if (count == 0)
				{
					dX = dZ = dtheta = 0.0;
				}else
				{
					dX = X - oldX;
					dZ = Z - oldZ;
					dtheta = theta - oldtheta;
				}
				float pitch_ref = 600;				
				
// 				// le yaw sert à corriger l'angle  (tester pour corriger X et pitch pour l'angle)
// 				yaw_ =   ( 0 - (theta /*+ 20* dtheta*/  ) )* 1.0;
// 				float X_ref =  pitch_ref*sin(theta);			
// 				roll_ = - ( X_ref - (X+ 20 * dX)) * 0.002; 
// 				pitch_ = - ( (Z + 20*dZ) - cos(theta) * pitch_ref) * 0.002;
// 				gaz_ = ( 0- Y ) * 0.001;  // seems good


				// le yaw sert à corriger l'angle  (tester pour corriger X et pitch pour l'angle)
				yaw_ =   (  X -0 )* 0.005;
// 				float X_ref =  pitch_ref*sin(theta);			
// 				roll_ = - ( X_ref - (X+ 20 * dX)) * 0.002; 
				roll_ = - ( 0 - (theta + 15* dtheta)) * 0.3; 
				pitch_ = - ( (Z + 15*dZ) - pitch_ref) * 0.001;
				gaz_ = ( 0- Y ) * 0.002;  // seems good

				
				oldX = X;
				oldZ = Z;
				oldtheta = theta;
			}else
			{
// 				std::cout<<" no cube "<<std::endl;
			}

            //show input with augmented information and  the thresholded image
            cv::imshow("in",TheInputImageCopy);
	    cv::imshow("thres",MDetector.getThresholdedImage());
//             cv::imshow("thres",MDetector.getThresholdedImage());

	    
		if (count == 10)
		{
			printf("Wait to take off\n");
// 			getchar();
			// take off
			seb_take_off();	
		}
			
		saturation(&roll_, 1.0);
		saturation(&pitch_, 1.0);
		saturation(&yaw_, 1.0);
		saturation(&gaz_, 1.0);		
	    
		
		
// 		if ( pitch_ !=0 || roll_ != 0 || count > 100)
		if (count > 100)
// 			if (count%20 > 10)
				ARDrone_move(bob,1,roll_,pitch_,yaw_,gaz_);
// 			else
// 				ARDrone_move(bob,1,roll_,0,yaw_,gaz_);
// 			ARDrone_move(bob,1,roll_,pitch_,yaw_,gaz_);
// 			ARDrone_move(bob,3,roll_,pitch_,yaw_,gaz_);
// 			ARDrone_move(bob,5,roll_,pitch_,yaw_,gaz_);
		else
			ARDrone_move(bob,0,0,0,0,0);
			


			
// 		std::cout<<" count = "<< count <<std::endl;
	    count ++;	    
	    
            key=cv::waitKey(30);//wait for key to be pressed

        }	

	seb_landing();
// 	// on se pose et c'est tout pour l'instant
	ARDrone_free( bob ) ;

	return 0 ;
}
