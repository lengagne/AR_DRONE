#include <stdio.h>

#include <time.h>
#include <unistd.h>

#define rougeB 77
#define rougeG 52
#define rougeR 226

// #define rougeY 160
// #define rougeU 180

#define bleuY 206
#define bleuU 106
#define bleuV 143

#define SEUIL 15

#define NBP img_->width*img_->height

#define PSI_ZERO -30

extern "C" {
  
  #include <RT_ARDrone/RT_ARDrone.h>
}

#include "aruco.h"
#include "cvdrawingutils.h"

#include "MoGS_Joystick.h"


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
// 	ARDrone_takeoff(bob);
	sleep(5);
	flag_ = 1;
	printf("take off ended\n");
}


int main ( int argc, char** argv, char** envv ) {

// 	img_ = RGB24Image_new(640,360);
	img_src_ = RGB24Image_new(640,360);
	bob = ARDrone_new( "192.168.10.1" ) ;

	ARDrone_connect( bob ) ;	
	ARDrone_trim ( bob ) ;
	// reinit drone
	ARDrone_reset_defaults( bob ) ;	
	ARDrone_trim ( bob ) ;
	
	sleep(5);

	MoGS_Joystick pad;
	
	camera_ = 0;

	int i,j;
	
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

//         cv::namedWindow("thres",1);
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
        
	//capture until press ESC or until the end of the video
        while ( key!=27 && !pad.get_stop() )
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
                if (TheMarkers.size()> 0)
		{
//                  std::cout<<" position = "<< TheMarkers[0].Tvec<<std::endl;
// 			std::cout<<" X = "<< TheMarkers[0].Tvec.ptr<float>(0)[0] <<std::endl;
// 			std::cout<<" Y = "<< TheMarkers[0].Tvec.ptr<float>(0)[1] <<std::endl;
// 			std::cout<<" Z = "<< TheMarkers[0].Tvec.ptr<float>(0)[2] <<std::endl;
			float X =  TheMarkers[0].Tvec.ptr<float>(0)[0] ;
		    float Z =  TheMarkers[0].Tvec.ptr<float>(0)[2] ;
		}
		    
		    pitch_ = - pad.get_forward_velocity() * 0.2;
		    roll_ =  pad.get_side_velocity()* 0.2;
		    gaz_ = pad.get_up_velocity();		// hauteur
		    yaw_ = pad.get_rotate_velocity();

            //show input with augmented information and  the thresholded image
            cv::imshow("in",TheInputImageCopy);
//             cv::imshow("thres",MDetector.getThresholdedImage());

	    
		if (count == 10)
		{
			printf("Wait to take off\n");
			getchar();
			// take off
			seb_take_off();	
		}
		
	    
		
		
// 		if ( pitch_ !=0 || roll_ != 0 || count > 100)
		if (count > 100)
// 			ARDrone_move(bob,0,roll_,pitch_,yaw_,gaz_);
// 			ARDrone_move(bob,1,roll_,pitch_,yaw_,gaz_);
			ARDrone_move(bob,3,roll_,pitch_,yaw_,gaz_);
// 			ARDrone_move(bob,5,roll_,pitch_,yaw_,gaz_);

// 		else
// 			ARDrone_move(bob,0,0,0,0,0);
	    
	    count ++;	    
	    
            key=cv::waitKey(30);//wait for key to be pressed

        }	

	seb_landing();
// 	// on se pose et c'est tout pour l'instant
	ARDrone_free( bob ) ;

	return 0 ;
}
