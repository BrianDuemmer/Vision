#include "libusb-1.0/libusb.h"
#include "libfreenect/libfreenect.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <unistd.h>
#include </usr/include/arpa/inet.h>
#include </usr/include/linux/socket.h>
#include <netdb.h>

using namespace cv;   //opencv
using namespace std;

/*************************MUTEX*****************************************/
/*****PREVENTS MEMORY FROM BEING WRITTEN TO ON MULTIPLE THREADS*********/
/***********************************************************************/
class myMutex {
	public:
		myMutex() {
			pthread_mutex_init( &m_mutex, NULL );
		}
		void lock() {
			pthread_mutex_lock( &m_mutex );
		}
		void unlock() {
			pthread_mutex_unlock( &m_mutex );
		}
	private:
		pthread_mutex_t m_mutex;
};
/***********************************************************************/

//*************KINECT INTERFACE*****************************************/
class MyFreenectDevice : public Freenect::FreenectDevice {
	public:
		MyFreenectDevice(freenect_context *_ctx, int _index)
	 		: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_11BIT),
			m_buffer_rgb(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_ir_frame(false),
			m_new_depth_frame(false), depthMat(Size(640,480),CV_16UC1),
			IRMat(Size(640,480), CV_8U),
			ownMat(Size(640,480),CV_8UC3,Scalar(0)) {
			
			for( unsigned int i = 0 ; i < 2048 ; i++) {
				float v = i/2048.0;
				v = std::pow(v, 3)* 6;
				m_gamma[i] = v*6*256;
			}
		}
		
		// Do not call directly even in child
		void VideoCallback(void* _rgb, uint32_t timestamp) {
		  //std::cout << "RGB callback" << std::endl;
			m_rgb_mutex.lock();
			uint8_t* rgb = static_cast<uint8_t*>(_rgb);
			IRMat.data = rgb;
			m_new_ir_frame = true;
			m_rgb_mutex.unlock();
		};
		
		// Do not call directly even in child
		void DepthCallback(void* _depth, uint32_t timestamp) {
		  //	std::cout << "Depth callback" << std::endl;
			m_depth_mutex.lock();
			uint16_t* depth = static_cast<uint16_t*>(_depth);
			depthMat.data = (uchar*) depth;
			m_new_depth_frame = true;
			m_depth_mutex.unlock();
		}
		
		bool getVideo(Mat& output) {
			m_rgb_mutex.lock();
			if(m_new_ir_frame) {
			  //cv::cvtColor(rgbMat, output, CV_RGB2BGR);
			  IRMat.copyTo(output);
				m_new_ir_frame = false;
				m_rgb_mutex.unlock();
				return true;
			} else {
				m_rgb_mutex.unlock();
				return false;
			}
		}
		
		bool getDepth(Mat& output) {
				m_depth_mutex.lock();
				if(m_new_depth_frame) {
					depthMat.copyTo(output);
					m_new_depth_frame = false;
					m_depth_mutex.unlock();
					return true;
				} else {
					m_depth_mutex.unlock();
					return false;
				}
			}
	private:
		std::vector<uint8_t> m_buffer_depth;
		std::vector<uint8_t> m_buffer_rgb;
		std::vector<uint16_t> m_gamma;
		Mat depthMat;
		Mat IRMat;
		Mat ownMat;
		myMutex m_rgb_mutex;
		myMutex m_depth_mutex;
		bool m_new_ir_frame;
		bool m_new_depth_frame;
};
/***************************************************************/

/***************************************************************/
/************************NETWORKING*****************************/
/***************************************************************/
class tcp_client
{
private:
    int sock;
    std::string address;
    int port;
    struct sockaddr_in server;
     
public:
    tcp_client();
    bool conn(string, int);
    bool send_data(string data);
    string receive(int);
};
 
tcp_client::tcp_client()
{
    sock = -1;
    port = 0;
    address = "";
}
 
/**
    Connect to a host on a certain port number
*/
bool tcp_client::conn(string address , int port)
{
    //create socket if it is not already created
    if(sock == -1)
    {
        //Create socket
        sock = socket(AF_INET , SOCK_STREAM , 0);
        if (sock == -1)
        {
            perror("Could not create socket");
        }
         
        cout<<"Socket created\n";
    }
    else    {   /* OK , nothing */  }
     
    //setup address structure
cout << "Setting up address structure\n";
    if(inet_addr(address.c_str()) == -1)
      {
cout << "setting up hostnet structure" << endl;
        struct hostent *he;
        struct in_addr **addr_list;
         
	//resolve the hostname, its not an ip address
	cout << "Resolving hostname" << endl;
	if ( (he = gethostbyname( address.c_str() ) ) == NULL)
        {
            //gethostbyname failed
            herror("gethostbyname");
            cout<<"Failed to resolve hostname\n";
             
            return false;
        }
         
        //Cast the h_addr_list to in_addr , since h_addr_list also has the ip address in long format only
        addr_list = (struct in_addr **) he->h_addr_list;
 
        for(int i = 0; addr_list[i] != NULL; i++)
        {
            //strcpy(ip , inet_ntoa(*addr_list[i]) );
            server.sin_addr = *addr_list[i];
             
            cout<<address<<" resolved to "<<inet_ntoa(*addr_list[i])<<endl;
             
            break;
        }
    }
     
    //plain ip address
    else
    {
        server.sin_addr.s_addr = inet_addr( address.c_str() );
    }
     
    server.sin_family = AF_INET;
    server.sin_port = htons( port );
     
    //Connect to remote server
    if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
    {
        perror("connect failed. Error");
        return 1;
    }
     
    cout<<"Connected\n";
    return true;
}
 
/**
    Send data to the connected host
*/
bool tcp_client::send_data(string data)
{
    //Send some data
    if( send(sock , data.c_str() , strlen( data.c_str() ) , 0) < 0)
    {
        perror("Send failed : ");
        return false;
    }
    cout<<"Data send\n";
     
    return true;
}
 
/**
    Receive data from the connected host
*/
string tcp_client::receive(int size=512)
{
    char buffer[size];
    string reply;
     
    //Receive a reply from the server
    if( recv(sock , buffer , sizeof(buffer) , 0) < 0)
    {
        puts("recv failed");
    }
     
    reply = buffer;
    return reply;
}
/***************************************************************/


/***************************************************************/
/***********************MAIN************************************/
/***************************************************************/
int main(int argc, char **argv) {
	bool die(false);
	string filename("snapshot");
	string suffix(".png");
	int i_snap(0),iter(0);
	
	/*********INITIALIZE TCP CLIENT*******************/
	tcp_client c;
	string host = "192.168.0.184";
	
	//connect to host
	c.conn(host , 15083);
	/*************************************************/
	
	Mat depthMat(Size(640,480),CV_16UC1);
	Mat depthf (Size(640,480),CV_8UC3);
	Mat IRMat(Size(640,480),CV_8U);
	Mat ownMat(Size(640,480),CV_8UC3,Scalar(0));
	
	// The next two lines must be changed as Freenect::Freenect
	// isn't a template but the method createDevice:
	// Freenect::Freenect<MyFreenectDevice> freenect;
	// MyFreenectDevice& device = freenect.createDevice(0);
	// by these two lines:
	
	freenect_video_format requested_format(FREENECT_VIDEO_IR_8BIT);
	Freenect::Freenect freenect;
	MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);
	device.setVideoFormat(requested_format);
	namedWindow("IR",CV_WINDOW_AUTOSIZE);
	namedWindow("depth",CV_WINDOW_AUTOSIZE);
	device.startVideo();
	device.startDepth();
	while (!die) {
	  device.getVideo(IRMat);
	  device.getDepth(depthMat);
	  
	  /*********Identify_Goal*************************/  
	  int maxIntensity = 0;
	  bool valuesSet = false;
	  int max_y, max_x, min_y, min_x;
	  for(int x = 0; x < IRMat.cols; x++) {                  
	    for(int y = 0; y < IRMat.rows; y++) {
	      /*
		check for intensity of pixel - reflective tape will generally represented by 
		a value of 255, the maximum value. Note that ambient lighting will also be 
		represented by this value. Florenscent lights haven't affected the image in tests. 

		Note: Looping through Mat images go from the top left to the bottom right.
		1 2 3 4 -- x 
		2 
		3
		4
		|
		Y
		
	      */
	      int intensity = IRMat.at<uchar>(Point(x, y)); 
	      if(intensity > 250) {                                                //If the pixel has a high intensity, remember the position
		if(intensity > maxIntensity) {
		  maxIntensity = intensity;
		  cout << "max intensity: " << maxIntensity << endl;
		}
		if(!valuesSet && (IRMat.at<uchar>((y + 5), (x + 5)) > 250)) {      //if no values have been set, set them
		  max_y = y;                                                       //(y+5) and (x+5) are checking to see if pixel is a random
		  min_y = y;                                                       //occurence or if there is high intensity pixel close by.
		  max_x = x;
		  min_x = x;
		  valuesSet = true;
		}
		int offsetTest = 5;
		if(x < min_x && (IRMat.at<uchar>(y, (x + offsetTest)) > 250))               //if there is a high intensity pixel lower than the lowest set
		  min_x = x;                                                                //minimum, then update the minimum x value
		if(y < min_y && (IRMat.at<uchar>((y + offsetTest), x) > 250))               //Same but for mininum y value
		  min_y = y;
		if(x > max_x && (IRMat.at<uchar>(y, (x - offsetTest)) > 250))               //Same but for maximum x value
		  max_x = x;
		if(y > max_y && (IRMat.at<uchar>((y - offsetTest), x) > 250))               //Same but for maximum y value
		  max_y = y;
	      }
	    }
	  }
	  
	  cout << endl << endl << endl;                                            //print out the extremas 
	  cout << "max_x = " << max_x << endl;
	  cout << "min_x = " << min_x << endl;
	  cout << "max_y = " << max_y << endl;
	  cout << "min_y = " << min_y << endl;
	  cout << endl << endl << endl;
	  usleep(100000);                                                          //sleep for 100 milliseconds 
	  /***********************************************/

	  /*************Draw Rectangle *******************/
	  int padding = 0;                                                         //Draw a rectangle with an x 
	  cv::Point leftTop((min_x - padding), (min_y + padding));                 //Points on graph (x, y)
	  cv::Point rightBottom((max_x + padding), (max_y - padding));
	  cv::Point rightTop((max_x + padding), (min_y - padding));
	  cv::Point leftBottom((min_x - padding), (max_y + padding));
	  cv::line(IRMat, leftTop, rightBottom, 255, 1, 0);
	  cv::line(IRMat, leftBottom, rightTop, 255, 1, 0);
	  cv::rectangle(IRMat, leftTop, rightBottom, 255, 1, 8);
	  /***********************************************/

	  /***********Caculate_average_depth**************/
	  int offset = 3;
	  int depthTL = depthf.at<uchar>(Point((min_x + offset), (min_y + offset))); //top left pixel depth
	  int depthTR = depthf.at<uchar>(Point((max_x - offset), (min_y + offset))); //top right pixel depth
	  int depthBL = depthf.at<uchar>(Point((min_x + offset), (max_y - offset))); //bottom left pixel depth
	  int depthBR = depthf.at<uchar>(Point((max_x - offset), (max_y - offset))); //bottom right pixel depth
	  int averageDepth = (depthTL + depthTR + depthBL + depthBR) / 4;
	  
	  int depthCenter = depthf.at<uchar>(Point((((min_x + max_x)/2)), (((min_y + max_y)/2)))); //top left pixel depth
	  stringstream ss;                                        
	  stringstream ss2;
	  ss << depthCenter;                
	  string str = ss.str();
	  str = "Center  Depth: " + str;
	  ss2 << averageDepth;
	  string str2 = ss2.str();
	  str2 = "Average Depth: " + str2;
	  cv::putText(IRMat, str, Point(10,30), CV_FONT_HERSHEY_PLAIN, 2, 255);
	  cv::putText(IRMat, str2, Point(10,60), CV_FONT_HERSHEY_PLAIN, 2, 255);
	  /***********************************************/
	  
	  
	  //DISPLAY IR IMAGE
	  cv::imshow("IR", IRMat);
	  ////
	  
	  //PROCESS DEPTH IMAGE
	  Scalar rgbImg;
	  depthMat.convertTo(depthf, CV_8UC3, 255.0/2048.0);
	  for(int x = 0; x < depthf.cols; x++) {
	    for(int y = 0; y < depthf.rows; y++) {
	      Vec3b color = image.at<Vec3b>(Point(x,y));
	      if(color[0] > 100 &&  color[0] < 200) {
		color[0] = 150;
		color[1] = 100;
		color[2] = 200;
		depthf.at<uchar>(y,x) = rgbImg;
	      }
	    }
	  }
       
	  cv::imshow("depth",depthf);
	  ////
	  
	  stringstream ssOut;
	  ssOut <<"&&" << min_x << "&&" << max_y << "&&" << max_x << "&&" << min_y << "&&";
	  string strOut = ssOut.str();
	  cout << "Data send: " << strOut << endl;
	  string netOut = strOut + "\r\n";
	  c.send_data(netOut);
	  
	  char k = cvWaitKey(5); //waitKey specifies how long the image should be displayed for
	  if( k == 27 ){
	    cvDestroyWindow("rgb");
	    cvDestroyWindow("depth");
	    break;
	  }
	  if( k == 8 ) {
	    std::ostringstream file;
	    file << filename << i_snap << suffix;
	    cv::imwrite(file.str(),IRMat);
	    i_snap++;
	  }
	  if(iter >= 100000) break;
	  iter++;
	}
	
	device.stopVideo();
	device.stopDepth();
	return 0;
}
/*********************************************************************/
