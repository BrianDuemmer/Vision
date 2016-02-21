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


using namespace cv;
using namespace std;


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


int main(int argc, char **argv) {
	bool die(false);
	string filename("snapshot");
	string suffix(".png");
	int i_snap(0),iter(0);
	
	Mat depthMat(Size(640,480),CV_16UC1);
	Mat depthf (Size(640,480),CV_8UC1);
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
	      int intensity = IRMat.at<uchar>(Point(x, y));
	      if(intensity > 250) {
		if(intensity > maxIntensity) {
		  maxIntensity = intensity;
		  cout << "max intensity: " << maxIntensity << endl;
		}
		if(!valuesSet && (IRMat.at<uchar>((y + 5), (x + 5)) > 250)) {
		  max_y = y;
		  min_y = y;
		  max_x = x;
		  min_x = x;
		  valuesSet = true;
		}
		if(x < min_x && (IRMat.at<uchar>(y, (x + 5)) > 250))
		  min_x = x;
		if(y < min_y && (IRMat.at<uchar>((y + 5), x) > 250))
		  min_y = y;
		if(x > max_x && (IRMat.at<uchar>(y, (x - 5)) > 250))
		  max_x = x;
		if(y > max_y && (IRMat.at<uchar>((y - 5), x) > 250))
		  max_y = y;
	      }
	    }
	  }
	  
	  cout << endl << endl << endl;
	  cout << "max_x = " << max_x << endl;
	  cout << "min_x = " << min_x << endl;
	  cout << "max_y = " << max_y << endl;
	  cout << "min_y = " << min_y << endl;
	  cout << endl << endl << endl;
	  usleep(100000);
	  /***********************************************/
	  /*************Draw Rectangle *******************/
	  int padding = 0;
	  cv::Point leftTop((min_x - padding), (min_y + padding));
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
	  
	  int depthCenter = depthf.at<uchar>(Point((IRMat.rows/2), (IRMat.cols/2))); //top left pixel depth
	  stringstream ss;
	  ss << depthCenter;
	  string str = ss.str();
	  cv::putText(IRMat, str, Point(10,60), CV_FONT_HERSHEY_SIMPLEX, 2, 255);
	  /***********************************************/
	  
	  cv::imshow("IR", IRMat);
	  depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
	  cv::imshow("depth",depthf);
	  char k = cvWaitKey(5);
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
