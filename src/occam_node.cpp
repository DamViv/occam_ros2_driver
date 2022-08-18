#include <stdio.h>
#include <chrono>
using namespace std::chrono_literals;
#include <iostream>
#include <string>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <opencv2/highgui.hpp>
#include "cv_bridge/cv_bridge.h"

#include "indigo.h"



static void reportError(int error_code) {
  RCLCPP_ERROR(rclcpp::get_logger("my logger :"), "Occam API Error: %i", error_code);
  abort();
}
#define OCCAM_CHECK(call) { int r = call; if (r != OCCAM_API_SUCCESS) reportError(r); }

static std::string dataNameString(OccamDataName data_name) {
    switch (data_name) {
        case OCCAM_IMAGE0: return "image0";
        case OCCAM_IMAGE1: return "image1";
        case OCCAM_IMAGE2: return "image2";
        case OCCAM_IMAGE3: return "image3";
        case OCCAM_IMAGE4: return "image4";
        case OCCAM_IMAGE5: return "image5";
        case OCCAM_IMAGE6: return "image6";
        case OCCAM_IMAGE7: return "image7";
        case OCCAM_IMAGE8: return "image8";
        case OCCAM_IMAGE9: return "image9";
        case OCCAM_IMAGE10: return "image10";
        case OCCAM_IMAGE11: return "image11";
        case OCCAM_IMAGE12: return "image12";
        case OCCAM_IMAGE13: return "image13";
        case OCCAM_IMAGE14: return "image14";
        case OCCAM_RAW_IMAGE0: return "raw_image0";
        case OCCAM_RAW_IMAGE1: return "raw_image1";
        case OCCAM_RAW_IMAGE2: return "raw_image2";
        case OCCAM_RAW_IMAGE3: return "raw_image3";
        case OCCAM_RAW_IMAGE4: return "raw_image4";
        case OCCAM_RAW_IMAGE5: return "raw_image5";
        case OCCAM_RAW_IMAGE6: return "raw_image6";
        case OCCAM_RAW_IMAGE7: return "raw_image7";
        case OCCAM_RAW_IMAGE8: return "raw_image8";
        case OCCAM_RAW_IMAGE9: return "raw_image9";
        case OCCAM_RAW_IMAGE10: return "raw_image10";
        case OCCAM_RAW_IMAGE11: return "raw_image11";
        case OCCAM_RAW_IMAGE12: return "raw_image12";
        case OCCAM_RAW_IMAGE13: return "raw_image13";
        case OCCAM_RAW_IMAGE14: return "raw_image14";
        case OCCAM_IMAGE_TILES0: return "image_tiles0";
        case OCCAM_IMAGE_TILES1: return "image_tiles1";
        case OCCAM_IMAGE_TILES2: return "image_tiles2";
        case OCCAM_RAW_IMAGE_TILES0: return "raw_image_tiles0";
        case OCCAM_RAW_IMAGE_TILES1: return "raw_image_tiles1";
        case OCCAM_RAW_IMAGE_TILES2: return "raw_image_tiles2";
        case OCCAM_UNDISTORTED_IMAGE_TILES0: return "undistorted_image_tiles0";
        case OCCAM_UNDISTORTED_IMAGE_TILES1: return "undistorted_image_tiles1";
        case OCCAM_UNDISTORTED_IMAGE_TILES2: return "undistorted_image_tiles2";
        case OCCAM_UNDISTORTED_IMAGE0: return "undistorted_image0";
        case OCCAM_UNDISTORTED_IMAGE1: return "undistorted_image1";
        case OCCAM_UNDISTORTED_IMAGE2: return "undistorted_image2";
        case OCCAM_UNDISTORTED_IMAGE3: return "undistorted_image3";
        case OCCAM_UNDISTORTED_IMAGE4: return "undistorted_image4";
        case OCCAM_UNDISTORTED_IMAGE5: return "undistorted_image5";
        case OCCAM_UNDISTORTED_IMAGE6: return "undistorted_image6";
        case OCCAM_UNDISTORTED_IMAGE7: return "undistorted_image7";
        case OCCAM_UNDISTORTED_IMAGE8: return "undistorted_image8";
        case OCCAM_UNDISTORTED_IMAGE9: return "undistorted_image9";
        case OCCAM_UNDISTORTED_IMAGE10: return "undistorted_image10";
        case OCCAM_UNDISTORTED_IMAGE11: return "undistorted_image11";
        case OCCAM_UNDISTORTED_IMAGE12: return "undistorted_image12";
        case OCCAM_UNDISTORTED_IMAGE13: return "undistorted_image13";
        case OCCAM_UNDISTORTED_IMAGE14: return "undistorted_image14";
        case OCCAM_STITCHED_IMAGE0: return "stitched_image0";
        case OCCAM_STITCHED_IMAGE1: return "stitched_image1";
        case OCCAM_STITCHED_IMAGE2: return "stitched_image2";
        case OCCAM_RECTIFIED_IMAGE0: return "rectified_image0";
        case OCCAM_RECTIFIED_IMAGE1: return "rectified_image1";
        case OCCAM_RECTIFIED_IMAGE2: return "rectified_image2";
        case OCCAM_RECTIFIED_IMAGE3: return "rectified_image3";
        case OCCAM_RECTIFIED_IMAGE4: return "rectified_image4";
        case OCCAM_RECTIFIED_IMAGE5: return "rectified_image5";
        case OCCAM_RECTIFIED_IMAGE6: return "rectified_image6";
        case OCCAM_RECTIFIED_IMAGE7: return "rectified_image7";
        case OCCAM_RECTIFIED_IMAGE8: return "rectified_image8";
        case OCCAM_RECTIFIED_IMAGE9: return "rectified_image9";
        case OCCAM_DISPARITY_IMAGE0: return "disparity_image0";
        case OCCAM_DISPARITY_IMAGE1: return "disparity_image1";
        case OCCAM_DISPARITY_IMAGE2: return "disparity_image2";
        case OCCAM_DISPARITY_IMAGE3: return "disparity_image3";
        case OCCAM_DISPARITY_IMAGE4: return "disparity_image4";
        case OCCAM_TILED_DISPARITY_IMAGE: return "tiled_disparity_image";
        case OCCAM_STITCHED_DISPARITY_IMAGE: return "stitched_disparity_image";
        case OCCAM_POINT_CLOUD0: return "point_cloud0";
        case OCCAM_POINT_CLOUD1: return "point_cloud1";
        case OCCAM_POINT_CLOUD2: return "point_cloud2";
        case OCCAM_POINT_CLOUD3: return "point_cloud3";
        case OCCAM_POINT_CLOUD4: return "point_cloud4";
        case OCCAM_STITCHED_POINT_CLOUD: return "stitched_point_cloud";
    }
    return std::string();
}



class OccamNode : public rclcpp::Node
{
public:
    OccamNode() : Node("OccamNode"), device_(0)
    {       
	// Check the connected devices	
        OccamDeviceList* device_list;        
        int dev_index = check_connected_devices(device_list);

        RCLCPP_INFO(this->get_logger(), "OCCAM_CHECK: %s", device_list->entries[dev_index].cid);
        if(dev_index < 0)
            return;     
        
        // Open and Check if selected device is color
        OCCAM_CHECK(occamOpenDevice(device_list->entries[dev_index].cid, &device_));

	    // clear list of devices
    	OCCAM_CHECK(occamFreeDeviceList(device_list));

    	// Create publishers
    	create_publishers(device_);

    	// Main loop
        RCLCPP_INFO(this->get_logger(), "START PUBLISHING" );
	    timer_ = this->create_wall_timer(10ms, std::bind(&OccamNode::take_and_send_data, this));
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    OccamDevice* device_;
    int is_color_ = 0;

    std::vector< rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr > image_pubs_;
    std::vector< rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr > camera_info_pubs_;
    std::vector<OccamDataName> image_pubs_names_;
private:

    int check_connected_devices(OccamDeviceList* &device_list){

        OCCAM_CHECK(occamEnumerateDeviceList(2000, &device_list));
        if (device_list->entry_count > 0)
      	    RCLCPP_INFO(this->get_logger(),"%i devices found", device_list->entry_count);
	    else
	        RCLCPP_ERROR(this->get_logger(), "No compatible devices detected");

        int dev_index = 0;
        std::string cid;
        for (int i=0;i<device_list->entry_count;++i) {
            if (!cid.empty() && device_list->entries[i].cid == cid)
                dev_index = i;
            RCLCPP_INFO(this->get_logger(),"device[%i]: cid = %s",i,device_list->entries[i].cid);
        }

        if (dev_index< 0 || dev_index >= device_list->entry_count)
            return -1;
        if (!cid.empty() && device_list->entries[dev_index].cid != cid) {
            RCLCPP_ERROR(this->get_logger(),"Requested cid %s not found",cid.c_str());
            return -1;
        }
        return dev_index;
    }
    
    
    void create_publishers(OccamDevice* device_) {
    	OCCAM_CHECK(occamGetDeviceValuei(device_, OCCAM_COLOR, &is_color_));

        // CREATE IMAGE PUBLISHER AND PUSH BACK IN A VECTOR OF IMAGE PUBLISHER
        int req_count;
    	OccamDataName* req;
	    OccamDataType* types;
        OCCAM_CHECK(occamDeviceAvailableData(device_, &req_count, &req, &types));
    	for (int j=0;j<req_count;++j) {
            if (types[j] == OCCAM_IMAGE){
                rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
                std::string req_name = dataNameString(req[j]);
                RCLCPP_INFO(this->get_logger(), "advertising %s", ("occam/" + req_name).c_str());
                publisher = this->create_publisher<sensor_msgs::msg::Image>("occam/" + req_name, 10);
                image_pubs_.push_back(publisher);
                image_pubs_names_.push_back(req[j]);
            }
	       else if (types[j] == OCCAM_POINT_CLOUD) {
                /// TODO for stereo case with 3D point cloud
            }
	    }

	    // CREATE CAMERA INFO PUBLISHER AND PUSH BACK IN A VECTOR OF CAMERA INFO PUBLISHER
        int sensor_count;
        int sensor_width;
        int sensor_height;
        OCCAM_CHECK(occamGetDeviceValuei(device_, OCCAM_SENSOR_COUNT, &sensor_count));
        OCCAM_CHECK(occamGetDeviceValuei(device_, OCCAM_SENSOR_WIDTH, &sensor_width));
        OCCAM_CHECK(occamGetDeviceValuei(device_, OCCAM_SENSOR_HEIGHT, &sensor_height));

        while (camera_info_pubs_.size() < sensor_count) {
            std::stringstream sout;
            sout << "occam/camera_info" << camera_info_pubs_.size();
            RCLCPP_INFO(this->get_logger(), "advertising %s", sout.str().c_str());
            rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher;
            publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>(sout.str(), 1);
            camera_info_pubs_.push_back(publisher);
        }
        occamFree(req);
        occamFree(types);
    }


    cv::Mat convert2cvMat(void* data){
        OccamImage* image = (OccamImage*)data;
        cv::Mat img;
        if (image && image->format == OCCAM_GRAY8) {
            img = cv::Mat_<uchar>(image->height,image->width,(uchar*)image->data[0],image->step[0]);
        }
        else if (image && image->format == OCCAM_RGB24) {
            img = cv::Mat_<cv::Vec3b>(image->height,image->width,(cv::Vec3b*)image->data[0],image->step[0]);
            cv::Mat img1;
            cv::cvtColor(img, img1, cv::COLOR_BGR2RGB);
            img = img1;
        }
        else if (image && image->format == OCCAM_SHORT1) {
            img = cv::Mat_<short>(image->height,image->width,(short*)image->data[0],image->step[0]);
        }
        else{
            printf("image format not supported yet...\n");
        }
        return img.clone();
    }



    void convert(void* data, sensor_msgs::msg::Image &img1) {
        OccamImage* img0 = (OccamImage*)data;
        if (!img0)
            return;
        const char* image_encoding = 0;
        int bpp = 1;
        switch (img0->format) {
            case OCCAM_GRAY8:
                bpp = 1;
                image_encoding = is_color_ ? "bgr8" : "mono8";
                break;
            case OCCAM_RGB24:
                bpp = 3;
                image_encoding = "bgr8";
                break;
            case OCCAM_SHORT1:
                bpp = 2;
                image_encoding = "mono16";
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Image format not valid (%i)",img0->format);
                return;
                break;
        }

        sensor_msgs::fillImage(img1, image_encoding, img0->height, img0->width, img0->step[0], img0->data[0]);


    }


    bool take_and_send_data(){

        uint nb_img = image_pubs_.size();
        std::vector<OccamDataType> types(nb_img);
        OccamDataName* req = (OccamDataName*)occamAlloc(nb_img*sizeof(OccamDataName));
        for (int i=0;i<nb_img;++i)
            req[i] = image_pubs_names_.at(i);
        OccamImage** images = (OccamImage**)occamAlloc(nb_img*sizeof(OccamImage*));

        int r;
        if ((r = occamDeviceReadData(device_, nb_img, req,  &types[0], (void**)images, 1)) != OCCAM_API_SUCCESS){
            char error_str[256] = {0};
            occamGetErrorString((OccamError)r, error_str, sizeof(error_str));
            RCLCPP_ERROR(this->get_logger(), "Driver returned error %s (%i)",error_str,r);
            return false;
        }


        for (int j=0;j<nb_img;++j) {
//            printf("read image %i: time_ns = %llu, index = %i, format = %i, width = %i, height = %i\n",
//                   j, (long long unsigned int)images[j]->time_ns, images[j]->index,
//                   images[j]->format, images[j]->width, images[j]->height);

            // keep only rgb images
            /// TODO : publish raw images too
            if(images[j]->format !=2)
                continue;
            
            cv::Mat I = convert2cvMat(images[j]);
            if(I.cols == 0)
                return false;

            // Create message from the cv::Mat image
            sensor_msgs::msg::Image::SharedPtr msg;
            switch (images[j]->format) {
//                case OCCAM_GRAY8:
//                    if(is_color_) {
//                        msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", I).toImageMsg();
//                    }
//                    else{
//                        msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", I).toImageMsg();
//                    }
//                    break;
                case OCCAM_RGB24:
                    msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", I).toImageMsg();
                    break;
//                case OCCAM_SHORT1:
//                    msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", I).toImageMsg();
//                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Image format not valid (%i)",images[j]->format);
                    return false;
                    break ;
            }
            image_pubs_[j]->publish(*msg.get());
        }
        occamFree(images);
        occamFree(req);
    	return true;
    }
    
};



int main(int argc, char **argv)
{	
    OCCAM_CHECK(occamInitialize());

    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccamNode>();
    cv::namedWindow("view", cv::WINDOW_AUTOSIZE);
    cv::startWindowThread();

    rclcpp::spin(node);
    rclcpp::shutdown();
}
