#pragma once
#include "drivers/xfilter_image.h" 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>
#include <stdint.h>

class ImageSubscriber : public rclcpp::Node
{

public:
	ImageSubscriber();

	~ImageSubscriber();

private:
	void onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg);
	void filterImage(const uint8_t *rowAbove, const uint8_t *rowCenter, const uint8_t* rowBelow,  uint8_t* outputRow, int width);
	cv::Mat applyConvolutionToImage(cv::Mat& inputImage);


public:
	int test = 0;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
 	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher2_;
	XFilter_image ip_inst;
private:
};

