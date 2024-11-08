#include "image_subscriber.hpp"
#include <iostream>
#include <vector>
#include <stdio.h>


ImageSubscriber::ImageSubscriber() : Node("image_subscriber") {
	int status = XFilter_image_Initialize(&this->ip_inst, "filter_image");
	if (status != XST_SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "Error: Could not initialize the IP core.");
		return;
	}
	else{
		std::cout << "Imge processor initiated\n";
	}

	RCLCPP_INFO(this->get_logger(), "Initializing ImageSubscriber node");

	RCLCPP_INFO(this->get_logger(), "Starting camera subscription");

	camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
				"/image_raw",
				10,
				std::bind(&ImageSubscriber::onImageMsg, this, std::placeholders::_1));

	publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_image", 10);
	publisher2_ = this->create_publisher<sensor_msgs::msg::Image>("/sized_grey_image", 10);
}

ImageSubscriber::~ImageSubscriber()
{
	RCLCPP_INFO(this->get_logger(), "Subscriber out");
}

void ImageSubscriber::filterImage(const uint8_t *rowAbove, const uint8_t *rowCenter, const uint8_t* rowBelow,  uint8_t* outputRow, int width)
{
	const float kernel[3][3] = {
        {1 / 16.0f, 2 / 16.0f, 1 / 16.0f},
        {2 / 16.0f, 4 / 16.0f, 2 / 16.0f},
        {1 / 16.0f, 2 / 16.0f, 1 / 16.0f}
    	};

    	for (int x = 1; x < width - 1; ++x) {
        	float pixelValue = 0.0f;

        	pixelValue += rowAbove[x - 1] * kernel[0][0];
        	pixelValue += rowAbove[x]     * kernel[0][1];
        	pixelValue += rowAbove[x + 1] * kernel[0][2];

        	pixelValue += rowCenter[x - 1] * kernel[1][0];
        	pixelValue += rowCenter[x]     * kernel[1][1];
        	pixelValue += rowCenter[x + 1] * kernel[1][2];

        	pixelValue += rowBelow[x - 1] * kernel[2][0];
        	pixelValue += rowBelow[x]     * kernel[2][1];
        	pixelValue += rowBelow[x + 1] * kernel[2][2];

        	outputRow[x] = static_cast<uint8_t>(std::min(std::max(static_cast<int>(pixelValue), 0), 255));
   	 }

   	outputRow[0] = 0;
    	outputRow[width - 1] = 0;
}

cv::Mat ImageSubscriber::applyConvolutionToImage(cv::Mat& inputImage) {

 	cv::Mat outputImage = cv::Mat::zeros(120, 120, CV_8UC1);
	uint8_t rowAbove[120];
	uint8_t  rowCenter[120];
	uint8_t  rowBelow[120];
	uint8_t  outRow[120];
	
	for(int i = 0; i < 120; i++){
		for(int j = 0; j < 120; j++){
			rowAbove[j] = inputImage.at<uint8_t>(i,j);
			rowCenter[j] = inputImage.at<uint8_t>(i+1,j);
			rowBelow[j] = inputImage.at<uint8_t>(i+2,j);
		}
		std::cout << std::endl;
		XFilter_image_Write_rowAbove_Words(&ip_inst, 0, (unsigned int*)rowAbove, 120/ sizeof(unsigned int));
		XFilter_image_Write_rowCenter_Words(&ip_inst, 0,  (unsigned int*)rowCenter, 120/ sizeof(unsigned int));
		XFilter_image_Write_rowBelow_Words(&ip_inst, 0,  (unsigned int*)rowBelow, 120/ sizeof(unsigned int));

		XFilter_image_Start(&ip_inst);

	    	while (!XFilter_image_IsDone(&ip_inst));

		XFilter_image_Read_outputRow_Words(&ip_inst, 0,  (unsigned int*)outRow, 120/ sizeof(unsigned int));

		for(int x = 0; x < 120; x++){
			outputImage.at<uint8_t>(i,x) = outRow[x];
		}
	}

    return outputImage;
}


void ImageSubscriber::onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg) {
	RCLCPP_INFO(this->get_logger(), "Received image!");

	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
	cv::Mat img = cv_ptr->image;
	
	cv::Mat grey_scale;
	cv::cvtColor(img, grey_scale, cv::COLOR_RGB2GRAY);
	cv::Mat resized;
	resize(grey_scale, resized, cv::Size(120,120), cv::INTER_LINEAR);
	int rows = img.rows;
	int cols = img.cols;

	std::cout << ">> APPLYING CONVOLUTION" << std::endl;
	cv::Mat outImg  = applyConvolutionToImage(resized);

 	std::cout << "Image rows = " << rows << ",  Image cols = " << cols  << std::endl;
	RCLCPP_INFO(this->get_logger(), "Successfully loaded image!");

	RCLCPP_INFO(this->get_logger(), "Publishing to topic!");
	sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", outImg).toImageMsg();
	publisher_->publish(*img_msg.get());

	sensor_msgs::msg::Image::SharedPtr img_msg2 = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", resized).toImageMsg();
        publisher2_->publish(*img_msg2.get());
}

int main(int argc, char *argv[])
{
	setvbuf(stdout,NULL,_IONBF,BUFSIZ);

	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<ImageSubscriber>());

	rclcpp::shutdown();
	return 0;
}
