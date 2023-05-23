#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace cv;
const int RED = 0;
const int BLUE = 1;


class Energy : public rclcpp::Node
{
public:
    Energy() : Node("energy_test")
    {
        RCLCPP_INFO(this->get_logger(), "Start energy test !");
        auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
        auto model_path = pkg_path + "/model/fc.onnx";
        auto label_path = pkg_path + "/model/label.txt";
        net_ = cv::dnn::readNetFromONNX(model_path);
        cv::Mat image;
        image = cv::imread("/home/jlcv/yjq/sentry/src/roborts_test/datasets/2/001.jpg",IMREAD_GRAYSCALE);
        image = image / 255.0;
        std::cout<<image<<std::endl;
        std::cout<<image.size()<<"  "<<image.channels() <<std::endl;
        cv::Mat blob;

        cv::dnn::blobFromImage(image, blob, 1., cv::Size(20, 28));
        // Set the input blob for the neural network
        net_.setInput(blob);
        // Forward pass the image blob through the model
        cv::Mat outputs = net_.forward();
        std::cout<<outputs<<std::endl;

        // Do softmax
        float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
        cv::Mat softmax_prob;
        cv::exp((outputs - max_prob), softmax_prob);
        float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
        softmax_prob /= sum;
        std::cout<<softmax_prob<<std::endl;

        double confidence;
        cv::Point class_id_point;
        minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);

    }

private:
    cv::dnn::Net net_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Energy>());
    rclcpp::shutdown();
    return 0;
}
