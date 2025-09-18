#include <unitree/robot/go2/video/video_client.hpp>

#include <iostream>
#include <fstream>
#include <ctime>


int main(int argc, char **argv){

    std::string networkInterface = "eno1";


    unitree::robot::ChannelFactory::Instance()->Init(0, networkInterface);
    unitree::robot::go2::VideoClient video_client;

    video_client.SetTimeout(0.1f);
    video_client.Init();


    std::vector<uint8_t> img_buf;
    int ret;



    ret = video_client.GetImageSample(img_buf);

    if (ret == 0) {
        time_t rawtime;
        struct tm *timeinfo;
        char buffer[80];

        time(&rawtime);
        timeinfo = localtime(&rawtime);

        strftime(buffer, sizeof(buffer), "%Y%m%d%H%M%S.jpg", timeinfo);
        std::string image_name = std::string("/workspace/images/") + buffer;

        std::ofstream image_file(image_name, std::ios::binary);
        if (image_file.is_open()) {
            image_file.write(reinterpret_cast<const char*>(img_buf.data()), img_buf.size());
            image_file.close();
            std::cout << "Image saved successfully as " << image_name << std::endl;
        } else {
            std::cerr << "Error: Failed to save image." << std::endl;
        }
    }
    else {
        std::cerr << "Error: Failed to get image sample, return code: " << ret << std::endl;
    }


    return 0;



}