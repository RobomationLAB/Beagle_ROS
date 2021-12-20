#include "camera.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    
    AIcamera camera;    

    camera.AIcameraCMD = nh.subscribe("/aicameraCMD", 10, &AIcamera::CMD_Callback, &camera);
    camera.pub = it.advertise("camera/image", 1);
    camera.connect();
    

    ros::spin();

    return 0;
}