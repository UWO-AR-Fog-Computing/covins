/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <definitions/Img.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include "ImuTypes.h"
#include <System.h>

// COVINS
#include <covins/covins_base/config_comm.hpp> //for covins_params

class ImageGrabber {
    public:
        ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

        void GrabImage(const definitions::Img& msg);

        ORB_SLAM3::System* mpSLAM;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "orbslam_3_frontend_node");
    ros::start();

    if(argc != 3)
    {
        std::cerr << std::endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << std::endl;        
        ros::shutdown();
        return 1;
    }    

    std::cout << "Hello" << std::endl;

    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,covins_params::orb::activate_visualization);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber subscriber = nodeHandler.subscribe("test", 100, &ImageGrabber::GrabImage, &igb);

    ros::spin();

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const definitions::Img& msg)
{

    float timestamp = msg.timestamp;

    // std::cout << "timestamp: " << timestamp << std::endl;

    // sensor_msgs::Image img = msg.image;

    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg.image,"");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}