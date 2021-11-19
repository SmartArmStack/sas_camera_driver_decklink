/**
Copyright (C) 2021 Murilo Marques Marinho

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not,
see <https://www.gnu.org/licenses/>.
*/
#include <sas_camera_driver_decklink/sas_camera_driver_decklink.h>

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}

/*********************************************
 * GLOBAL SCOPE FUNCTIONS (INCLUDING MAIN)
 * *******************************************/
sas::CameraDriverDecklink* create_instance_from_ros_parameter_server(ros::NodeHandle& nodehandle)
{
    ROS_INFO_STREAM(ros::this_node::getName() + "Trying to load configuration.");

    int decklink_index;
    if(!nodehandle.getParam(ros::this_node::getName()+"/decklink_index",decklink_index))
        throw std::runtime_error("Unable to get parameter 'decklink_index' from the parameter server.");

    return new sas::CameraDriverDecklink(&kill_this_process, nodehandle, decklink_index);
}

int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error(ros::this_node::getName() + "::Error setting the signal int handler.");
    }

    ros::init(argc, argv, "sas_decklink_camera_1", ros::init_options::NoSigintHandler);

    ros::NodeHandle nodehandle;

    try
    {
        std::unique_ptr<sas::CameraDriverDecklink> camera_driver_p(create_instance_from_ros_parameter_server(nodehandle));
        camera_driver_p->initialize();
        camera_driver_p->loop();

    }  catch (const std::exception& e)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() + "::Exception::" + e.what());
    }

    return 0;
}
