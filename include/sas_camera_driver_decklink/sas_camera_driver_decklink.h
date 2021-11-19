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
#pragma once

#include <memory>

#include <ros/ros.h>

namespace sas
{

class CameraDriverDecklink
{
private:
    class CameraDriverDecklinkImplementation;
    std::unique_ptr<CameraDriverDecklinkImplementation> impl_;

public:
    CameraDriverDecklink(std::atomic_bool* kill_this_thread,
                         ros::NodeHandle& nodehandle,
                         const int& decklink_index);
    ~CameraDriverDecklink();

    void initialize();
    void loop();
};

}
