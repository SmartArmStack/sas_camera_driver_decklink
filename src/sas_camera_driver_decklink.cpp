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

#include <stdio.h>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <tuple>
#include <vector>

#include <opencv2/opencv.hpp>

#include "decklink/platform.h"
#include "decklink/Bgra32VideoFrame.h"
#include "sas_decklink/DeckLinkInputDevice.h"
#include "decklink_api/DeckLinkAPI.h"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

namespace sas
{

// Pixel format tuple encoding {BMDPixelFormat enum, Pixel format display name}
const std::vector<std::tuple<BMDPixelFormat, std::string>> kSupportedPixelFormats
{
    std::make_tuple(bmdFormat8BitYUV, "8 bit YUV (4:2:2)"),
            std::make_tuple(bmdFormat10BitYUV, "10 bit YUV (4:2:2)"),
            std::make_tuple(bmdFormat8BitARGB, "8 bit ARGB (4:4:4)"),
            std::make_tuple(bmdFormat8BitBGRA, "8 bit BGRA (4:4:4)"),
            std::make_tuple(bmdFormat10BitRGB, "10 bit RGB (4:4:4)"),
            std::make_tuple(bmdFormat12BitRGB, "12 bit RGB (4:4:4)"),
            std::make_tuple(bmdFormat12BitRGBLE, "12 bit RGB (4:4:4) Little-Endian"),
            std::make_tuple(bmdFormat10BitRGBX, "10 bit RGBX (4:4:4)"),
            std::make_tuple(bmdFormat10BitRGBXLE, "10 bit RGBX (4:4:4) Little-Endian"),
};
enum {
    kPixelFormatValue = 0,
    kPixelFormatString
};

class CameraDriverDecklink::CameraDriverDecklinkImplementation
{
public:
    int image_index = 0;
    int image_width = -1; //Initially we don't know
    int image_height = -1; //Initially we don't know
    int image_channels = 4; //For now, it's a bgra image so 4 channels

    std::atomic_bool* kill_this_thread_;
    IDeckLinkVideoFrame* bgra32Frame;
    IDeckLinkVideoFrame* receivedVideoFrame;
    IDeckLinkVideoConversion* deckLinkFrameConverter;
    const int decklink_index_;

    IDeckLinkIterator* deckLinkIterator;
    IDeckLink* deckLink;
    DeckLinkInputDevice* selectedDeckLinkInput;
    cv::Mat cv_image_rgba;

    const std::string node_prefix_;
    image_transport::ImageTransport image_transport_;
    image_transport::Publisher image_publisher_;

    CameraDriverDecklinkImplementation(std::atomic_bool* kill_this_thread,
                                       ros::NodeHandle& node_handle,
                                       const int& decklink_index):
        kill_this_thread_(kill_this_thread),
        decklink_index_(decklink_index),
        image_transport_(node_handle)
    {
        image_publisher_ = image_transport_.advertise(std::string("/sas/decklink/") +
                                   std::to_string(decklink_index) +
                                   std::string("/get/video"), 1
                                   );
    }

    void set_image(void)
    {
        try
        {
                if (bgra32Frame != NULL)
                {
                    if (image_index == 0)
                    {
                        image_width = bgra32Frame->GetWidth();
                        image_height = bgra32Frame->GetHeight();
                    }

                    uint8_t* buffer;
                    bgra32Frame->GetBytes((void**)&buffer);
                    cv_image_rgba = cv::Mat(
                                image_height,
                                image_width,
                                CV_8UC4,
                                buffer); //Clone here if someone can mess up the buffer midway

                    try
                    {
                        cv_bridge::CvImage cv_bridge_image(std_msgs::Header(),
                                                           "bgra8",
                                                           cv_image_rgba);
                        image_publisher_.publish(cv_bridge_image.toImageMsg());
                        ros::spinOnce();
                    }
                    catch (cv_bridge::Exception& e)
                    {
                        ROS_ERROR("cv_bridge exception: %s", e.what());
                        return;
                    }

                    image_index++;
                }
                else
                {
                    fprintf(stderr, "Image is NULL, ignoring...");
                }
        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "%s", std::string(std::string("Exception caught in set_image()") + e.what()).c_str());
        }
    }


    void capture_thread_loop()
    {
        int							captureFrameCount = 0;
        HRESULT						result = S_OK;

        //Persistent data
        receivedVideoFrame = NULL;
        deckLinkFrameConverter = NULL;
        bgra32Frame = NULL;

        // Create frame conversion instance
        result = GetDeckLinkVideoConversion(&deckLinkFrameConverter);
        if (result != S_OK)
            return;

        while (not *kill_this_thread_)
        {
            try
            {
                if (!selectedDeckLinkInput->GetLatestFrame(&receivedVideoFrame))
                {
                    //fprintf(stderr, "No frame to get!\n");
                    continue;
                }
                else
                {
                    //Clean queue if the current frame was obtained successfully
                    selectedDeckLinkInput->CleanQueue();

                    if (receivedVideoFrame->GetPixelFormat() == bmdFormat8BitBGRA)
                    {
                        throw(std::runtime_error("Not implemented yet"));
                    }
                    else
                    {
                        bgra32Frame = new Bgra32VideoFrame(receivedVideoFrame->GetWidth(), receivedVideoFrame->GetHeight(), receivedVideoFrame->GetFlags());

                        result = deckLinkFrameConverter->ConvertFrame(receivedVideoFrame, bgra32Frame);
                        if (FAILED(result))
                        {
                            fprintf(stderr, "Frame conversion to BGRA was unsuccessful\n");
                            *kill_this_thread_ = true;
                        }

                        set_image();

                        bgra32Frame->Release();
                    }
                }

                if (receivedVideoFrame != NULL)
                {
                    receivedVideoFrame->Release();
                    receivedVideoFrame = NULL;
                }
            }
            catch (const std::exception& e)
            {
                fprintf(stderr, "%s", std::string(std::string("Exception caught in CaptureStills()") + e.what()).c_str());
                *kill_this_thread_ = true;
            }
        }

        if (deckLinkFrameConverter != NULL)
        {
            deckLinkFrameConverter->Release();
            deckLinkFrameConverter = NULL;
        }

        if (selectedDeckLinkInput != NULL)
        {
            selectedDeckLinkInput->Release();
            selectedDeckLinkInput = NULL;
        }

        if (deckLinkIterator != NULL)
        {
            deckLinkIterator->Release();
            deckLinkIterator = NULL;
        }

        fprintf(stderr, "Capture ended.\n");
    }

    int start_capture_thread()
    {
        // Configuration Flags
        int							pixelFormatIndex = 0;
        bool						enableFormatDetection = false;
        std::string					filenamePrefix;
        HRESULT						result;
        int							exitStatus = 1;
        int							idx;
        bool						supportsFormatDetection = false;

        deckLinkIterator = NULL;
        deckLink = NULL;
        selectedDeckLinkInput = NULL;
        const int displayModeIndex = -1;

        BMDDisplayMode				selectedDisplayMode = bmdModeNTSC;
        std::string					selectedDisplayModeName;
        std::vector<std::string>	deckLinkDeviceNames;

        result = GetDeckLinkIterator(&deckLinkIterator);
        if (result != S_OK)
            return 3;

        if (decklink_index_ < 0)
        {
            fprintf(stderr, "You must select a device\n");
        }

        // Obtain the required DeckLink device
        idx = 0;

        while ((result = deckLinkIterator->Next(&deckLink)) == S_OK)
        {
            dlstring_t deckLinkName;

            result = deckLink->GetDisplayName(&deckLinkName);
            if (result == S_OK)
            {
                deckLinkDeviceNames.push_back(DlToStdString(deckLinkName));
                DeleteString(deckLinkName);
            }

            if (idx++ == decklink_index_)
            {
                // Check that selected device supports capture
                IDeckLinkProfileAttributes*	deckLinkAttributes = NULL;
                int64_t						ioSupportAttribute = 0;
                dlbool_t					formatDetectionSupportAttribute;

                result = deckLink->QueryInterface(IID_IDeckLinkProfileAttributes, (void**)&deckLinkAttributes);

                if (result != S_OK)
                {
                    fprintf(stderr, "Unable to get IDeckLinkAttributes interface\n");
                    return 4;
                }

                // Check whether device supports cpature
                result = deckLinkAttributes->GetInt(BMDDeckLinkVideoIOSupport, &ioSupportAttribute);

                if ((result != S_OK) || ((ioSupportAttribute & bmdDeviceSupportsCapture) == 0))
                {
                    fprintf(stderr, "Selected device does not support capture\n");
                }
                else
                {
                    // Check if input mode detection is supported.
                    result = deckLinkAttributes->GetFlag(BMDDeckLinkSupportsInputFormatDetection, &formatDetectionSupportAttribute);
                    supportsFormatDetection = (result == S_OK) && (formatDetectionSupportAttribute != false);

                    selectedDeckLinkInput = new DeckLinkInputDevice(deckLink);
                }

                deckLinkAttributes->Release();
            }

            deckLink->Release();
        }

        // Get display modes from the selected decklink output
        if (selectedDeckLinkInput != NULL)
        {
            result = selectedDeckLinkInput->Init();
            if (result != S_OK)
            {
                fprintf(stderr, "Unable to initialize DeckLink input interface");
                return 5;
            }

            // Get the display mode
            if ((displayModeIndex < -1) || (displayModeIndex >= (int)selectedDeckLinkInput->GetDisplayModeList().size()))
            {
                fprintf(stderr, "You must select a valid display mode\n");
            }
            else if (displayModeIndex == -1)
            {
                if (!supportsFormatDetection)
                {
                    fprintf(stderr, "Format detection is not supported on this device\n");
                }
                else
                {
                    enableFormatDetection = true;

                    // Format detection still needs a valid mode to start with
                    selectedDisplayMode = bmdModeNTSC;
                    selectedDisplayModeName = "Automatic mode detection";
                    pixelFormatIndex = 0;
                }
            }
            else if ((pixelFormatIndex < 0) || (pixelFormatIndex >= (int)kSupportedPixelFormats.size()))
            {
                fprintf(stderr, "You must select a valid pixel format\n");
            }
            else
            {
                dlbool_t				displayModeSupported;
                dlstring_t				displayModeNameStr;
                IDeckLinkDisplayMode*	displayMode = selectedDeckLinkInput->GetDisplayModeList()[displayModeIndex];

                result = displayMode->GetName(&displayModeNameStr);
                if (result == S_OK)
                {
                    selectedDisplayModeName = DlToStdString(displayModeNameStr);
                    DeleteString(displayModeNameStr);
                }

                selectedDisplayMode = displayMode->GetDisplayMode();

                // Check display mode is supported with given options
                result = selectedDeckLinkInput->GetDeckLinkInput()->DoesSupportVideoMode(bmdVideoConnectionUnspecified,
                                                                                         selectedDisplayMode,
                                                                                         std::get<kPixelFormatValue>(kSupportedPixelFormats[pixelFormatIndex]),
                                                                                         bmdNoVideoInputConversion,
                                                                                         bmdSupportedVideoModeDefault,
                                                                                         NULL,
                                                                                         &displayModeSupported);
                if ((result != S_OK) || (!displayModeSupported))
                {
                    fprintf(stderr, "Display mode %s with pixel format %s is not supported by device\n",
                            selectedDisplayModeName.c_str(),
                            std::get<kPixelFormatString>(kSupportedPixelFormats[pixelFormatIndex]).c_str()
                            );
                }
            }
        }
        else
        {
            fprintf(stderr, "Invalid input device selected\n");
        }


        // Start capturing
        result = selectedDeckLinkInput->StartCapture(selectedDisplayMode, std::get<kPixelFormatValue>(kSupportedPixelFormats[pixelFormatIndex]), enableFormatDetection);
        if (result != S_OK)
            return 7;

        // Print the selected configuration
        fprintf(stderr,
                "DecklinkInteface + ARUCO (c) 2021 Murilo M. Marinho.\n"
            " Initializing capture with the following configuration:\n"
            " - Capture device: %s\n"
            " - Video mode: %s\n"
            " - Pixel format: %s\n",
                selectedDeckLinkInput->GetDeviceName().c_str(),
                selectedDisplayModeName.c_str(),
                std::get<kPixelFormatString>(kSupportedPixelFormats[pixelFormatIndex]).c_str()
                );

        return exitStatus;
    }
};

CameraDriverDecklink::CameraDriverDecklink(std::atomic_bool *kill_this_thread, ros::NodeHandle& nodehandle,
                                           const int& decklink_index):
    impl_(new CameraDriverDecklinkImplementation(kill_this_thread,
                                                 nodehandle,
                                                 decklink_index)
          )
{

}


void CameraDriverDecklink::initialize()
{
    impl_->start_capture_thread();
}

void CameraDriverDecklink::loop()
{
    impl_->capture_thread_loop();
}

CameraDriverDecklink::~CameraDriverDecklink() = default;

}
