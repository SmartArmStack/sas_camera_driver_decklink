# sas_camera_driver_decklink

Make DeckLink captured images visible through ROS Noetic.

Tested on
- `DeckLink Quad HDMI Recorder - Blackmagic Design`

## License

The code in

`src/decklink`

`src/decklink_api`

are licensed by  BlackMagicDesign. Read their license in detail before using this software.
```
** 
** A copy of the Software is available free of charge at 
** https://www.blackmagicdesign.com/desktopvideo_sdk under the EULA.
** 
```

Everything else is licensed according to this repository's license.

## Installation

1- Download and install [Desktop Video](https://www.blackmagicdesign.com/developer/product/capture-and-playback) for Linux.

2- Add this repository to your catkin workspace and compile normally.

## Running

One example launch file is available at `launch/`. Configure it according to your needs.

`roslaunch sas_camera_driver_decklink camera_0.launch --screen`
