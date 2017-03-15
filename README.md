# TrackerKudan-OSVR
Tracking plugin for OSVR using KudanCV and an OSVR orientation plugin 

## Instructions

Copy your Kudan license key to kLicenseKey variable in TrackerKudanGeneric.cpp and TrackerKudanRS.cpp.
Set the dependencies header and lib folders. For Kudan, you'll need libcurl.dll, KudanCV.h, libcurl.lib and a version of KudanCV.lib compiled with arbitrack support for Windows.
Compile x64 dll in Visual Studio 2015.
Copy TrackerKudan-OSVR\build_x64\bin\osvr-plugins-0\Release\com_samaust_trackerkudan_osvr.dll to C:\Program Files\OSVR\Runtime\bin\osvr-plugins-0 folder.
Copy opencv_world320.dll and libcurl.dll to C:\Program Files\OSVR\Runtime\bin folder.
Remove other plugins you don't need from C:\Program Files\OSVR\Runtime\bin\osvr-plugins-0.
Edit C:\Program Files\OSVR\Runtime\bin\osvr_server_config.json as necessary. A sample osvr_server_config.json is included in this repository.
Run osvr_server.exe.

Orientation tracking is done using the orientation tracker plugin set in osvr_server_config.json file. The tracker fusion is based on OSVR-fusion code.
Position tracking is done using a webcam and Kudan.

## Shorcuts

Recenter : CTRL + F12

## Dependencies

Kudan
OpenCV 3.2
jsoncpp
Intel RealSense SDK

## Credits

OSVR-fusion (https://github.com/simlrh/OSVR-fusion)