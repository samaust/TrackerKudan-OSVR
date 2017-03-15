#include "stdafx.h"

#include <Windows.h>

// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>

// Standard includes
#include <cmath>
#include <iostream>
#include <fstream>

#include "TrackerKudanRS.h"


/// Add your Kudan license key here
const std::string kLicenseKey = "";


TrackerKudanRS::TrackerKudanRS()
{
	m_x_recenter = 0;
	m_y_recenter = 0;
	m_z_recenter = -2.0f;
}

TrackerKudanRS::~TrackerKudanRS(void)
{
	// Clean RealSense Camera
	m_pxcSenseManager->Release();
}

void TrackerKudanRS::init() {
	std::cout << "[TrackerKudan-OSVR] Initializing Tracker..." << std::endl;
	try {
		// Initialize RealSense Camera
		//Define some parameters for the camera
		m_frameSize = cv::Size(640, 480);
		float frameRate = 60;

		//Initialize the RealSense Manager
		m_pxcSenseManager = PXCSenseManager::CreateInstance();
		if (!m_pxcSenseManager) {
			std::cout << "[TrackerKudan-OSVR] Initialization Failed. CreateInstance() failed." << std::endl;
		}

		//Enable the streams to be used
		pxcStatus status;
		status = m_pxcSenseManager->EnableStream(PXCCapture::STREAM_TYPE_COLOR, m_frameSize.width, m_frameSize.height, frameRate);
		if (status < PXC_STATUS_NO_ERROR) {
			std::cout << "[TrackerKudan-OSVR] Initialization Failed. Failed to enable Stream" << std::endl;
		}

		//Initialize the pipeline
		status = m_pxcSenseManager->Init();
		if (status < PXC_STATUS_NO_ERROR) {
			std::cout << "[TrackerKudan-OSVR] Initialization Failed. SenseManager Init() failed." << std::endl;
		}

		// Set up the intrinsics, by setting the size, and using the function to guess the intrinsics (if they are known, use setIntrinsics())
		KudanCameraParameters cameraParameters;
		cameraParameters.setSize(m_frameSize.width, m_frameSize.height);
		cameraParameters.guessIntrinsics();

		// Retrieve the camera calibration matrix from the parameters, because it is needed for drawing:
		KudanMatrix3 K = cameraParameters.getMatrix();

		// Create the tracker:
		KudanImageTracker tracker;

		// Set your API key here
		tracker.setApiKey(kLicenseKey);

		// set global tracker properties:
		tracker.setMaximumSimultaneousTracking(2);

		// The tracker needs to know the intrinsics:
		tracker.setCameraParameters(cameraParameters);

		/* Note: it is NOT possible to create trackables without initialising them. This will not be allowed:
		std::shared_ptr<KudanImageTrackable> blankTrackable = std::make_shared<KudanImageTrackable>();
		*/

		// Also want to be able to run arbitrack
		m_arbiTracker.setApiKey(kLicenseKey);
		m_arbiTracker.setCameraParameters(cameraParameters);

		m_isRunningArbitrack = false;
		m_doStartArbitrack = true;

		std::cout << "[TrackerKudan-OSVR] Tracker initialized" << std::endl;
	}
	catch (KudanException &e) {
		printf("[TrackerKudan-OSVR] Tracker initialization failed. Caught exception: %s \n", e.what());
	}
}

OSVR_ReturnCode TrackerKudanRS::update(OSVR_PositionState* position, OSVR_OrientationState* orientation) {
	// Acquire frame from the camera
	m_pxcSenseManager->AcquireFrame();
	PXCCapture::Sample *sample = m_pxcSenseManager->QuerySample();

	if (!sample) {
		// Error
		// Return position
		position->data[0] = 0;
		position->data[1] = 0;
		position->data[2] = 0;
		//Release the memory from the frame
		m_pxcSenseManager->ReleaseFrame();
		std::cout << "[TrackerKudan-OSVR] QuerySample failed." << std::endl;
		return OSVR_RETURN_SUCCESS;
	}

	// Convert to an OpenCV Mat
	cv::Mat frameColor = cv::Mat::zeros(m_frameSize, CV_8UC3);

	PXCImage::ImageData data;
	pxcStatus status;
	status = sample->color->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB24, &data);

	if (status < PXC_STATUS_NO_ERROR) {
		// Error
		// Return position
		position->data[0] = 0;
		position->data[1] = 0;
		position->data[2] = 0;
		//Release the memory from the frame
		m_pxcSenseManager->ReleaseFrame();
		std::cout << "[TrackerKudan-OSVR] AcquireAccess failed." << std::endl;
		return OSVR_RETURN_SUCCESS;
	}

	//int width = sample->color->QueryInfo().width;
	//int height = sample->color->QueryInfo().height;
	frameColor = cv::Mat(cv::Size(m_frameSize.width, m_frameSize.height), CV_8UC3, data.planes[0], data.pitches[0]);
	sample->color->ReleaseAccess(&data);

	//if (frameColor.type() != CV_8UC3)
	//{
	//	// Error
	//	// Return position
	//	position->data[0] = 0;
	//	position->data[1] = 0;
	//	position->data[2] = 0;
	//	//Release the memory from the frame
	//	m_pxcSenseManager->ReleaseFrame();
	//	std::cout << "[TrackerKudan-OSVR] frameColor type not CV_8UC3. type = " << frameColor.type() << std::endl;
	//	return OSVR_RETURN_SUCCESS;
	//}

	// Tracker requires greyscale data:
	cv::Mat frameGrey = cv::Mat::zeros(m_frameSize, CV_8UC1);
	cv::cvtColor(frameColor, frameGrey, CV_BGR2GRAY);

	if (!frameGrey.data) {
		// Error
		// Return position
		position->data[0] = 0;
		position->data[1] = 0;
		position->data[2] = 0;
		//Release the memory from the frame
		m_pxcSenseManager->ReleaseFrame();
		std::cout << "[TrackerKudan-OSVR] frameGrey empty." << std::endl;
		return OSVR_RETURN_SUCCESS;
	} 

	uchar *imageData = frameGrey.data;
	int imageWidth = frameGrey.size().width;
	int imageHeight = frameGrey.size().height;

	if (m_isRunningArbitrack) {
		KudanQuaternion orientationQuaternion = KudanQuaternion(orientation->data[1], orientation->data[2], orientation->data[3], orientation->data[0]);

		m_arbiTracker.setSensedOrientation(orientationQuaternion);

		// * TRACK *
		m_arbiTracker.processFrame(imageData, imageWidth, imageHeight, frameGrey.channels(), 0 /*padding*/, false);

		KudanVector3 arbitrackPosition = m_arbiTracker.getPosition();
		//KudanQuaternion arbitrackOrientation = m_arbiTracker.getOrientation();

		// Recenter if CTRL + F12 is pressed
		if (GetAsyncKeyState(VK_CONTROL) & 0x8000)
		{
			if (GetAsyncKeyState(VK_F12) & 0x8000)
			{
				m_x_recenter = arbitrackPosition.x / 100.0f;
				m_y_recenter = -arbitrackPosition.y / 100.0f;
				m_z_recenter = -arbitrackPosition.z / 100.0f;
			}
		}

		// Return position
		// Convert to m
		position->data[0] = -arbitrackPosition.x / 100.0f + m_x_recenter;
		position->data[1] = arbitrackPosition.y / 100.0f + m_y_recenter;
		position->data[2] = arbitrackPosition.z / 100.0f + m_z_recenter;
	}
	else {
		// If arbitrack needs to be started, but has not been (i.e. no trackables) then do so now, from a pose in front of the camera
		if (m_doStartArbitrack) {
			m_isRunningArbitrack = true;

			// Start via a position and quaternion:
			//                    KudanVector3 startPosition(0,0,200); // in front of the camera
			//                    KudanQuaternion startOrientation(1,0,0,0); // without rotation
			//                    m_arbiTracker.start(startPosition, startOrientation);

			// Start via a 4x4 matrix:
			// Set to identity:
			KudanMatrix4 transform;
			for (int i = 0; i < 4; i++) {
				transform(i, i) = 1.0;
			}
			// z coordinate of T, in third column
			transform(2, 3) = 200;

			m_arbiTracker.start(transform);

			printf("[TrackerKudan-OSVR] Starting Arbitrack from here \n");
			m_doStartArbitrack = false;
		}

		// Return position
		position->data[0] = 0;
		position->data[1] = 0;
		position->data[2] = 0;
	}
	
	//Release the memory from the frame
	m_pxcSenseManager->ReleaseFrame();

	//std::cout << "[TrackerKudan-OSVR] position [x, y, z] = " << position->data[0] << ", " << position->data[1] << ", " << position->data[2] << std::endl;
	
	return OSVR_RETURN_SUCCESS;
}
