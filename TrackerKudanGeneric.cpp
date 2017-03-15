#include "stdafx.h"

#include <Windows.h>

// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>

// Standard includes
#include <cmath>
#include <iostream>
#include <fstream>

#include "TrackerKudanGeneric.h"


/// Add your Kudan license key here
const std::string kLicenseKey = "";


TrackerKudanGeneric::TrackerKudanGeneric(int cameraIndex)
{
	m_cameraIndex = cameraIndex;
	m_x_recenter = 0;
	m_y_recenter = 0;
	m_z_recenter = -2.0f;
}

TrackerKudanGeneric::~TrackerKudanGeneric(void)
{
}

void TrackerKudanGeneric::init() {
	std::cout << "[TrackerKudan-OSVR] Initializing Tracker..." << std::endl;
	try {
		// Initialize Camera
		m_videoCapture.open(m_cameraIndex);
		if (!m_videoCapture.isOpened()) {
			std::cout << "[TrackerKudan-OSVR] Failed to open video capture" << std::endl;
		}

		cv::Mat frameColor;
		bool success = m_videoCapture.read(frameColor);
		m_frameSize.width = frameColor.size().width;
		m_frameSize.height = frameColor.size().height;
		std::cout << "[TrackerKudan-OSVR] Opened video capture at resolution " << m_frameSize.width << " x " << m_frameSize.height << std::endl;

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

OSVR_ReturnCode TrackerKudanGeneric::update(OSVR_PositionState* position, OSVR_OrientationState* orientation) {
	// Acquire frame from the camera
	cv::Mat frameColor;
	bool success = m_videoCapture.read(frameColor);

	//if (!success) {
	//	// Error
	//	// Return position
	//	position->data[0] = 0;
	//	position->data[1] = 0;
	//	position->data[2] = 0;
	//	std::cout << "[TrackerKudan-OSVR] frame read failed." << std::endl;
	//	return OSVR_RETURN_SUCCESS;
	//}

	//if (frameColor.type() != CV_8UC3)
	//{
	//	// Error
	//	// Return position
	//	position->data[0] = 0;
	//	position->data[1] = 0;
	//	position->data[2] = 0;
	//	std::cout << "[TrackerKudan-OSVR] frameColor type not CV_8UC3. type = " << frameColor.type() << std::endl;
	//	return OSVR_RETURN_SUCCESS;
	//}

	// Tracker requires greyscale data
	cv::Mat frameGrey;
	cv::cvtColor(frameColor, frameGrey, CV_BGR2GRAY);

	//if (!frameGrey.data) {
	//	// Error
	//	// Return position
	//	position->data[0] = 0;
	//	position->data[1] = 0;
	//	position->data[2] = 0;
	//	std::cout << "[TrackerKudan-OSVR] conversion to grayscale failed." << std::endl;
	//	return OSVR_RETURN_SUCCESS;
	//} 

	uchar *imageData = frameGrey.data;

	if (m_isRunningArbitrack) {
		KudanQuaternion orientationQuaternion = KudanQuaternion(orientation->data[1], orientation->data[2], orientation->data[3], orientation->data[0]);

		m_arbiTracker.setSensedOrientation(orientationQuaternion);

		// * TRACK *
		m_arbiTracker.processFrame(imageData, m_frameSize.width, m_frameSize.height, frameGrey.channels(), 0 /*padding*/, false);

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

	//std::cout << "[TrackerKudan-OSVR] position [x, y, z] = " << position->data[0] << ", " << position->data[1] << ", " << position->data[2] << std::endl;
	
	return OSVR_RETURN_SUCCESS;
}
