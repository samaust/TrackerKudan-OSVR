#pragma once
#include "stdafx.h"

#include <memory>

// OpenCV is required for reading the webcam
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// include the Kudan Tracker Interface
#include "KudanCV.h"

// RealSense
#include <pxcsensemanager.h>
//#include <pxcimage.h>

class TrackerKudanRS
{
public:
	TrackerKudanRS();
	~TrackerKudanRS();

	void init();
	OSVR_ReturnCode update(OSVR_PositionState* position, OSVR_OrientationState* orientation);

private:
	osvr::pluginkit::DeviceToken m_dev;
	OSVR_TrackerDeviceInterface m_tracker;

	PXCSenseManager *m_pxcSenseManager;
	cv::Size m_frameSize;

	bool m_isRunningArbitrack;
	bool m_doStartArbitrack;

	KudanArbiTracker m_arbiTracker;

	float m_x_recenter;
	float m_y_recenter;
	float m_z_recenter;
};