#include "stdafx.h"
#include <iostream>

#include "TrackerKudanRS.h"
#include "TrackerKudanGeneric.h"

// Anonymous namespace to avoid symbol collision
namespace com_samaust_trackerkudan_osvr {

	class TrackerKudanFusion {
	public:
		TrackerKudanFusion(OSVR_PluginRegContext ctx, Json::Value config) {
			osvrPose3SetIdentity(&m_state);

			m_useTimestamp = config.isMember("timestamp");
			m_usePositionTimestamp = m_useTimestamp && config["timestamp"].asString().compare("position") == 0;
			m_useKudanPositionOnly = config["position"].asString().compare("") == 0;
			m_cameraType = config["cameraType"].asInt();
			m_cameraIndex = config["cameraIndex"].asInt();
				

			if ((m_useOffset = config.isMember("offsetFromRotationCenter"))) {
				osvrVec3Zero(&m_offset);

				if (config["offsetFromRotationCenter"].isMember("x")) {
					osvrVec3SetX(&m_offset, config["offsetFromRotationCenter"]["x"].asDouble());
				}
				if (config["offsetFromRotationCenter"].isMember("y")) {
					osvrVec3SetY(&m_offset, config["offsetFromRotationCenter"]["y"].asDouble());
				}
				if (config["offsetFromRotationCenter"].isMember("z")) {
					osvrVec3SetZ(&m_offset, config["offsetFromRotationCenter"]["z"].asDouble());
				}
			}

			OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

			osvrDeviceTrackerConfigure(opts, &m_tracker);

			OSVR_DeviceToken token;
			osvrAnalysisSyncInit(ctx, config["name"].asCString(), opts, &token, &m_ctx);

			m_dev = new osvr::pluginkit::DeviceToken(token);

			m_orientationReader = OrientationReaderFactory::getReader(m_ctx, config["orientation"]);
			if (m_orientationReader == NULL) {
				std::cout << "[TrackerKudan-OSVR] Fusion Device: Orientation Reader not created" << std::endl;
			}

			if (m_useKudanPositionOnly) {
				if (m_cameraType == 0) {
					m_TrackerKudanRS = new TrackerKudanRS();
					m_TrackerKudanRS->init();
				}
				else {
					m_TrackerKudanGeneric = new TrackerKudanGeneric(m_cameraIndex);
					m_TrackerKudanGeneric->init();
				}
			}
			else {
				m_positionReader = PositionReaderFactory::getReader(m_ctx, config["position"]);
				if (m_positionReader == NULL) {
					std::cout << "[TrackerKudan-OSVR] Fusion Device: Position Reader not created" << std::endl;
				}
			}

			m_dev->sendJsonDescriptor(com_samaust_trackerkudan_osvr_json);
			m_dev->registerUpdateCallback(this);
		}

		~TrackerKudanFusion() { delete m_TrackerKudanRS; delete m_TrackerKudanGeneric; }

		OSVR_ReturnCode update() {
			osvrClientUpdate(m_ctx);

			OSVR_TimeValue timeValuePosition;
			OSVR_TimeValue timeValueOrientation;

			m_orientationReader->update(&m_state.rotation, &timeValueOrientation);

			if (m_useKudanPositionOnly) {
				if (m_cameraType == 0)
					m_TrackerKudanRS->update(&m_state.translation, &m_state.rotation);
				else
					m_TrackerKudanGeneric->update(&m_state.translation, &m_state.rotation);
			} 
			else {
				m_positionReader->update(&m_state.translation, &timeValuePosition);
			}

			if (m_useOffset) {
				Eigen::Quaterniond rotation = osvr::util::fromQuat(m_state.rotation);
				Eigen::Map<Eigen::Vector3d> translation = osvr::util::vecMap(m_state.translation);

				translation += rotation._transformVector(osvr::util::vecMap(m_offset));
			}

			if (m_useTimestamp) {
				OSVR_TimeValue timeValue = m_usePositionTimestamp ? timeValuePosition : timeValueOrientation;
				osvrDeviceTrackerSendPoseTimestamped(*m_dev, m_tracker, &m_state, 0, &timeValue);
			}
			else {
				osvrDeviceTrackerSendPose(*m_dev, m_tracker, &m_state, 0);
			}

			return OSVR_RETURN_SUCCESS;
		}

	private:
		IPositionReader* m_positionReader;
		IOrientationReader* m_orientationReader;

		OSVR_ClientContext m_ctx;
		OSVR_ClientInterface m_position;
		OSVR_ClientInterface m_orientation;

		osvr::pluginkit::DeviceToken* m_dev;
		OSVR_TrackerDeviceInterface m_tracker;
		OSVR_PoseState m_state;
		
		TrackerKudanRS *m_TrackerKudanRS;
		TrackerKudanGeneric *m_TrackerKudanGeneric;

		bool m_useOffset;
		OSVR_Vec3 m_offset;

		bool m_useTimestamp;
		bool m_usePositionTimestamp;
		bool m_useKudanPositionOnly;
		int m_cameraType;
		int m_cameraIndex;
	};

	class TrackerKudanFusionConstructor {
	public:
		TrackerKudanFusionConstructor() {}
		OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx, const char *params) {
			Json::Value root;
			if (params) {
				Json::Reader r;
				if (!r.parse(params, root)) {
					std::cerr << "[TrackerKudan-OSVR] Could not parse parameters!" << std::endl;
				}
			}

			if (!root.isMember("name") || !root.isMember("position") || !root.isMember("orientation")) {
				std::cerr << "[TrackerKudan-OSVR] Warning: got configuration, but no trackers specified"
					<< std::endl;
				return OSVR_RETURN_FAILURE;
			}

			osvr::pluginkit::registerObjectForDeletion(
				ctx, new TrackerKudanFusion(ctx, root));

			return OSVR_RETURN_SUCCESS;
		}
	};
} // namespace

OSVR_PLUGIN(com_samaust_trackerkudan_osvr) {
	std::cout << "[TrackerKudan-OSVR] Registering plugin" << std::endl;
	osvr::pluginkit::registerDriverInstantiationCallback(
		ctx, "TrackerKudanFusion", new com_samaust_trackerkudan_osvr::TrackerKudanFusionConstructor);

	return OSVR_RETURN_SUCCESS;
}
