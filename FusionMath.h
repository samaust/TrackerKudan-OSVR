#include "stdafx.h"

namespace com_samaust_trackerkudan_osvr {

	void rpyFromQuaternion(OSVR_Quaternion* quaternion, OSVR_Vec3* rpy);
	void quaternionFromRPY(OSVR_Vec3* rpy, OSVR_Quaternion* quaternion);

}