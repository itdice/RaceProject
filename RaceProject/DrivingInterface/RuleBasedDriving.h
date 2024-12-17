#ifdef RULEBASEDDRIVING_EXPORTS
#define RULEBASEDDRIVING_API __declspec(dllexport)
#else
#define RULEBASEDDRIVING_API __declspec(dllimport)
#endif

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#ifdef RULEBASEDDRIVING_JNI_EXPORTS
#include <jni.h>    // 프로젝트 속성 >> C/C++ >> 일반 >> 추가 포함 디렉터리 >> %JAVA_HOME%\include,  %JAVA_HOME%\include\win32 추가 필요
#endif

#include "DrivingInfo.h"

namespace Car 
{
	typedef ControlValues(*CALLBACK_FUNCTION)(CarStateValues values);
	int RULEBASEDDRIVING_API StartDriving(CALLBACK_FUNCTION handler, bool bEnableApicontrol = true);
#ifdef RULEBASEDDRIVING_JNI_EXPORTS
	ControlValues control_driving(CarStateValues state_values);
#endif
}

