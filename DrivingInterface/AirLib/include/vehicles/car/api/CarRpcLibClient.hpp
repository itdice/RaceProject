// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarRpcLibClient_hpp
#define air_CarRpcLibClient_hpp

#include "common/Common.hpp"
#include <functional>
#include "common/CommonStructs.hpp"
#include "vehicles/car/api/CarApiBase.hpp"
#include "api/RpcLibClientBase.hpp"
#include "common/ImageCaptureBase.hpp"


namespace msr { namespace airlib {

class CarRpcLibClient : public RpcLibClientBase {
public:
    CarRpcLibClient(const string& ip_address = "localhost", uint16_t port = 41451, float timeout_sec = 60);

    void setCarControls(const CarApiBase::CarControls& controls, const std::string& vehicle_name = "");
    CarApiBase::CarState getCarState(const std::string& vehicle_name = "");
	
	// 2020 Rule Based Driving START
	CarApiBase::Algo_User_api getAlgoUserAPI(const std::string& vehicle_name = "");
	CarApiBase::Algo_Admin_api getAlgoAdminAPI(const std::string& vehicle_name = "");
	CarApiBase::Algo_setResetLocation setResetLocation(float x_pos, float y_pos, float yaw, const std::string& vehicle_name = "");
	CarApiBase::Algo_input_player_lap_progress input_player_lap_progress(float curr_lap_progress, const std::string& vehicle_name = "");
	CarApiBase::Algo_ac_rematch_check ac_rematch_check(const std::string& vehicle_name = "");
	// 2020 Rule Based Driving END
	
    virtual ~CarRpcLibClient();    //required for pimpl
};

}} //namespace
#endif
