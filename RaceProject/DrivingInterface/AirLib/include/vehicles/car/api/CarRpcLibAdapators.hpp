// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarRpcLibAdapators_hpp
#define air_CarRpcLibAdapators_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/RpcLibAdapatorsBase.hpp"
#include "common/ImageCaptureBase.hpp"
#include "vehicles/car/api/CarApiBase.hpp"

#include "common/common_utils/WindowsApisCommonPre.hpp"
#include "rpc/msgpack.hpp"
#include "common/common_utils/WindowsApisCommonPost.hpp"

namespace msr { namespace airlib_rpclib {

class CarRpcLibAdapators : public RpcLibAdapatorsBase {
public:
    struct CarControls {
        float throttle = 0;
        float steering = 0;
        float brake = 0;
        bool handbrake = false;
        bool is_manual_gear = false;
        int manual_gear = 0;
        bool gear_immediate = true;

        MSGPACK_DEFINE_MAP(throttle, steering, brake, handbrake, is_manual_gear, manual_gear, gear_immediate);

        CarControls()
        {}

        CarControls(const msr::airlib::CarApiBase::CarControls& s)
        {
            throttle = s.throttle;
            steering = s.steering;
            brake = s.brake;
            handbrake = s.handbrake;
            is_manual_gear = s.is_manual_gear;
            manual_gear = s.manual_gear;
            gear_immediate = s.gear_immediate;
        }
        msr::airlib::CarApiBase::CarControls to() const
        {
            return msr::airlib::CarApiBase::CarControls(throttle, steering, brake, handbrake,
                is_manual_gear, manual_gear, gear_immediate);
        }
    };

    struct CarState {
        float speed;
        int gear;
        float rpm;
        float maxrpm;
        bool handbrake;
        KinematicsState kinematics_estimated;
        uint64_t timestamp;

        MSGPACK_DEFINE_MAP(speed, gear, rpm, maxrpm, handbrake, kinematics_estimated, timestamp);

        CarState()
        {}

        CarState(const msr::airlib::CarApiBase::CarState& s)
        {
            speed = s.speed;
            gear = s.gear;
            rpm = s.rpm;
            maxrpm = s.maxrpm;
            handbrake = s.handbrake;
            timestamp = s.timestamp;
            kinematics_estimated = s.kinematics_estimated;
        }
        msr::airlib::CarApiBase::CarState to() const
        {
            return msr::airlib::CarApiBase::CarState(
                speed, gear, rpm, maxrpm, handbrake, kinematics_estimated.to(), timestamp);
        }
    };
	
	// 2020 Rule Based Driving START
	struct Algo_User_api {
		std::array<std::vector<float>,1000> wayPoints;
		bool ac_RaceComplete_YN;
		std::array<std::vector<float>, 100> ac_block_points;
		int ac_player_current_lap;
		std::string settings_path;
		std::array<int, 100> ac_respawn_yaws;
		float ac_road_width_half;
		float ac_player_lap_progress;
		bool ac_rematch_YN;

		MSGPACK_DEFINE_MAP(wayPoints, ac_RaceComplete_YN, ac_block_points, ac_player_current_lap, settings_path, ac_respawn_yaws, ac_road_width_half, ac_player_lap_progress, ac_rematch_YN);

		Algo_User_api()
		{}

		Algo_User_api(const msr::airlib::CarApiBase::Algo_User_api& s)
		{
			wayPoints = s.wayPoints;
			ac_RaceComplete_YN = s.ac_RaceComplete_YN;
			ac_block_points = s.ac_block_points;
			ac_player_current_lap = s.ac_player_current_lap;
			settings_path = s.settings_path;
			ac_respawn_yaws = s.ac_respawn_yaws;
			ac_road_width_half = s.ac_road_width_half;
			ac_player_lap_progress = s.ac_player_lap_progress;
			ac_rematch_YN = s.ac_rematch_YN;
		}
		msr::airlib::CarApiBase::Algo_User_api to() const
		{
			return msr::airlib::CarApiBase::Algo_User_api(
				wayPoints, ac_RaceComplete_YN, ac_block_points, ac_player_current_lap, settings_path, ac_respawn_yaws, ac_road_width_half, ac_player_lap_progress, ac_rematch_YN);
		}
	};
	struct Algo_Admin_api {
		bool ac_RaceStart_YN;
		std::array<float, 8> ac_PlayerRaceTimes;
		float ac_player_first_lap_time;
		int ac_player_count;
		
		MSGPACK_DEFINE_MAP(ac_RaceStart_YN, ac_PlayerRaceTimes, ac_player_first_lap_time, ac_player_count);

		Algo_Admin_api()
		{}

		Algo_Admin_api(const msr::airlib::CarApiBase::Algo_Admin_api& s)
		{
			ac_RaceStart_YN = s.ac_RaceStart_YN;
			ac_PlayerRaceTimes = s.ac_PlayerRaceTimes;
			ac_player_first_lap_time = s.ac_player_first_lap_time;
			ac_player_count = s.ac_player_count;
		}
		msr::airlib::CarApiBase::Algo_Admin_api to() const
		{
			return msr::airlib::CarApiBase::Algo_Admin_api(
				ac_RaceStart_YN, ac_PlayerRaceTimes, ac_player_first_lap_time, ac_player_count);
		}
	};
	struct Algo_setResetLocation {
		bool resetLocation;

		MSGPACK_DEFINE_MAP(resetLocation);

		Algo_setResetLocation()
		{}

		Algo_setResetLocation(const msr::airlib::CarApiBase::Algo_setResetLocation& s)
		{
			resetLocation = s.resetLocation;
		}
		msr::airlib::CarApiBase::Algo_setResetLocation to() const
		{
			return msr::airlib::CarApiBase::Algo_setResetLocation(
				resetLocation);
		}
	};

	struct Algo_input_player_lap_progress{
		bool curr_lapprogress;

		MSGPACK_DEFINE_MAP(curr_lapprogress);

		Algo_input_player_lap_progress()
		{}

		Algo_input_player_lap_progress(const msr::airlib::CarApiBase::Algo_input_player_lap_progress& s)
		{
			curr_lapprogress = s.curr_lapprogress;
		}
		msr::airlib::CarApiBase::Algo_input_player_lap_progress to() const
		{
			return msr::airlib::CarApiBase::Algo_input_player_lap_progress(
				curr_lapprogress);
		}
	};

	struct Algo_ac_rematch_check {
		bool ac_rematch_check;

		MSGPACK_DEFINE_MAP(ac_rematch_check);

		Algo_ac_rematch_check()
		{}

		Algo_ac_rematch_check(const msr::airlib::CarApiBase::Algo_ac_rematch_check& s)
		{
			ac_rematch_check = s.ac_rematch_check;
		}
		msr::airlib::CarApiBase::Algo_ac_rematch_check to() const
		{
			return msr::airlib::CarApiBase::Algo_ac_rematch_check(
				ac_rematch_check);
		}
	};
	// 2020 Rule Based Driving END
};

}} //namespace


#endif
