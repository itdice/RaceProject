// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarApiBase_hpp
#define air_CarApiBase_hpp

#include "common/VectorMath.hpp"
#include "common/CommonStructs.hpp"
#include "api/VehicleApiBase.hpp"
#include "physics/Kinematics.hpp"
#include "sensors/SensorBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/SensorFactory.hpp"

namespace msr { namespace airlib {

class CarApiBase : public VehicleApiBase  {
public:
    struct CarControls {
        float throttle = 0; /* 1 to -1 */
        float steering = 0; /* 1 to -1 */
        float brake = 0;    /* 1 to -1 */
        bool handbrake = false;
        bool is_manual_gear = false;
        int manual_gear = 0;
        bool gear_immediate = true;

        CarControls()
        {
        }
        CarControls(float throttle_val, float steering_val, float brake_val, bool handbrake_val,
            bool is_manual_gear_val, int manual_gear_val, bool gear_immediate_val)
            : throttle(throttle_val), steering(steering_val), brake(brake_val), handbrake(handbrake_val),
            is_manual_gear(is_manual_gear_val), manual_gear(manual_gear_val), gear_immediate(gear_immediate_val)
        {
        }
        void set_throttle(float throttle_val, bool forward)
        {
            if (forward) {
                is_manual_gear = false;
                manual_gear = 0;
                throttle = std::abs(throttle_val);
            }
            else {
                is_manual_gear = false;
                manual_gear = -1;
                throttle = - std::abs(throttle_val);
            }
        }
    };

    struct CarState {
        float speed;
        int gear;
        float rpm;
        float maxrpm;
        bool handbrake;
        Kinematics::State kinematics_estimated;
        uint64_t timestamp;
		
		// 2020 Rule Based Driving
		CarState() {}
        CarState(float speed_val, int gear_val, float rpm_val, float maxrpm_val, bool handbrake_val, 
            const Kinematics::State& kinematics_estimated_val, uint64_t timestamp_val)
            : speed(speed_val), gear(gear_val), rpm(rpm_val), maxrpm(maxrpm_val), handbrake(handbrake_val), 
              kinematics_estimated(kinematics_estimated_val), timestamp(timestamp_val)
        {
        }
    };

	// 2020 Rule Based Driving START
	struct Algo_User_api {
		std::array<std::vector<float>, 1000> wayPoints;
		bool ac_RaceComplete_YN;
		std::array<std::vector<float>, 100> ac_block_points;
		int ac_player_current_lap;
		std::string settings_path;
		std::array<int, 100> ac_respawn_yaws;
		float ac_road_width_half;
		float ac_player_lap_progress;
		bool ac_rematch_YN;

		Algo_User_api() {}
		Algo_User_api(std::array<std::vector<float>, 1000> wayPoints_val, bool ac_RaceComplete_YN_val, std::array<std::vector<float>, 100> ac_block_points_val, int ac_player_current_lap_val, std::string settings_path_val, std::array<int, 100> ac_respawn_yaws_val, float ac_road_width_half_val, float ac_player_lap_progress_val, bool ac_rematch_YN_val)
			: wayPoints(wayPoints_val), ac_RaceComplete_YN(ac_RaceComplete_YN_val), ac_block_points(ac_block_points_val), ac_player_current_lap(ac_player_current_lap_val), settings_path(settings_path_val), ac_respawn_yaws(ac_respawn_yaws_val), ac_road_width_half(ac_road_width_half_val), ac_player_lap_progress(ac_player_lap_progress_val), ac_rematch_YN(ac_rematch_YN_val)
		{
		}
	};

	struct Algo_Admin_api {
		bool ac_RaceStart_YN;
		std::array<float, 8> ac_PlayerRaceTimes;
		float ac_player_first_lap_time;
		int ac_player_count;

		Algo_Admin_api(bool ac_RaceStart_YN_val, std::array<float, 8> ac_PlayerRaceTimes_val, float ac_player_first_lap_time_val, int ac_player_count_val)
			: ac_RaceStart_YN(ac_RaceStart_YN_val), ac_PlayerRaceTimes(ac_PlayerRaceTimes_val), ac_player_first_lap_time(ac_player_first_lap_time_val), ac_player_count(ac_player_count_val)
		{
		}
	};

	struct Algo_setResetLocation {
		bool resetLocation;

		Algo_setResetLocation() {}
		Algo_setResetLocation(bool resetLocation_val)
			: resetLocation(resetLocation_val)
		{
		}
	};

	struct Algo_input_player_lap_progress {
		bool curr_lapprogress;

		Algo_input_player_lap_progress(bool curr_lapprogress_val)
			: curr_lapprogress(curr_lapprogress_val)
		{
		}
	};

	struct Algo_ac_rematch_check {
		bool ac_rematch_check;

		Algo_ac_rematch_check(bool ac_rematch_check_val)
			: ac_rematch_check(ac_rematch_check_val)
		{
		}
	};
	// 2020 Rule Based Driving END

public:

    // TODO: Temporary constructor for the Unity implementation which does not use the new Sensor Configuration Settings implementation.
	//CarApiBase() {}

    CarApiBase(const AirSimSettings::VehicleSetting* vehicle_setting, 
        std::shared_ptr<SensorFactory> sensor_factory, 
        const Kinematics::State& state, const Environment& environment)
    {
        initialize(vehicle_setting, sensor_factory, state, environment);
    }

    //default implementation so derived class doesn't have to call on VehicleApiBase
    virtual void reset() override
    {
        VehicleApiBase::reset();

        //reset sensors last after their ground truth has been reset
        getSensors().reset();
    }
    virtual void update() override
    {
        VehicleApiBase::update();

        getSensors().update();
    }
    void reportState(StateReporter& reporter) override
    {
        getSensors().reportState(reporter);
    }

    // sensor helpers
    virtual const SensorCollection& getSensors() const override
    {
        return sensors_;
    }

    SensorCollection& getSensors()
    {
        return sensors_;
    }

    void initialize(const AirSimSettings::VehicleSetting* vehicle_setting, 
        std::shared_ptr<SensorFactory> sensor_factory, 
        const Kinematics::State& state, const Environment& environment)
    {
        sensor_factory_ = sensor_factory;

        sensor_storage_.clear();
        sensors_.clear();
        
        addSensorsFromSettings(vehicle_setting);

        getSensors().initialize(&state, &environment);
    }

    void addSensorsFromSettings(const AirSimSettings::VehicleSetting* vehicle_setting)
    {
        // use sensors from vehicle settings; if empty list, use default sensors.
        // note that the vehicle settings completely override the default sensor "list";
        // there is no piecemeal add/remove/update per sensor.
        const std::map<std::string, std::unique_ptr<AirSimSettings::SensorSetting>>& sensor_settings
            = vehicle_setting->sensors.size() > 0 ? vehicle_setting->sensors : AirSimSettings::AirSimSettings::singleton().sensor_defaults;

        sensor_factory_->createSensorsFromSettings(sensor_settings, sensors_, sensor_storage_);
    }

    virtual void setCarControls(const CarControls& controls) = 0;
    virtual CarState getCarState() const = 0;
	
	// 2020 Rule Based Driving START
	virtual Algo_User_api getAlgoUserAPI() const = 0;
	virtual Algo_Admin_api getAlgoAdminAPI() const = 0;
	virtual Algo_setResetLocation setResetLocation(float x_pos, float y_pos, float yaw) const = 0;
	virtual Algo_input_player_lap_progress input_player_lap_progress(float curr_lapprogress) const = 0;
	virtual Algo_ac_rematch_check ac_rematch_check() const = 0;
	// 2020 Rule Based Driving END

    virtual const CarApiBase::CarControls& getCarControls() const = 0;

    virtual ~CarApiBase() = default;

    std::shared_ptr<const SensorFactory> sensor_factory_;
    SensorCollection sensors_; //maintains sensor type indexed collection of sensors
    vector<unique_ptr<SensorBase>> sensor_storage_; //RAII for created sensors
};


}} //namespace
#endif