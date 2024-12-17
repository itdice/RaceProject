#ifndef DrivingInfo_h
#define DrivingInfo_h

#include <iostream>
#include <vector>

namespace Car
{
	using namespace std;

	struct ControlValues
	{
		float throttle;
		float steering;
		float brake;
	};

	struct CarsInfo
	{
		string name;
		float dist;
		float to_middle;
		float speed;
	};

	struct CarsPos
	{
		string name;
		float x;
		float y;
		float z;
	};

	struct DistanceIndex
	{
		int prev;
		int next;
	};

	struct ObstaclesInfo
	{
		DistanceIndex index;
		float dist;
		float to_middle;
	};

	struct Settings_Json
	{
		string player_name;
		string VehicleType;
		float x;
		float y;
		float z;
	};

	struct CarStateValues
	{
		bool collided;
		float collision_distance;
		float speed;
		float to_middle;
		float moving_angle;

		float moving_forward;
		float lap_progress;
		float half_road_limit;

		std::vector<float> track_forward_angles;
		std::vector<ObstaclesInfo> track_forward_obstacles;
		std::vector<CarsInfo> opponent_cars_info;
		std::vector<float> distance_to_way_points;
	};

	enum ReturnCodes { OK, CLIENT_FAILED, TIMEOUT };

#ifdef RULEBASEDDRIVING_JNI_EXPORTS
	struct CarsInfoToJava
	{
		float dist;
		float to_middle;
		float speed;
	};

	struct CarStateValuesToJava
	{
		bool collided;
		float collision_distance;
		float speed;
		float to_middle;
		float moving_angle;

		float moving_forward;
		float lap_progress;
		float half_road_limit;

		float track_forward_angles[10];
		float track_forward_obstacles[20];
		float opponent_cars_info[12];
		float distance_to_way_points[10];
	};
#endif
}

#endif