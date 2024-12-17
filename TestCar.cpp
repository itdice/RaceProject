#include <iostream>
#include <algorithm>
#include <vector>
#include "RuleBasedDriving.h"

using namespace std;
using namespace Car;

bool is_debug = false;  // true -> 차량 상황 디버깅용 데이터 출력, false -> 차량 상황 디버깅 없이 수행
bool enable_api_control = true;	 // true -> 코드를 통한 자율주행, false -> 키보드를 이용한 수동주행

ControlValues control_driving(CarStateValues sensing_info)
{
	if (is_debug)
	{
		cout << "=========================================================" << endl;
		cout << "[MyCar] to middle: " << sensing_info.to_middle << endl;

		cout << "[MyCar] collided: " << sensing_info.collided << endl;
		cout << "[MyCar] car speed: " << sensing_info.speed << "km/h" << endl;

		cout << "[MyCar] is moving forward: " << sensing_info.moving_forward << endl;
		cout << "[MyCar] moving angle: " << sensing_info.moving_angle << endl;
		cout << "[MyCar] lap_progress: " << sensing_info.lap_progress << endl;

		cout << "[MyCar] track_forward_angles: ";
		for (int i = 0; i < sensing_info.track_forward_angles.size(); i++)
		{
			cout << sensing_info.track_forward_angles[i] << ", ";
		}
		cout << endl;

		cout << "[MyCar] track_forward_obstacles: ";
		for (int i = 0; i < sensing_info.track_forward_obstacles.size(); i++)
		{
			cout << "{dist: " << sensing_info.track_forward_obstacles[i].dist 
				 << ", to_middle: " << sensing_info.track_forward_obstacles[i].to_middle << "}, ";
		}
		cout << endl;

		cout << "[MyCar] opponent_cars_info: ";
		for (int i = 0; i < sensing_info.opponent_cars_info.size(); i++)
		{
			cout << "{dist: " << sensing_info.opponent_cars_info[i].dist 
				 << ", to_middle: " << sensing_info.opponent_cars_info[i].to_middle 
				 << ", speed: " << sensing_info.opponent_cars_info[i].speed << "km/h}, ";
		}
		cout << endl;

		cout << "[MyCar] distance_to_way_points: ";
		for (int i = 0; i < sensing_info.distance_to_way_points.size(); i++)
		{
			cout << sensing_info.distance_to_way_points[i] << ", ";
		}
		cout << endl;

		cout << "=========================================================" << endl;
	}

	ControlValues car_controls;

	// ===========================================================
	// Area for writing code about driving rule
	// ===========================================================
	// Editing area starts from here
	// ===========================================================

	// 도로의 실제 폭의 1/2 로 계산됨
	float half_load_width = sensing_info.half_road_limit - 1.25f;

	// 차량 핸들 조정을 위해 참고할 전방의 커브 값 가져오기
	int angle_num = (int)(sensing_info.speed / 45);
	float ref_angle = angle_num > 0 ? sensing_info.track_forward_angles[angle_num] : 0;

	// 차량의 차선 중앙 정렬을 위한 미세 조정 값 계산
	float middle_add = (sensing_info.to_middle / 80) * -1;

	// 전방의 커브 각도에 따라 throttle 값을 조절하여 속도를 제어함
	float throttle_factor = 0.6f / (abs(ref_angle) + 0.1f);
	if (throttle_factor > 0.11f) throttle_factor = 0.11f;   // throttle 값을 최대 0.81 로 설정
	float set_throttle = 0.7f + throttle_factor;
	if (sensing_info.speed < 60) set_throttle = 0.9f;  // 속도가 60Km/h 이하인 경우 0.9 로 설정
	if (sensing_info.speed > 80) set_throttle = 0.6f;  // 최대속도를 80km/h로 설정

	// 차량의 Speed 에 따라서 핸들을 돌리는 값을 조정함
	float steer_factor = sensing_info.speed * 1.5f;
	if (sensing_info.speed > 70) steer_factor = sensing_info.speed * 0.85f;
	if (sensing_info.speed > 100) steer_factor = sensing_info.speed * 0.7f;

	// (참고할 전방의 커브 - 내 차량의 주행 각도) / (계산된 steer factor) 값으로 steering 값을 계산
	float set_steering = (ref_angle - sensing_info.moving_angle) / (steer_factor + 0.001f);

	// 차선 중앙정렬 값을 추가로 고려함
	set_steering += middle_add;

	///////////////////////////////////////
	//// 긴급 및 예외 상황 처리 시작 /////////
	///////////////////////////////////////
	bool full_throttle = true;
	bool emergency_brake = false;

	// 전방 커브의 각도가 큰 경우 속도를 제어함
	// 차량 핸들 조정을 위해 참고하는 커브 보다 조금 더 멀리 참고하여 미리 속도를 줄임
	int road_range = (int)(sensing_info.speed / 30);
	for (int i = 0; i < road_range; i++) {
		float fwd_angle = abs(sensing_info.track_forward_angles[i]);
		if (fwd_angle > 45) {   // 커브가 45도 이상인 경우 brake, throttle 을 제어
			full_throttle = false;
		}
		if (fwd_angle > 80) {   // 커브가 80도 이상인 경우 steering 까지 추가로 제어
			emergency_brake = true;
			break;
		}
	}

	// brake, throttle 제어
	float set_brake = 0.0f;
	if (!full_throttle) {
		if (sensing_info.speed > 100) {
			set_brake = 0.3f;
		}
		if (sensing_info.speed > 120) {
			set_throttle = 0.7f;
			set_brake = 0.7f;
		}
		if (sensing_info.speed > 130) {
			set_throttle = 0.5f;
			set_brake = 1.0f;
		}
	}

	// steering 까지 추가로 제어
	if (emergency_brake) {
		if (set_steering > 0) {
			set_steering += 0.3f;
		}
		else {
			set_steering -= 0.3f;
		}
	}

	///////////////////////////////////////
	//// 긴급 및 예외 상황 처리 종료 /////////
	///////////////////////////////////////

	// Moving straight forward
	car_controls.steering = set_steering;
	car_controls.throttle = set_throttle;
	car_controls.brake = set_brake;

	// ===========================================================
	// Editing area ends
	// ===========================================================

	return car_controls;
}

int main()
{
	// ===========================================================
	// Don't modify below area.
	// ===========================================================

	cout << "[MyCar] Start Bot! (CPP)" << endl;

	int return_code = StartDriving(control_driving, enable_api_control);

	cout << "[MyCar] End Bot! (CPP)" << endl;

	// ==========================================================

	return return_code;
}
