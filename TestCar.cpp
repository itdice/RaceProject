#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include "RuleBasedDriving.h"

using namespace std;
using namespace Car;

constexpr bool debugMode = true;  // 디버깅용 로그 데이터 출력 여부
constexpr bool autopilotMode = true;  // 자율주행 코드 적용 여부

// 조정 가능한 매개변수
struct ControlParameters {
	bool is_start = false;  // 출발 여부 감지

	float wheelbase = 2.5f;  // 차량 휠베이스 (m)

	float max_speed = 200.0f;  // 차량 설계 최고 속도 (km/h)
	float base_target_speed = 100.0f;  // 기본 목표 속도 (km/h)
	float speed_accept = 0.1f;  // 차량 속도 인정 범위 (%)

	float min_steering = 0.1f;  // 최소 스티어링 휠 각도 - 최대 속도 시 (%)
	float max_steering = 0.8f;  // 최대 스티어링 휠 각도 - 최소 속도 시 (%)
	int steering_slope = 1;  // 속도별 최대 스티어링 휠 상수
	float distance_steering_ratio = 0.3f;  // 거리기준 스티어링 영향도
	float angle_steering_ratio = 0.7f;  // 각도기준 스티어링 영향도

	float past_steering_value = 0.0f;  // 이전 스티어링 휠 각도 (%)
	float max_difference_steering = 0.03f;  // 최대 스티어링 휠 각도 변화 (%);
};

ControlParameters params;

ControlValues driveControl(CarStateValues sensing_info)
{
	ControlValues car_controls;

	// ┏━━━━━━━━━━━━━━━━┓
	// ┃ Autopilot Mode ┃
	// ┗━━━━━━━━━━━━━━━━┛

	// 차량 속도 관련
	float speed = sensing_info.speed;  // 현재 차량 속도 (km/h)
	float target_speed = params.base_target_speed;  // 차량 목표 속도 (km/h)

	// 차량 방향 및 속도 제어 관련
	float steering = 0.0f;  // 차량 스티어링 휠 제어 (%)
	float throttle = 1.0f;  // 차량 엑셀 제어 (%)
	float brake = 0.0f;  // 차량 브레이크 제어 (%)

	// 도로 및 차량, 장애물 규격 관련
	float road_half_width = sensing_info.half_road_limit - (params.wheelbase / 2);  // 도로폭 절반 (m)
	float safe_distance = road_half_width - (params.wheelbase / 2);  // 바퀴가 다 들어가는 도로 중심부터의 거리 (m)

	// 주행 정보 관련
	bool is_collided = sensing_info.collided;  // 차량 충돌 여부
	bool is_forward = sensing_info.moving_forward;  // 정주행 여부
	float distance_from_center = sensing_info.to_middle;  // 도로 중심으로부터 거리 (m)
	float angle_from_center = sensing_info.moving_angle;  // 도로 중심과의 각도 (°)
	vector<float> forward_distance_from_center = sensing_info.distance_to_way_points;  // 전방의 도로 중심지점과의 직선 거리 (m)
	vector<float> forward_angle_from_center = sensing_info.track_forward_angles;  // 전방 도로 중심지점과의 각도 (°)

	// 출발 전 제어
	if (!params.is_start && speed == 0.0f) {
		car_controls.steering = steering;
		car_controls.throttle = throttle;
		car_controls.brake = brake;

		return car_controls;
	}
	else if (!params.is_start && speed > 0.0f) {
		cout << "[WLT Race Project] Race Start!!!" << "\n";
		params.is_start = true;
	}

	// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

	// 차량 방향 제어
	float max_steering = static_cast<float>(params.min_steering + (params.max_steering - params.min_steering) 
		* exp(-params.steering_slope * (speed / params.max_speed)));  // 현재 속도와 연동된 최대 스티어링 휠 각도

	float distance_difference_ratio = (distance_from_center < 0 ? -1 : 1) *
		min(1.0f, abs(distance_from_center / safe_distance)) * params.distance_steering_ratio;  // 거리기준 스티어링 제어 비율
	float angle_difference_ratio = (angle_from_center < 0 ? -1 : 1) *
		min(1.0f, abs(angle_from_center / 30.0f)) * params.angle_steering_ratio;  // 각도기준 스티어링 제어 비율


	float total_ratio = (angle_difference_ratio + distance_difference_ratio < 0 ? -1 : 1) * 
		min(1.0f, abs(angle_difference_ratio + distance_difference_ratio));  // 최종 스티어링 제어 비율
	float required_steering = -1 * max_steering * total_ratio;  // 위의 결과를 토대로 스티어링 각도 계산

	if (abs(required_steering - params.past_steering_value) > params.max_difference_steering) {
		steering = params.past_steering_value + params.max_difference_steering *
			(required_steering - params.past_steering_value < 0 ? -1 : 1);  // 스티어링이 급격하게 변화하는 경우 점진적으로 변화하도록 함
	}
	else {
		steering = required_steering;
	}
	params.past_steering_value = steering;


	// 차량 속도 제어
	float required_speed = target_speed - speed;  // 목표 속도까지 필요한 속도

	if (target_speed * params.speed_accept < required_speed) {
		throttle = 1.0f;  // 목표속도까지 최대한 가속
	}
	else if (0 <= required_speed && required_speed <= target_speed * params.speed_accept) {
		throttle = static_cast<float>(pow(target_speed / params.max_speed, 0.4));  // 목표속도에 거의 다다르면 적절하게 가속
	}
	else if (-1 * target_speed * params.speed_accept <= required_speed && required_speed < 0) {
		throttle = static_cast<float>(pow(target_speed / params.max_speed, 0.5));  // 목표속도를 약간 초과하면 이전보다 덜 가속
	}
	else {
		throttle = 0.0f;  // 목표속도를 완전히 초과하면 가속 중지
	}

	// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

	// 최종 제어값 반환
	car_controls.steering = steering;
	car_controls.throttle = throttle;
	car_controls.brake = brake;

	if (debugMode) {
		printf("=========================================================================\n");
		printf("[Debug - SPD] %07.3fkm/h -> %07.3fkm/h, THR : %.3f%%, BRK : %.3f%%\n", speed, target_speed, throttle * 100, brake * 100);
		printf("[Debug - LOC] DFC : %.3fm, AFC : %.3f°\n", distance_from_center, angle_from_center);

		printf("[Debug - FOC] Forward DFC : ");
		for (const auto value : forward_distance_from_center)
			printf("%.3fm ", value);
		printf("\n");

		printf("[Debug - FOC] Forward AFC : ");
		for (const auto value : forward_angle_from_center)
			printf("%.3f° ", value);
		printf("\n");

		printf("[Debug - STR] STR : %.3f%%\n", steering * 100);

		if (is_collided || !is_forward) {
			printf("[Debug - EXT] ");
			if (is_collided) { printf("Car Collided "); }
			if (!is_forward) { printf("Car Backward "); }
			printf("\n");
		}
	}

	return car_controls;
}

int main()
{
	// =========================
	// Don't modify below area.
	// =========================

	cout << "[WLT Race Project] Program Start" << "\n";

	int return_code = StartDriving(driveControl, autopilotMode);

	cout << "[WLT Race Project] Program End" << "\n";

	return return_code;
}
