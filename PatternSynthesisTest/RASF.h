#pragma once
#include <vector>
#include <Box2D\Box2D.h>

constexpr auto DEGTORAD = 0.0174532925199432957f;
constexpr auto RADTODEG = 57.295779513082320876f;

#define RASF std::function<float32(std::vector<float32>, std::vector<float32>, float32, unsigned int)>

enum RASF_TYPE {
	RASF_CONSTANT,
	RASF_AVERAGE,
	RASF_BASIC_LERP,
};

inline float32 lerp(float32 a, float32 b, float32 t)
{
	return a + (t * (b - a));
}

static RASF getLerpRASF() {
	auto lerpRASF = [](std::vector<float32> startAngles, std::vector<float32> endAngles, float32 T, unsigned int numSegments) {
		float32 ret = 0.0f;

		// Lerp
		float32 start = 0.0f;
		for (float32 angle : startAngles) {
			if (abs(angle) > abs(start)) start = angle;
		}
		float32 end = 0.0f;
		for (float32 angle : endAngles) {
			if (abs(angle) > abs(end)) end = angle;
		}

		float32 invNumSegments = 1.0f / (float32)numSegments;

		ret = invNumSegments * lerp(-start, end, T);
		return ret;
	};

	return lerpRASF;
}

static RASF getAveAngleRASF() {
	auto aveAngleRASF = [](std::vector<float32> startAngles, std::vector<float32> endAngles, float32 T, unsigned int numSegments) {

		float32 invNumSegments = 1.0f / (float32)numSegments;

		float32 averageAngle = 0.0f;
		for (float32 angle : startAngles) averageAngle += -angle;
		for (float32 angle : endAngles) averageAngle += angle;
		averageAngle /= startAngles.size() + endAngles.size();
		return invNumSegments * averageAngle;

	};

	return aveAngleRASF;
}

static RASF getConstantRASF(float32 angle) {
	auto constantRASF = [angle](std::vector<float32> startAngles, std::vector<float32> endAngles, float32 T, unsigned int numSegments) {
		return angle * DEGTORAD;
	};

	return constantRASF;
}

// type: Type of rest angle function to get
// value: parameter for rest angle function (depends on type)
static RASF getRASF(RASF_TYPE type, float32 value = 1.0f) {
	switch (type) {
	case RASF_CONSTANT:
		return getConstantRASF(value);
	case RASF_AVERAGE:
		return getAveAngleRASF();
		break;
	case RASF_BASIC_LERP:
		return getLerpRASF();
		break;
	}
}
 