#pragma once
#include <vector>
#include <Box2D\Box2D.h>

#include "util.h"

constexpr auto DEGTORAD = 0.0174532925199432957f;
constexpr auto RADTODEG = 57.295779513082320876f;

#define RASF std::function<float32(std::vector<float32>, std::vector<float32>, float32, unsigned int)>

enum RASF_TYPE {
	RASF_CONSTANT,
	RASF_AVERAGE,
	RASF_BASIC_LERP,
	RASF_RANDOMIZED,
	RASF_SINWAVE,
	RASF_PSEUDORANDOM
};

inline float32 lerp(float32 a, float32 b, float32 t)
{
	return a + (t * (b - a));
}

static RASF getLerpRASF(float32 multiplier) {
	auto lerpRASF = [multiplier](std::vector<float32> startAngles, std::vector<float32> endAngles, float32 T, unsigned int numSegments) {
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
		float32 minimizer = 0.1f;
		ret = multiplier * invNumSegments * lerp(-start, end, T);
		//ret = multiplier * minimizer * lerp(-start, end, T);
		return ret;
	};

	return lerpRASF;
}

static RASF getAveAngleRASF(float32 multiplier) {
	auto aveAngleRASF = [multiplier](std::vector<float32> startAngles, std::vector<float32> endAngles, float32 T, unsigned int numSegments) {

		float32 invNumSegments = 1.0f / (float32)numSegments;

		float32 averageAngle = 0.0f;
		for (float32 angle : startAngles) averageAngle += -angle;
		for (float32 angle : endAngles) averageAngle += angle;
		averageAngle /= startAngles.size() + endAngles.size();
		return multiplier * invNumSegments * averageAngle;

	};

	return aveAngleRASF;
}

static RASF getConstantRASF(float32 angle) {
	auto constantRASF = [angle](std::vector<float32> startAngles, std::vector<float32> endAngles, float32 T, unsigned int numSegments) {
		return angle * DEGTORAD;
	};

	return constantRASF;
}

static RASF getRandomizedRASF(float32 multiplier) {
	auto randomizedRASF = [multiplier](std::vector<float32> startAngles, std::vector<float32> endAngles, float32 T, unsigned int numSegments) {
		//float32 invNumSegments = 1.0f / (float32)numSegments;
		float32 minimizer = 0.1f;

		return multiplier * minimizer * RandomFloat(-90.0f * DEGTORAD, 90.0f * DEGTORAD);
	};

	return randomizedRASF;
}

static RASF getSINWaveRASF(float32 multiplier) {
	auto SINWaveRASF = [multiplier](std::vector<float32> startAngles, std::vector<float32> endAngles, float32 T, unsigned int numSegments) {
		float32 ret = sin(T * (2 * b2_pi)) * multiplier;
		return ret;
	};

	return SINWaveRASF;
}

static RASF getPseudorandomRASF(float32 multiplier) {
	auto pseudoRandRASF = [multiplier](std::vector<float32> startAngles, std::vector<float32> endAngles, float32 T, unsigned int numSegments) {
		float32 ret = 0;
		unsigned int numInLine = T * numSegments;
		float32 minimizer = 0.4;

		// Alternate positive and negative random float
		if (numInLine % 2 == 0) {
			ret = minimizer * RandomFloat(-90.0f * DEGTORAD, -30.0f * DEGTORAD);
		}
		else {
			ret = minimizer * RandomFloat(30.0f * DEGTORAD, 90.0f * DEGTORAD);
		}
		return ret;
	};

	return pseudoRandRASF;
}

// type: Type of rest angle function to get
// value: parameter for rest angle function (depends on type)
static RASF getRASF(RASF_TYPE type, float32 value = 1.0f) {
	switch (type) {
	case RASF_CONSTANT:
		return getConstantRASF(value);
	case RASF_AVERAGE:
		return getAveAngleRASF(value);
		break;
	case RASF_BASIC_LERP:
		return getLerpRASF(value);
		break;
	case RASF_RANDOMIZED:
		return getRandomizedRASF(value);
		break;
	case RASF_SINWAVE:
		return getSINWaveRASF(value);
		break;
	case RASF_PSEUDORANDOM:
		return getPseudorandomRASF(value);
		break;
	default:
		return getConstantRASF(value);
		break;
	}
}
 