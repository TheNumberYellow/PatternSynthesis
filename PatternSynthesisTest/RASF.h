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
	RASF_PSEUDORANDOM,
	RASF_SEQUENTIAL_SIN,
	RASF_SEQUENTIAL_SIN_PLUS_LERP
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
		float32 invNumSegments = 1.0f / (float32)numSegments;
		float32 minimizer = 0.1f;

		return multiplier * invNumSegments * RandomFloat(-90.0f * DEGTORAD, 90.0f * DEGTORAD);
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
		if (numInLine % 4 <= 1) {
			ret = minimizer * RandomFloat(-90.0f * DEGTORAD, 0.0f * DEGTORAD);
		}
		else {
			ret = minimizer * RandomFloat(0.0f * DEGTORAD, 90.0f * DEGTORAD);
		}
		return ret;
	};

	return pseudoRandRASF;
}

static RASF getSequentialSinRASF(float32 multiplier) {
	auto sequentialSinRASF = [multiplier](std::vector<float32> startAngles, std::vector<float32> endAngles, float32 T, unsigned int numSegments) {
		// Set random seed equal to numSegments so that springs in the same spring line will have the same random results for continuity in a single spring line
		// ... Yeah this is bad and needs refactoring so that RASF can carry over information from one spring to the next in a a single spring line, but there's
		// no time right now.
		srand(numSegments);
		float32 ret = 0;

		std::vector<float32> sinBeginLocations;
		// First sin wave begin location always at start
		sinBeginLocations.push_back(0.0f);
		unsigned int currentBeginLocation = 0;

		while (true) {
			sinBeginLocations.push_back(RandomFloat(sinBeginLocations[currentBeginLocation], 1.0f));
			currentBeginLocation++;
			// If close to end, no more sin waves
			if (1.0f - sinBeginLocations[currentBeginLocation] < 0.15f) {
				break;
			}
		}
		unsigned int whichSegment = 0;
		// Check where current T is
		for (unsigned int i = 1; i < sinBeginLocations.size(); i++) {
			if (T < sinBeginLocations[i]) {
				whichSegment = i;
				break;
			}
			whichSegment = i;
		}
		// Check how far through current sin wave T is
		float32 sinSegmentLength = sinBeginLocations[whichSegment] - sinBeginLocations[whichSegment - 1];
		float32 howFarThrough = (T - sinBeginLocations[whichSegment - 1]) / sinSegmentLength;

		ret = (1.0f - sinSegmentLength) * sin(howFarThrough * (2 * b2_pi)) * multiplier;
		
		return ret;
	};

	return sequentialSinRASF;
}

static RASF RASFgetSequentialSinPlusLERPRASF(float32 multiplier) {
	auto sequentialSinPlusLERPRASF = [multiplier](std::vector<float32> startAngles, std::vector<float32> endAngles, float32 T, unsigned int numSegments) {
		// Set random seed equal to numSegments so that springs in the same spring line will have the same random results for continuity in a single spring line
		// ... Yeah this is bad and needs refactoring so that RASF can carry over information from one spring to the next in a a single spring line, but there's
		// no time right now.
		srand(numSegments);
		float32 ret = 0;

		std::vector<float32> sinBeginLocations;
		// First sin wave begin location always at start
		sinBeginLocations.push_back(0.0f);
		unsigned int currentBeginLocation = 0;

		while (true) {
			sinBeginLocations.push_back(RandomFloat(sinBeginLocations[currentBeginLocation], 1.0f));
			currentBeginLocation++;
			// If close to end, no more sin waves
			if (1.0f - sinBeginLocations[currentBeginLocation] < 0.15f) {
				break;
			}
		}
		unsigned int whichSegment = 0;
		// Check where current T is
		for (unsigned int i = 1; i < sinBeginLocations.size(); i++) {
			if (T < sinBeginLocations[i]) {
				whichSegment = i;
				break;
			}
			whichSegment = i;
		}
		// Check how far through current sin wave T is
		float32 sinSegmentLength = sinBeginLocations[whichSegment] - sinBeginLocations[whichSegment - 1];
		float32 howFarThrough = (T - sinBeginLocations[whichSegment - 1]) / sinSegmentLength;

		ret = (1.0f - sinSegmentLength) * sin(howFarThrough * (2 * b2_pi)) * multiplier * 3.0f;
		
		//// LERP segment
		//float32 start = 0.0f;
		//for (float32 angle : startAngles) {
		//	if (abs(angle) > abs(start)) start = angle;
		//}
		//float32 end = 0.0f;
		//for (float32 angle : endAngles) {
		//	if (abs(angle) > abs(end)) end = angle;
		//}

		//float32 invNumSegments = 1.0f / (float32)numSegments;
		//float32 minimizer = 0.4f;
		//ret += multiplier * minimizer * lerp(-start, end, T);
		float32 invNumSegments = 1.0f / (float32)numSegments;


		float32 averageAngle = 0.0f;
		for (float32 angle : startAngles) averageAngle += -angle;
		for (float32 angle : endAngles) averageAngle += angle;
		averageAngle /= startAngles.size() + endAngles.size();
		ret += 0.4f * multiplier * averageAngle;
		ret /= 2.0f;

		return ret;
	};

	return sequentialSinPlusLERPRASF;
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
	case RASF_SEQUENTIAL_SIN:
		return getSequentialSinRASF(value);
		break;
	case RASF_SEQUENTIAL_SIN_PLUS_LERP:
		return RASFgetSequentialSinPlusLERPRASF(value);
		break;
	default:
		return getConstantRASF(value);
		break;
	}
}
 