#pragma once
#include <vector>
#include <Box2D\Box2D.h>

float32 lerp(float32 a, float32 b, float32 t)
{
	return a + (t * (b - a));
}

static auto lerpRASF = [](std::vector<float32> startAngles, std::vector<float32> endAngles, float32 T, unsigned int numSegments) {
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

static auto aveAngleRASF = [](std::vector<float32> startAngles, std::vector<float32> endAngles, float32 T, unsigned int numSegments) {

	float32 invNumSegments = 1.0f / (float32)numSegments;

	float32 averageAngle = 0.0f;
	for (float32 angle : startAngles) averageAngle += -angle;
	for (float32 angle : endAngles) averageAngle += angle;
	averageAngle /= startAngles.size() + endAngles.size();
	return invNumSegments * averageAngle;

};