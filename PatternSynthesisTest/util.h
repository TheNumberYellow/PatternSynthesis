#pragma once
#include <Box2D\Box2D.h>

inline float32 RandomFloat(float32 a, float32 b) {
	float32 random = ((float32)rand()) / (float32)RAND_MAX;
	float32 diff = b - a;
	float32 r = random * diff;
	return a + r;
}

// Clamps angle between -pi and pi
inline float32 clampAngle(float32 angle)
{
	//return fmod((angle + b2_pi), (2.0f * b2_pi)) - b2_pi;
	if (angle > b2_pi) {
		return angle - (2.0f * b2_pi);
	}
	else if (angle < -b2_pi) {
		return angle + (2.0f * b2_pi);
	}
	return angle;
}
