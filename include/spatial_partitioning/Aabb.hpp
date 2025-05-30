// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <limits>

#include "../../glm/glm/ext/vector_float3.hpp"
#include "../../glm/glm/ext/vector_int3_sized.hpp"

namespace spp
{
inline const float EPSILON = 0.000001f;
inline const float BIG_EPSILON = 0.02f;

inline const glm::vec3 VEC_INF = {std::numeric_limits<float>::infinity(),
								  std::numeric_limits<float>::infinity(),
								  std::numeric_limits<float>::infinity()};

struct AabbCentered;
struct Aabb_i16;

struct Aabb {
	glm::vec3 min;
	glm::vec3 max;

public:
	bool IsValid() const;

	float GetVolume() const;
	float GetSurface() const;

	glm::vec3 GetCenter() const;
	glm::vec3 GetSizes() const;
	glm::vec3 GetMin() const;
	glm::vec3 GetMax() const;

	Aabb Expanded(float by) const;

	bool HasIntersection(const Aabb &r, float eps = 0.0f) const;

	bool IsIn(const glm::vec3 &r, float eps = 0.0f) const;
	Aabb Intersection(const Aabb &r) const;
	Aabb Sum(const Aabb &r) const;
	Aabb Sum(const glm::vec3 &r) const;

	bool ContainsAll(const Aabb &r, float eps = 0.0f) const;

	bool FastRayTestCenter(const glm::vec3 &ro, const glm::vec3 &rd,
						   const glm::vec3 &invDir, float length, float &near,
						   float &far) const;
	bool SlowRayTestCenter(const glm::vec3 &start, const glm::vec3 &end,
						   float &near, float &far) const;

	bool FastRayTest2(const glm::vec3 &ro, const glm::vec3 &invDir,
					  const int raySign[3], float &near, float &far) const;

	bool FastRayTest2(const glm::vec3 &ro, const glm::vec3 &invDir, float &near,
					  float &far) const;

	bool SlowRayTest2(const glm::vec3 &start, const glm::vec3 &end, float &near,
					  float &far) const;

	operator AabbCentered() const;
	operator Aabb_i16() const;

public:
	bool operator&&(const Aabb &r) const;
	bool operator&&(const glm::vec3 &r) const;
	Aabb operator*(const Aabb &r) const;
	Aabb operator+(const Aabb &r) const;
	Aabb operator+(const glm::vec3 &r) const;
	bool operator==(const Aabb &r) const;
	bool operator!=(const Aabb &r) const;
};

inline const Aabb AABB_INVALID = {VEC_INF, -VEC_INF};

struct AabbCentered {
	glm::vec3 center;
	glm::vec3 halfSize;

	operator Aabb() const;

public:
	float GetVolume() const;
	float GetSurface() const;

	AabbCentered Expanded(float by) const;

	glm::vec3 GetCenter() const;
	glm::vec3 GetSizes() const;
	glm::vec3 GetMin() const;
	glm::vec3 GetMax() const;

	bool HasIntersection(const AabbCentered &r, float eps = 0.0f) const;

	bool IsIn(const glm::vec3 &r, float eps = 0.0f) const;
	Aabb Intersection(const AabbCentered &r) const;
	Aabb Sum(const AabbCentered &r) const;
	Aabb Sum(const glm::vec3 &r) const;

	bool ContainsAll(const AabbCentered &r, float eps = 0.0f) const;

	bool FastRayTestCenter(const glm::vec3 &_ro, const glm::vec3 &rd,
						   const glm::vec3 &invDir, float length, float &near,
						   float &far) const;

	bool SlowRayTestCenter(const glm::vec3 &start, const glm::vec3 &end,
						   float &near, float &far) const;

	operator Aabb_i16() const;

public:
	bool operator&&(const AabbCentered &r) const;
	Aabb operator*(const AabbCentered &r) const;
	Aabb operator+(const AabbCentered &r) const;
	Aabb operator+(const glm::vec3 &r) const;
	bool operator==(const AabbCentered &r) const;
	bool operator!=(const AabbCentered &r) const;
};

struct Aabb_i16 {
	glm::i16vec3 min;
	glm::i16vec3 max;

public:
	bool IsValid() const;

	float GetVolume() const;
	float GetSurface() const;

	glm::i16vec3 GetCenter() const;
	glm::i16vec3 GetSizes() const;
	glm::i16vec3 GetMin() const;
	glm::i16vec3 GetMax() const;

	Aabb_i16 Expanded(float _by) const;

	bool HasIntersection(const Aabb_i16 &r) const;

	bool IsIn(const glm::i16vec3 &r) const;
	Aabb_i16 Intersection(const Aabb_i16 &r) const;
	Aabb_i16 Sum(const Aabb_i16 &r) const;
	Aabb_i16 Sum(const glm::i16vec3 &r) const;

	bool ContainsAll(const Aabb_i16 &r) const;

	bool FastRayTest2(const glm::vec3 &ro, const glm::vec3 &invDir,
					  const int raySign[3], float &near, float &far) const;

	bool FastRayTest2(const glm::vec3 &ro, const glm::vec3 &invDir, float &near,
					  float &far) const;

	bool SlowRayTest2(const glm::vec3 &start, const glm::vec3 &end, float &near,
					  float &far) const;

	operator AabbCentered() const;

	operator Aabb() const;

public:
	bool operator&&(const Aabb_i16 &r) const;
	bool operator&&(const glm::i16vec3 &r) const;
	Aabb_i16 operator*(const Aabb_i16 &r) const;
	Aabb_i16 operator+(const Aabb_i16 &r) const;
	Aabb_i16 operator+(const glm::i16vec3 &r) const;
	bool operator==(const Aabb_i16 &r) const;
	bool operator!=(const Aabb_i16 &r) const;
};
} // namespace spp
