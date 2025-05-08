// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "../../glm/glm/ext/vector_float3.hpp"
#include "../../glm/glm/common.hpp"
#include "../../glm/glm/geometric.hpp"
#include "../../glm/glm/vector_relational.hpp"

namespace spp
{
inline const float EPSILON = 0.000001f;
inline const float BIG_EPSILON = 0.02f;

struct AabbCentered;

struct Aabb {
	glm::vec3 min;
	glm::vec3 max;

public:
	inline float GetVolume() const
	{
		const glm::vec3 v = max - min;
		return v.x * v.y * v.z;
	}
	inline float GetSurface() const
	{
		const glm::vec3 v = max - min;
		return (v.x * v.y + v.x * v.z + v.y * v.z) * 2.0f;
	}

	inline glm::vec3 GetCenter() const { return min + (max - min) * 0.5f; }
	inline glm::vec3 GetSizes() const { return max - min; }
	inline glm::vec3 GetMin() const { return min; }
	inline glm::vec3 GetMax() const { return max; }

	inline Aabb Expanded(float by) const { return {min - by, max + by}; }

	inline bool HasIntersection(const Aabb &r, float eps = 0.0f) const
	{
		return glm::all(glm::lessThanEqual(min - eps, r.max) &
						glm::lessThanEqual(r.min - eps, max));
	}

	inline bool IsIn(const glm::vec3 &r, float eps = 0.0f) const
	{
		return glm::all(glm::lessThanEqual(min - eps, r) &
						glm::lessThanEqual(r - eps, max));
	}
	inline Aabb Intersection(const Aabb &r) const
	{
		return {glm::max(min, r.min), glm::min(max, r.max)};
	}
	inline Aabb Sum(const Aabb &r) const
	{
		return {glm::min(min, r.min), glm::max(max, r.max)};
	}
	inline Aabb Sum(const glm::vec3 &r) const
	{
		return {glm::min(min, r), glm::max(max, r)};
	}

	inline bool ContainsAll(const Aabb &r, float eps = 0.0f) const
	{
		return glm::all(glm::lessThanEqual(min - eps, r.min) &
						glm::lessThanEqual(r.max, max + eps));
	}

	inline bool FastRayTestCenter(const glm::vec3 &ro, const glm::vec3 &rd,
								  const glm::vec3 &invDir, float length,
								  float &near, float &far) const;
	inline bool SlowRayTestCenter(const glm::vec3 &start, const glm::vec3 &end,
								  float &near, float &far) const;

	inline bool FastRayTest2(const glm::vec3 &ro, const glm::vec3 &invDir,
							 const int raySign[3], float &near,
							 float &far) const
	{
		assert(glm::all(glm::lessThanEqual(min, max)));
		alignas(16) glm::vec3 bounds[2] = {min, max}; //{min - ro, max - ro};
		alignas(16) glm::vec3 tmin, tmax;

		for (int i = 0; i < 2; ++i) {
			tmin[i] = (bounds[raySign[i]][i] - ro[i]) * invDir[i];
			tmax[i] = (bounds[1 - raySign[i]][i] - ro[i]) * invDir[i];
		}

		if ((tmin.x > tmax.y) || (tmin.y > tmax.x))
			return false;

		if (tmin.y > tmin.x)
			tmin.x = tmin.y;

		if (tmax.y < tmax.x)
			tmax.x = tmax.y;

		for (int i = 2; i < 3; ++i) {
			tmin[i] = (bounds[raySign[i]][i] - ro[i]) * invDir[i];
			tmax[i] = (bounds[1 - raySign[i]][i] - ro[i]) * invDir[i];
		}

		if ((tmin.x > tmax.z) || (tmin.z > tmax.x))
			return false;

		if (tmin.z > tmin.x)
			tmin.x = tmin.z;
		if (tmax.z < tmax.x)
			tmax.x = tmax.z;
		near = tmin.x;
		far = tmax.x;

		if (far < 0.0f)
			return false;

		if (near > far)
			return false;

		if (near < 0.0f) {
			near = 0.0f;
		}

		return true;
	}

	inline bool FastRayTest2(const glm::vec3 &ro, const glm::vec3 &invDir,
							 float &near, float &far) const
	{
		int signs[3] = {invDir[0] < 0.0, invDir[1] < 0.0, invDir[2] < 0.0};
		return FastRayTest2(ro, invDir, signs, near, far);
	}

	inline bool SlowRayTest2(const glm::vec3 &start, const glm::vec3 &end,
							 float &near, float &far) const
	{
		const glm::vec3 dir = end - start;
		const glm::vec3 invDir = glm::vec3(1.0f, 1.0f, 1.0f) / dir; // dirNorm;
		return FastRayTest2(start, invDir, near, far);
	}

	inline operator AabbCentered() const;

public:
	inline bool operator&&(const Aabb &r) const { return HasIntersection(r); }
	inline bool operator&&(const glm::vec3 &r) const { return IsIn(r); }
	inline Aabb operator*(const Aabb &r) const { return Intersection(r); }
	inline Aabb operator+(const Aabb &r) const { return Sum(r); }
	inline Aabb operator+(const glm::vec3 &r) const { return Sum(r); }
	inline bool operator==(const Aabb &r) const
	{
		return min == r.min && max == r.max;
	}
	inline bool operator!=(const Aabb &r) const
	{
		return min != r.min || max != r.max;
	}
};

struct AabbCentered {
	glm::vec3 center;
	glm::vec3 halfSize;

	inline operator Aabb() const { return Aabb{GetMin(), GetMax()}; }

public:
	inline float GetVolume() const
	{
		const glm::vec3 v = halfSize;
		return v.x * v.y * v.z * 2.0f;
	}
	inline float GetSurface() const
	{
		const glm::vec3 v = halfSize;
		return (v.x * v.y + v.x * v.z + v.y * v.z) * 2.0f * 4.0f;
	}

	inline AabbCentered Expanded(float by) const
	{
		return {center, halfSize + by};
	}

	inline glm::vec3 GetCenter() const { return center; }
	inline glm::vec3 GetSizes() const { return halfSize * 2.0f; }
	inline glm::vec3 GetMin() const { return center - halfSize; }
	inline glm::vec3 GetMax() const { return center + halfSize; }

	inline bool HasIntersection(const AabbCentered &r, float eps = 0.0f) const
	{
		return glm::all(glm::lessThanEqual(glm::abs(center - r.center),
										   halfSize + r.halfSize + eps));
	}

	inline bool IsIn(const glm::vec3 &r, float eps = 0.0f) const
	{
		return glm::all(
			glm::lessThanEqual(glm::abs(center - r), halfSize + eps));
	}
	inline Aabb Intersection(const AabbCentered &r) const
	{
		return {glm::max(GetMin(), r.GetMin()), glm::min(GetMax(), r.GetMax())};
	}
	inline Aabb Sum(const AabbCentered &r) const
	{
		return {glm::min(GetMin(), r.GetMin()), glm::max(GetMax(), r.GetMax())};
	}
	inline Aabb Sum(const glm::vec3 &r) const
	{
		return {glm::min(GetMin(), r), glm::max(GetMax(), r)};
	}

	inline bool ContainsAll(const AabbCentered &r, float eps = 0.0f) const
	{
		return glm::all(glm::lessThanEqual(
			glm::abs(center - r.center) + r.halfSize, halfSize + eps));
	}

	inline bool FastRayTestCenter(const glm::vec3 &_ro, const glm::vec3 &rd,
								  const glm::vec3 &invDir, float length,
								  float &near, float &far) const
	{
		const glm::vec3 rad = halfSize;
		const glm::vec3 ro = _ro - center;

		glm::vec3 n = invDir * ro;
		glm::vec3 k = abs(invDir) * rad;
		glm::vec3 t1 = -n - k;
		glm::vec3 t2 = -n + k;

		float tN = near = glm::max(glm::max(t1.x, t1.y), t1.z);

		if (near > 1.0f) {
			return false;
		}

		float tF = far = glm::min(glm::min(t2.x, t2.y), t2.z);

		if (tN > tF || tF < 0.0f) {
			return false;
		}

		if (near < 0.0f) {
			near = 0.0f;
		}

		return true;
	}

	inline bool SlowRayTestCenter(const glm::vec3 &start, const glm::vec3 &end,
								  float &near, float &far) const
	{
		const glm::vec3 dir = end - start;
		const float len = glm::length(dir);
		const glm::vec3 dirNorm = dir / len;
		const glm::vec3 invDir = glm::vec3(1.0f, 1.0f, 1.0f) / dir; // dirNorm;
		return FastRayTestCenter(start, dirNorm, invDir, len, near, far);
	}

public:
	inline bool operator&&(const AabbCentered &r) const
	{
		return HasIntersection(r);
	}
	inline Aabb operator*(const AabbCentered &r) const
	{
		return Intersection(r);
	}
	inline Aabb operator+(const AabbCentered &r) const { return Sum(r); }
	inline Aabb operator+(const glm::vec3 &r) const { return Sum(r); }
	inline bool operator==(const AabbCentered &r) const
	{
		return center == r.center && halfSize == r.halfSize;
	}
	inline bool operator!=(const AabbCentered &r) const
	{
		return center != r.center || halfSize != r.halfSize;
	}
};

inline Aabb::operator AabbCentered() const
{
	return {GetCenter(), GetSizes() * 0.5f};
}

inline bool Aabb::FastRayTestCenter(const glm::vec3 &ro, const glm::vec3 &rd,
									const glm::vec3 &invDir, float length,
									float &near, float &far) const
{
	return ((AabbCentered) * this)
		.FastRayTestCenter(ro, rd, invDir, length, near, far);
}

inline bool Aabb::SlowRayTestCenter(const glm::vec3 &start,
									const glm::vec3 &end, float &near,
									float &far) const
{
	return ((AabbCentered) * this).SlowRayTestCenter(start, end, near, far);
}
} // namespace spp
