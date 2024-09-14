// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "../../thirdparty/glm/glm/ext/vector_float3.hpp"
#include "../../thirdparty/glm/glm/common.hpp"
#include "../../thirdparty/glm/glm/geometric.hpp"

namespace spp
{
struct AABB {
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
		return (v.x * v.y + v.x * v.z + v.y * v.z) * 2.f;
	}

	inline bool HasIntersection(const AABB &r) const
	{
		return glm::all(glm::lessThanEqual(min, r.max) &&
						glm::lessThanEqual(r.min, max));
	}
	inline AABB Intersection(const AABB &r) const
	{
		return {glm::max(min, r.min), glm::min(max, r.max)};
	}
	inline AABB Sum(const AABB &r) const
	{
		return {glm::min(min, r.min), glm::max(max, r.max)};
	}

	inline bool FastRayTest(glm::vec3 ro, glm::vec3 rd, const glm::vec3 invDir,
							float length, float &near, float &far) const
	{
		const glm::vec3 rad = max - min;
		const glm::vec3 center = min + (rad * 0.5f);
		ro -= center;

		const glm::vec3 m = invDir;
		glm::vec3 n = m * ro;
		glm::vec3 k = abs(m) * rad;
		glm::vec3 t1 = -n - k;
		glm::vec3 t2 = -n + k;

		float tN = near = glm::max(glm::max(t1.x, t1.y), t1.z);

		if (near * near > length) {
			return false;
		}

		float tF = far = glm::min(glm::min(t2.x, t2.y), t2.z);

		if (tN > tF || tF < 0.f) {
			return false;
		}

		return true;
	}

	inline bool SlowRayTest(const glm::vec3 &start, const glm::vec3 &end,
						float &near, float &far) const
	{
		const glm::vec3 dir = end - start;
		const float len = glm::length(dir);
		const glm::vec3 dirNorm = dir / len;
		const glm::vec3 invDir = glm::vec3(1.f, 1.f, 1.f) / dirNorm;
		return FastRayTest(start, dirNorm, invDir, len, near, far);
	}

public:
	inline bool operator&&(const AABB &r) const { return HasIntersection(r); }
	inline AABB operator*(const AABB &r) const { return Intersection(r); }
	inline AABB operator+(const AABB &r) const { return Sum(r); }
};
} // namespace spp
