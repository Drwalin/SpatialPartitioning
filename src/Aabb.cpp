// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../glm/glm/common.hpp"
#include "../glm/glm/vector_relational.hpp"

#include "../include/spatial_partitioning/Aabb.hpp"

namespace spp
{
bool Aabb::IsValid() const { return min.x > max.x; }

float Aabb::GetVolume() const
{
	const glm::vec3 v = max - min;
	return v.x * v.y * v.z;
}
float Aabb::GetSurface() const
{
	const glm::vec3 v = max - min;
	return (v.x * v.y + v.x * v.z + v.y * v.z) * 2.0f;
}

glm::vec3 Aabb::GetCenter() const { return min + (max - min) * 0.5f; }
glm::vec3 Aabb::GetSizes() const { return max - min; }
glm::vec3 Aabb::GetMin() const { return min; }
glm::vec3 Aabb::GetMax() const { return max; }

Aabb Aabb::Expanded(float by) const { return {min - by, max + by}; }

bool Aabb::HasIntersection(const Aabb &r, float eps) const
{
	return glm::all(glm::lessThanEqual(min - eps, r.max) &
					glm::lessThanEqual(r.min - eps, max));
}

bool Aabb::IsIn(const glm::vec3 &r, float eps) const
{
	return glm::all(glm::lessThanEqual(min - eps, r) &
					glm::lessThanEqual(r - eps, max));
}
Aabb Aabb::Intersection(const Aabb &r) const
{
	return {glm::max(min, r.min), glm::min(max, r.max)};
}
Aabb Aabb::Sum(const Aabb &r) const
{
	return {glm::min(min, r.min), glm::max(max, r.max)};
}
Aabb Aabb::Sum(const glm::vec3 &r) const
{
	return {glm::min(min, r), glm::max(max, r)};
}

bool Aabb::ContainsAll(const Aabb &r, float eps) const
{
	return glm::all(glm::lessThanEqual(min - eps, r.min) &
					glm::lessThanEqual(r.max - eps, max));
}

bool Aabb::FastRayTest2(const glm::vec3 &ro, const glm::vec3 &invDir,
						const int raySign[3], float &near, float &far) const
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

bool Aabb::FastRayTest2(const glm::vec3 &ro, const glm::vec3 &invDir,
						float &near, float &far) const
{
	int signs[3] = {invDir[0] < 0.0, invDir[1] < 0.0, invDir[2] < 0.0};
	return FastRayTest2(ro, invDir, signs, near, far);
}

bool Aabb::SlowRayTest2(const glm::vec3 &start, const glm::vec3 &end,
						float &near, float &far) const
{
	const glm::vec3 dir = end - start;
	const glm::vec3 invDir = glm::vec3(1.0f, 1.0f, 1.0f) / dir; // dirNorm;
	return FastRayTest2(start, invDir, near, far);
}

bool Aabb::operator&&(const Aabb &r) const { return HasIntersection(r); }
bool Aabb::operator&&(const glm::vec3 &r) const { return IsIn(r); }
Aabb Aabb::operator*(const Aabb &r) const { return Intersection(r); }
Aabb Aabb::operator+(const Aabb &r) const { return Sum(r); }
Aabb Aabb::operator+(const glm::vec3 &r) const { return Sum(r); }
bool Aabb::operator==(const Aabb &r) const
{
	return min == r.min && max == r.max;
}
bool Aabb::operator!=(const Aabb &r) const
{
	return min != r.min || max != r.max;
}


Aabb::operator AabbCentered() const { return {GetCenter(), GetSizes() * 0.5f}; }

bool Aabb::FastRayTestCenter(const glm::vec3 &ro, const glm::vec3 &rd,
							 const glm::vec3 &invDir, float length, float &near,
							 float &far) const
{
	return ((AabbCentered) * this)
		.FastRayTestCenter(ro, rd, invDir, length, near, far);
}

bool Aabb::SlowRayTestCenter(const glm::vec3 &start, const glm::vec3 &end,
							 float &near, float &far) const
{
	return ((AabbCentered) * this).SlowRayTestCenter(start, end, near, far);
}

Aabb::operator Aabb_i16() const
{
	constexpr glm::ivec3 MIN(std::numeric_limits<short>::min());
	constexpr glm::ivec3 MAX(std::numeric_limits<short>::max());
	glm::ivec3 a = min, b = max;
	a = glm::min(glm::max(a, MIN), MAX);
	b = glm::max(glm::min(b, MAX), MIN);
	return {a, b};
}

Aabb::operator Aabb_i32() const
{
	constexpr glm::vec3 MIN((float)std::numeric_limits<int32_t>::min());
	constexpr glm::vec3 MAX((float)std::numeric_limits<int32_t>::max());
	glm::vec3 a = min, b = max;
	a = glm::min(glm::max(a, MIN), MAX);
	b = glm::max(glm::min(b, MAX), MIN);
	return {a, b};
}
} // namespace spp
