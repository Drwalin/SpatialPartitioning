// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../glm/glm/common.hpp"
#include "../glm/glm/vector_relational.hpp"

#include "../include/spatial_partitioning/Aabb.hpp"

namespace spp
{
bool Aabb_i16::IsValid() const { return min.x > max.x; }

float Aabb_i16::GetVolume() const
{
	const glm::i16vec3 v = max - min;
	return v.x * v.y * v.z;
}
float Aabb_i16::GetSurface() const
{
	const glm::i16vec3 v = max - min;
	return (v.x * v.y + v.x * v.z + v.y * v.z) * 2.0f;
}

glm::i16vec3 Aabb_i16::GetCenter() const
{
	return (max + min) / glm::i16vec3(2);
}
glm::i16vec3 Aabb_i16::GetSizes() const { return max - min; }
glm::i16vec3 Aabb_i16::GetMin() const { return min; }
glm::i16vec3 Aabb_i16::GetMax() const { return max; }

Aabb_i16 Aabb_i16::Expanded(float _by) const
{
	glm::ivec3 by(_by);
	glm::ivec3 a = min, b = max;
	a -= by;
	b += by;
	a = glm::max(a, glm::ivec3(std::numeric_limits<short>::min()));
	b = glm::min(b, glm::ivec3(std::numeric_limits<short>::max()));
	return {a, b};
}

bool Aabb_i16::HasIntersection(const Aabb_i16 &r) const
{
	return glm::all(glm::lessThanEqual(min, r.max) &
					glm::lessThanEqual(r.min, max));
}

bool Aabb_i16::IsIn(const glm::i16vec3 &r) const
{
	return glm::all(glm::lessThanEqual(min, r) & glm::lessThanEqual(r, max));
}
Aabb_i16 Aabb_i16::Intersection(const Aabb_i16 &r) const
{
	return {glm::max(min, r.min), glm::min(max, r.max)};
}
Aabb_i16 Aabb_i16::Sum(const Aabb_i16 &r) const
{
	return {glm::min(min, r.min), glm::max(max, r.max)};
}
Aabb_i16 Aabb_i16::Sum(const glm::i16vec3 &r) const
{
	return {glm::min(min, r), glm::max(max, r)};
}

bool Aabb_i16::ContainsAll(const Aabb_i16 &r) const
{
	return glm::all(glm::lessThanEqual(min, r.min) &
					glm::lessThanEqual(r.max, max));
}

bool Aabb_i16::FastRayTest2(const glm::vec3 &ro, const glm::vec3 &invDir,
							const int raySign[3], float &near, float &far) const
{
	return Aabb(min, max).FastRayTest2(ro, invDir, raySign, near, far);
}

bool Aabb_i16::FastRayTest2(const glm::vec3 &ro, const glm::vec3 &invDir,
							float &near, float &far) const
{
	return Aabb(min, max).FastRayTest2(ro, invDir, near, far);
}

bool Aabb_i16::SlowRayTest2(const glm::vec3 &start, const glm::vec3 &end,
							float &near, float &far) const
{
	return Aabb(min, max).SlowRayTest2(start, end, near, far);
}

Aabb_i16::operator AabbCentered() const
{
	return {GetCenter(), GetSizes() / glm::i16vec3(2)};
}

Aabb_i16::operator Aabb() const { return {min, max}; }
Aabb_i16::operator Aabb_i32() const { return {min, max}; }

bool Aabb_i16::operator&&(const Aabb_i16 &r) const
{
	return HasIntersection(r);
}
bool Aabb_i16::operator&&(const glm::i16vec3 &r) const { return IsIn(r); }
Aabb_i16 Aabb_i16::operator*(const Aabb_i16 &r) const
{
	return Intersection(r);
}
Aabb_i16 Aabb_i16::operator+(const Aabb_i16 &r) const { return Sum(r); }
Aabb_i16 Aabb_i16::operator+(const glm::i16vec3 &r) const { return Sum(r); }
bool Aabb_i16::operator==(const Aabb_i16 &r) const
{
	return min == r.min && max == r.max;
}
bool Aabb_i16::operator!=(const Aabb_i16 &r) const
{
	return min != r.min || max != r.max;
}
} // namespace spp
