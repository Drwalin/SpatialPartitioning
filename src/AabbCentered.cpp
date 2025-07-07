// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../glm/glm/common.hpp"
#include "../glm/glm/geometric.hpp"
#include "../glm/glm/vector_relational.hpp"

#include "../include/spatial_partitioning/Aabb.hpp"

namespace spp
{
AabbCentered::operator Aabb() const { return Aabb{GetMin(), GetMax()}; }

float AabbCentered::GetVolume() const
{
	const glm::vec3 v = halfSize;
	return v.x * v.y * v.z * 2.0f;
}
float AabbCentered::GetSurface() const
{
	const glm::vec3 v = halfSize;
	return (v.x * v.y + v.x * v.z + v.y * v.z) * 2.0f * 4.0f;
}

AabbCentered AabbCentered::Expanded(float by) const
{
	return {center, halfSize + by};
}

glm::vec3 AabbCentered::GetCenter() const { return center; }
glm::vec3 AabbCentered::GetSizes() const { return halfSize * 2.0f; }
glm::vec3 AabbCentered::GetMin() const { return center - halfSize; }
glm::vec3 AabbCentered::GetMax() const { return center + halfSize; }

bool AabbCentered::HasIntersection(const AabbCentered &r, float eps) const
{
	return glm::all(glm::lessThanEqual(glm::abs(center - r.center),
									   halfSize + r.halfSize + eps));
}

bool AabbCentered::IsIn(const glm::vec3 &r, float eps) const
{
	return glm::all(glm::lessThanEqual(glm::abs(center - r), halfSize + eps));
}
Aabb AabbCentered::Intersection(const AabbCentered &r) const
{
	return {glm::max(GetMin(), r.GetMin()), glm::min(GetMax(), r.GetMax())};
}
Aabb AabbCentered::Sum(const AabbCentered &r) const
{
	return {glm::min(GetMin(), r.GetMin()), glm::max(GetMax(), r.GetMax())};
}
Aabb AabbCentered::Sum(const glm::vec3 &r) const
{
	return {glm::min(GetMin(), r), glm::max(GetMax(), r)};
}

bool AabbCentered::ContainsAll(const AabbCentered &r, float eps) const
{
	return glm::all(glm::lessThanEqual(glm::abs(center - r.center) + r.halfSize,
									   halfSize + eps));
}

bool AabbCentered::FastRayTestCenter(const glm::vec3 &_ro, const glm::vec3 &rd,
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

bool AabbCentered::SlowRayTestCenter(const glm::vec3 &start,
									 const glm::vec3 &end, float &near,
									 float &far) const
{
	const glm::vec3 dir = end - start;
	const float len = glm::length(dir);
	const glm::vec3 dirNorm = dir / len;
	const glm::vec3 invDir = glm::vec3(1.0f, 1.0f, 1.0f) / dir; // dirNorm;
	return FastRayTestCenter(start, dirNorm, invDir, len, near, far);
}

bool AabbCentered::operator&&(const AabbCentered &r) const
{
	return HasIntersection(r);
}
Aabb AabbCentered::operator*(const AabbCentered &r) const
{
	return Intersection(r);
}
Aabb AabbCentered::operator+(const AabbCentered &r) const { return Sum(r); }
Aabb AabbCentered::operator+(const glm::vec3 &r) const { return Sum(r); }
bool AabbCentered::operator==(const AabbCentered &r) const
{
	return center == r.center && halfSize == r.halfSize;
}
bool AabbCentered::operator!=(const AabbCentered &r) const
{
	return center != r.center || halfSize != r.halfSize;
}

AabbCentered::operator Aabb_i16() const { return Aabb(*this); }
AabbCentered::operator Aabb_i32() const { return Aabb(*this); }
} // namespace spp
