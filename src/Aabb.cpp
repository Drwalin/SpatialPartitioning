// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../glm/glm/common.hpp"
#include "../glm/glm/geometric.hpp"
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
					glm::lessThanEqual(r.max, max + eps));
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

bool AabbCentered::HasIntersection(const AabbCentered &r,
								   float eps) const
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

Aabb::operator Aabb_i16() const
{
	glm::ivec3 a = min, b = max;
	a = glm::max(a, glm::ivec3(std::numeric_limits<short>::min()));
	b = glm::min(b, glm::ivec3(std::numeric_limits<short>::max()));
	return {a, b};
}

AabbCentered::operator Aabb_i16() const { return Aabb(*this); }

} // namespace spp
