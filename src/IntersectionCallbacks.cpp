// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/IntersectionCallbacks.hpp"

namespace spp
{
void RayCallback::InitVariables()
{
	if (initedVars == false) {
		initedVars = true;
		dir = end - start;
		cutFactor = 1.0f;
		length = glm::length(dir);
		dirNormalized = glm::normalize(dir);
		invDir = glm::vec3(1.f, 1.f, 1.f) / dir; // dirNormalized;
		signs[0] = invDir[0] < 0.0 ? 1 : 0;
		signs[1] = invDir[1] < 0.0 ? 1 : 0;
		signs[2] = invDir[2] < 0.0 ? 1 : 0;
	}
}

bool IntersectionCallback::IsRelevant(AabbCentered aabb) const
{
	return aabb && this->aabb;
}

bool IntersectionCallback::IsRelevant(Aabb aabb) const
{
	return aabb && this->aabb;
}

bool RayCallback::IsRelevant(AabbCentered aabb, float &near, float &far) const
{
	if (aabb.FastRayTest(start, dirNormalized, invDir, length, near, far)) {
		if (near > cutFactor) {
			return false;
		} else {
			return true;
		}
	}
	return false;
}
bool RayCallback::IsRelevant(Aabb aabb, float &near, float &far) const
{
	if (aabb.FastRayTest2(start, invDir, signs, near, far)) {
		if (near > cutFactor) {
			return false;
		} else {
			return true;
		}
	}
	return false;
}

bool RayCallback::IsRelevant(AabbCentered aabb) const
{
	float n, f;
	return IsRelevant(aabb, n, f);
}

bool RayCallback::IsRelevant(Aabb aabb) const
{
	float n, f;
	return IsRelevant(aabb, n, f);
}

RayPartialResult RayCallback::ExecuteCallback(EntityType entity)
{
	auto res = callback(this, entity);
	++testedCount;
	if (res.intersection) {
		++hitCount;
		cutFactor = res.dist;
		return res;
	}
	return {1, false};
}

RayPartialResult RayCallback::ExecuteIfRelevant(AabbCentered aabb,
												EntityType entity, float &near,
												float &far)
{
	++nodesTestedCount;
	if (IsRelevant(aabb, near, far)) {
		float n = near;
		if (n < cutFactor) {
			return ExecuteCallback(entity);
		}
	}
	return {1, false};
}

RayPartialResult RayCallback::ExecuteIfRelevant(Aabb aabb, EntityType entity,
												float &near, float &far)
{
	++nodesTestedCount;
	if (IsRelevant(aabb, near, far)) {
		float n = near;
		if (n < cutFactor) {
			return ExecuteCallback(entity);
		}
	}
	return {1, false};
}

RayPartialResult RayCallback::ExecuteIfRelevant(AabbCentered aabb,
												EntityType entity)
{
	float n, f;
	return ExecuteIfRelevant(aabb, entity, n, f);
}

RayPartialResult RayCallback::ExecuteIfRelevant(Aabb aabb, EntityType entity)
{
	float n, f;
	return ExecuteIfRelevant(aabb, entity, n, f);
}
} // namespace spp
