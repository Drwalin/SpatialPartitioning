// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/IntersectionCallbacks.hpp"

namespace spp
{
SPP_TEMPLATE_DECL
void RayCallback<SPP_TEMPLATE_ARGS>::InitVariables()
{
	if (initedVars == false) {
		Calc(start, end);
		initedVars = true;
		cutFactor = 1.0f;
	}
}

SPP_TEMPLATE_DECL
bool AabbCallback<SPP_TEMPLATE_ARGS>::IsRelevant(AabbCentered aabb) const
{
	return this->aabb && aabb;
}

SPP_TEMPLATE_DECL
bool AabbCallback<SPP_TEMPLATE_ARGS>::IsRelevant(Aabb aabb) const
{
	return this->aabb && aabb;
}

SPP_TEMPLATE_DECL
void AabbCallback<SPP_TEMPLATE_ARGS>::ExecuteCallback(EntityType entity)
{
	++testedCount;
	callback(this, entity);
}
	
SPP_TEMPLATE_DECL
bool AabbCallback<SPP_TEMPLATE_ARGS>::ExecuteIfRelevant(AabbCentered aabb, EntityType entity)
{
	++nodesTestedCount;
	if (IsRelevant(aabb)) {
		ExecuteCallback(entity);
		return true;
	}
	return false;
}

SPP_TEMPLATE_DECL
bool AabbCallback<SPP_TEMPLATE_ARGS>::ExecuteIfRelevant(Aabb aabb, EntityType entity)
{
	++nodesTestedCount;
	if (IsRelevant(aabb)) {
		ExecuteCallback(entity);
		return true;
	}
	return false;
}

SPP_TEMPLATE_DECL
bool RayCallback<SPP_TEMPLATE_ARGS>::IsRelevant(AabbCentered aabb, float &near, float &far) const
{
	if (aabb.FastRayTestCenter(start, dirNormalized, invDir, length, near, far)) {
		if (near > cutFactor) {
			return false;
		} else {
			return true;
		}
	}
	return false;
}
SPP_TEMPLATE_DECL
bool RayCallback<SPP_TEMPLATE_ARGS>::IsRelevant(Aabb aabb, float &near, float &far) const
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

SPP_TEMPLATE_DECL
bool RayCallback<SPP_TEMPLATE_ARGS>::IsRelevant(AabbCentered aabb) const
{
	float n, f;
	return IsRelevant(aabb, n, f);
}

SPP_TEMPLATE_DECL
bool RayCallback<SPP_TEMPLATE_ARGS>::IsRelevant(Aabb aabb) const
{
	float n, f;
	return IsRelevant(aabb, n, f);
}

SPP_TEMPLATE_DECL
RayPartialResult RayCallback<SPP_TEMPLATE_ARGS>::ExecuteCallback(EntityType entity)
{
	++testedCount;
	auto res = callback(this, entity);
	if (res.intersection) {
		++hitCount;
		cutFactor = res.dist;
		return res;
	}
	return {1, false};
}

SPP_TEMPLATE_DECL
RayPartialResult RayCallback<SPP_TEMPLATE_ARGS>::ExecuteIfRelevant(AabbCentered aabb,
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

SPP_TEMPLATE_DECL
RayPartialResult RayCallback<SPP_TEMPLATE_ARGS>::ExecuteIfRelevant(Aabb aabb, EntityType entity,
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

SPP_TEMPLATE_DECL
RayPartialResult RayCallback<SPP_TEMPLATE_ARGS>::ExecuteIfRelevant(AabbCentered aabb,
												EntityType entity)
{
	float n, f;
	return ExecuteIfRelevant(aabb, entity, n, f);
}

SPP_TEMPLATE_DECL
RayPartialResult RayCallback<SPP_TEMPLATE_ARGS>::ExecuteIfRelevant(Aabb aabb, EntityType entity)
{
	float n, f;
	return ExecuteIfRelevant(aabb, entity, n, f);
}

SPP_DEFINE_VARIANTS(AabbCallback)
SPP_DEFINE_VARIANTS(RayCallback)
SPP_DEFINE_VARIANTS(RayCallbackFirstHit)
	
} // namespace spp
