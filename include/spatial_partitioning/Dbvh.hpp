// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <cstdint>

#include "AssociativeArray.hpp"
#include "BroadPhaseBase.hpp"

namespace spp
{
/*
 * Dynamic Bounding Volume Hierarchy
 * Split policy:
 *  Trying to do up to tripple middle of bounding AABB split at longest axis
 *    to find approximation of median split
 *
 * Limit of entities count is 268435456 (2^28-1)
 * 
 * Supports dynamically adding and removing entities
 */
class Dbvh final : public BroadphaseBase
{
public:
	Dbvh();
	virtual ~Dbvh();

	virtual const char *GetName() const override;

	virtual void Clear() override;
	virtual size_t GetMemoryUsage() const override;
	virtual void ShrinkToFit() override;

	virtual void Add(EntityType entity, Aabb aabb, MaskType mask) override;
	virtual void Update(EntityType entity, Aabb aabb) override;
	virtual void Remove(EntityType entity) override;
	virtual void SetMask(EntityType entity, MaskType mask) override;

	virtual Aabb GetAabb(EntityType entity) const override;
	virtual MaskType GetMask(EntityType entity) const override;

	virtual void IntersectAabb(IntersectionCallback &callback) override;
	virtual void IntersectRay(RayCallback &callback) override;

	virtual void Rebuild() override;

private:
	void UpdateAabb(int32_t entityOffset);
	void UpdateNodeAabb(int32_t nodeId);
	void RebuildNode(int32_t nodeId);

	void _Internal_IntersectAabb(IntersectionCallback &cb,
								 const int32_t nodeId);
	void _Internal_IntersectRay(RayCallback &cb, const int32_t nodeId);

private:
	inline const static int32_t OFFSET = 0x10000000;
	
	struct alignas(64) Data {
		AabbCentered aabb;
		EntityType entity = 0;
		MaskType mask = 0;
		int32_t parent = 0;
	};

	struct alignas(64) NodeData {
		AabbCentered aabb[2];
		MaskType mask = 0;
		int32_t parent = 0;
		// If (node > 0x10000000) than it is a leaf
		int32_t children[2] = {0,0};
	};
	
	AssociativeArray<EntityType, int32_t, Data> data;
	NodesArray<int32_t, NodeData> nodes;
	
	size_t modificationsSinceLastRebuild = 0;
	
	int32_t rootNode = 0;
};
} // namespace spp
