// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <cstdint>

#include <unordered_map>
#include <vector>

#include "BroadPhaseBase.hpp"

namespace spp
{
/*
 * Split policy:
 * 	Split between power-of-two median points
 * 	Sort against longest axis
 * Adding new object requires full rebuild
 * Removing object does not need to rebuild whole tree but shuld be needed
 * 
 * Limit is 268435456 (2^28-1)
 */
class BvhMedianSplitHeap final : public BroadphaseBase
{
public:
	BvhMedianSplitHeap();
	virtual ~BvhMedianSplitHeap();
	
	virtual void Clear() override;
	virtual size_t GetMemoryUsage() const override;
	virtual void ShrinkToFit() override;

	virtual void Add(EntityType entity, Aabb aabb,
					 MaskType mask) override;
	virtual void Update(EntityType entity, Aabb aabb) override;
	virtual void Remove(EntityType entity) override;
	virtual void SetMask(EntityType entity, MaskType mask) override;

	virtual void IntersectAabb(IntersectionCallback &callback) override;
	virtual void IntersectRay(RayCallback &callback) override;
	
	enum AabbUpdatePolicy : uint8_t
	{
		ON_UPDATE_EXTEND_AABB,
		ON_UPDATE_QUEUE_FULL_REBUILD_ON_NEXT_READ,
	};
	
	void SetAabbUpdatePolicy(AabbUpdatePolicy policy);
	AabbUpdatePolicy GetAabbUpdatePolicy() const;
	
	virtual void Rebuild() override;
	
private:
	void PruneEmptyEntitiesAtEnd();
	void UpdateAabb(int32_t entityOffset);
	void RebuildNode(int32_t nodeId);
	
	void _Internal_IntersectAabb(IntersectionCallback &cb, const int32_t nodeId);
	void _Internal_IntersectRay(RayCallback &cb, const int32_t nodeId);

private:
	struct alignas(64) Data {
		Aabb aabb;
		EntityType entity;
		MaskType mask;
	};
	
	struct alignas(32) NodeData {
		Aabb aabb;
		MaskType mask;
	};

	std::unordered_map<EntityType, int32_t> entitiesOffsets;
	// [0] - ignored, because heap works faster starting from 1
	std::vector<NodeData> nodesHeapAabb;
	std::vector<Data> entitiesData;
	
	int32_t entitiesCount = 0;
	int32_t entitiesPowerOfTwoCount = 0;
	bool rebuildTree = false;
	AabbUpdatePolicy updatePolicy = ON_UPDATE_EXTEND_AABB;
};
} // namespace spp
