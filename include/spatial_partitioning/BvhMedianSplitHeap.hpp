// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
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
 * Limit of entities count is 268435456 (2^28-1)
 *
 * Tree is perfectly balanced due to heap use as nodes storage
 */
class BvhMedianSplitHeap final : public BroadphaseBase
{
public:
	BvhMedianSplitHeap();
	virtual ~BvhMedianSplitHeap();

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

	enum AabbUpdatePolicy : uint8_t {
		ON_UPDATE_EXTEND_AABB,
		ON_UPDATE_QUEUE_FULL_REBUILD_ON_NEXT_READ,
	};

	void SetAabbUpdatePolicy(AabbUpdatePolicy policy);
	AabbUpdatePolicy GetAabbUpdatePolicy() const;

	virtual void Rebuild() override;
	
public:
	struct RebuildProgress {
		int32_t stack[64];
		int32_t size = 0;
		int32_t stage = 0;
		int32_t it = 0;
		bool done = false;
	};
	bool RebuildStep(RebuildProgress &progress);

private:
	void PruneEmptyEntitiesAtEnd();
	void UpdateAabb(int32_t entityOffset);
	void RebuildNode(int32_t nodeId);
	int32_t RebuildNodePartial(int32_t nodeId, int32_t *tcount);

	void _Internal_IntersectAabb(IntersectionCallback &cb,
								 const int32_t nodeId);
	void _Internal_IntersectRay(RayCallback &cb, const int32_t nodeId);

private:
	struct alignas(32) Data {
		AabbCentered aabb;
		EntityType entity;
		MaskType mask;
	};

	struct alignas(32) NodeData {
		AabbCentered aabb;
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
