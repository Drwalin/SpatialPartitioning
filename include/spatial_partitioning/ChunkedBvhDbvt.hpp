// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <cstdint>

#include <vector>

#include "DenseSparseIntMap.hpp"
#include "./BvhMedianSplitHeap.hpp"
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
SPP_TEMPLATE_DECL_NO_AABB
class ChunkedBvhDbvt final : public BroadphaseBase<SPP_TEMPLATE_ARGS>

{
public:
	using AabbCallback = spp::AabbCallback<SPP_TEMPLATE_ARGS>;
	using RayCallback = spp::RayCallback<SPP_TEMPLATE_ARGS>;
	using BroadphaseBaseIterator =
		spp::BroadphaseBaseIterator<SPP_TEMPLATE_ARGS>;

	ChunkedBvhDbvt(EntityType denseEntityRange);
	virtual ~ChunkedBvhDbvt();

	virtual const char *GetName() const override;

	virtual void Clear() override;
	virtual size_t GetMemoryUsage() const override;
	virtual void ShrinkToFit() override;

	virtual void Add(EntityType entity, Aabb aabb, MaskType mask) override;
	virtual void Update(EntityType entity, Aabb aabb) override;
	virtual void Remove(EntityType entity) override;
	virtual void SetMask(EntityType entity, MaskType mask) override;

	virtual int32_t GetCount() const override;
	virtual bool Exists(EntityType entity) const override;

	virtual Aabb GetAabb(EntityType entity) const override;
	virtual MaskType GetMask(EntityType entity) const override;

	virtual void IntersectAabb(AabbCallback &callback) override;
	virtual void IntersectRay(RayCallback &callback) override;

	enum AabbUpdatePolicy : uint8_t {
		ON_UPDATE_EXTEND_AABB,
		ON_UPDATE_QUEUE_FULL_REBUILD_ON_NEXT_READ,
	};

	void SetAabbUpdatePolicy(AabbUpdatePolicy policy);
	AabbUpdatePolicy GetAabbUpdatePolicy() const;

	virtual void Rebuild() override;

	virtual BroadphaseBaseIterator *RestartIterator() override;

public:
	void RebuildIncremental();

private:
	
	struct Chunk {
		BvhMedianSplitHeap<Aabb_i16, EntityType, MaskType, 0, 1> bvh;
	};
	
private:
	
	std::vector<Chunk> chunks;
	
	BvhMedianSplitHeap<Aabb, int32_t, int8_t, 0, 0> chunksBvh;
};
} // namespace spp
