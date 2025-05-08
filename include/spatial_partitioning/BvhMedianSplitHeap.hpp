// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <cstdint>

#include <vector>

#include "DenseSparseIntMap.hpp"
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
SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS = 0)
class BvhMedianSplitHeap final : public BroadphaseBase<SPP_TEMPLATE_ARGS>
{
public:
	using AabbCallback = spp::AabbCallback<SPP_TEMPLATE_ARGS>;
	using RayCallback = spp::RayCallback<SPP_TEMPLATE_ARGS>;
	using BroadphaseBaseIterator =
		spp::BroadphaseBaseIterator<SPP_TEMPLATE_ARGS>;

	BvhMedianSplitHeap(EntityType denseEntityRange);
	virtual ~BvhMedianSplitHeap();

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

	void _Internal_IntersectAabb(AabbCallback &cb, const int32_t nodeId);
	void _Internal_IntersectRay(RayCallback &cb, const int32_t nodeId);

private:
	struct Data {
		Aabb aabb;
		EntityType entity;
		MaskType mask;
	};

	struct NodeData {
		Aabb aabb;
		MaskType mask;
	};

	DenseSparseIntMap<EntityType, int32_t, true, -1> entitiesOffsets;
	// [0] - ignored, because heap works faster starting from 1
	std::vector<NodeData> nodesHeapAabb;
	std::vector<Data> entitiesData;

	int32_t entitiesCount = 0;
	int32_t entitiesPowerOfTwoCount = 0;
	bool rebuildTree = false;
	AabbUpdatePolicy updatePolicy = ON_UPDATE_EXTEND_AABB;

	class Iterator final : public BroadphaseBaseIterator
	{
	public:
		Iterator(BvhMedianSplitHeap &bp);
		virtual ~Iterator();

		Iterator &operator=(Iterator &&other) = default;

		virtual bool Next() override;
		virtual bool Valid() override;
		bool FetchData();

		std::vector<Data> *data;
		int it;
	} iterator;
};

SPP_EXTERN_VARIANTS_MORE(BvhMedianSplitHeap, 0)
SPP_EXTERN_VARIANTS_MORE(BvhMedianSplitHeap, 1)

} // namespace spp
