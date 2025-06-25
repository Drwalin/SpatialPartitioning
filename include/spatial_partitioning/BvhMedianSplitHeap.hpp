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
template <typename EntityType, typename SegmentType, typename OffsetType,
		  bool enableDense>
struct DenseSparseSegmentOffsetMapReference {
public:
	struct SegmentOffset {
		SegmentType segment;
		OffsetType offset;

		bool operator==(SegmentOffset o) const
		{
			return o.offset == offset && o.segment == segment;
		}
		bool operator!=(SegmentOffset o) const
		{
			return o.offset != offset || o.segment != segment;
		}
	};
	
	using MapType = DenseSparseIntMap<EntityType, SegmentOffset, enableDense,
					  SegmentOffset{-1, -1}>;
	

public:
	DenseSparseSegmentOffsetMapReference(EntityType denseEntityRange)
	{
		owning = true;
		map = new MapType(denseEntityRange);
		segment = -1;
	}

	DenseSparseSegmentOffsetMapReference(
		MapType *map, SegmentType segment)
		: map(map), owning(false), segment(segment)
	{
	}

	DenseSparseSegmentOffsetMapReference(
		DenseSparseSegmentOffsetMapReference &&other)
		: map(other.map), owning(other.owning), segment(other.segment)
	{
		other.map = nullptr;
		other.owning = false;
		other.segment = -1;
	}

	~DenseSparseSegmentOffsetMapReference()
	{
		if (owning) {
			if (map) {
				delete map;
			}
			owning = false;
		}
		map = nullptr;
	}

	void Clear() { map->Clear(); }
	uint64_t GetMemoryUsage() const { return map->GetMemoryUsage(); }

	auto find(EntityType key) { return map->find(key); }
	auto find(EntityType key) const { return map->find(key); }

	auto Set(EntityType entity, OffsetType offset)
	{
		return map->Set(entity, {segment, offset});
	}

	OffsetType operator[](EntityType entity) const
	{
		return map->operator[](entity).offset;
	}

	void Remove(EntityType entity) { map->Set(entity, {-1, -1}); }

	bool Has(EntityType entity) const { return map->Has(entity); }

	void Reserve(EntityType entitiesCount) { map->Reserve(entitiesCount); }

	static OffsetType &get_offset_from_it(SegmentOffset *it)
	{
		return it->offset;
	}

	static OffsetType get_offset_from_it(const SegmentOffset *it)
	{
		return it->offset;
	}

public:
	MapType *map;
	bool owning;
	SegmentType segment;
};

template <typename EntityType, typename OffsetType, bool enableDense>
struct DenseSparseSegmentOffsetMapReference<EntityType, void, OffsetType,
											enableDense>
	: public DenseSparseIntMap<EntityType, OffsetType, enableDense, -1> {
public:
	
	const bool owning = true;
	
	using MapType = DenseSparseIntMap<EntityType, OffsetType, enableDense, -1>;
	
	static OffsetType &get_offset_from_it(OffsetType *it) { return *it; }

	static OffsetType get_offset_from_it(const OffsetType *it) { return *it; }
};

SPP_TEMPLATE_DECL_NO_AABB
class ChunkedBvhDbvt;

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
SPP_TEMPLATE_DECL_MORE(int SKIP_LOW_LAYERS = 0, typename SegmentType = void)
class BvhMedianSplitHeap final : public BroadphaseBase<SPP_TEMPLATE_ARGS>
{
public:
	using EntitiesOffsetsMapType =
		DenseSparseSegmentOffsetMapReference<EntityType, SegmentType, int32_t,
											 true>;

	using AabbCallback = spp::AabbCallback<SPP_TEMPLATE_ARGS>;
	using RayCallback = spp::RayCallback<SPP_TEMPLATE_ARGS>;
	using BroadphaseBaseIterator =
		spp::BroadphaseBaseIterator<SPP_TEMPLATE_ARGS>;

	BvhMedianSplitHeap(BvhMedianSplitHeap &&o) = default;
	BvhMedianSplitHeap &operator=(BvhMedianSplitHeap &&o) = default;
	BvhMedianSplitHeap(EntityType denseEntityRange);
	BvhMedianSplitHeap(EntitiesOffsetsMapType &&entityIdsMap);
	virtual ~BvhMedianSplitHeap();

	virtual const char *GetName() const override;

	virtual void Clear() override;
	void ClearWithoutOffsets();
	virtual size_t GetMemoryUsage() const override;
	virtual void ShrinkToFit() override;

	virtual void Add(EntityType entity, Aabb aabb, MaskType mask) override;
	virtual void Update(EntityType entity, Aabb aabb) override;
	virtual void Remove(EntityType entity) override;
	virtual void SetMask(EntityType entity, MaskType mask) override;
	
	EntityType GetEntityByOffset(int32_t offset) const;

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

	EntitiesOffsetsMapType entitiesOffsets;
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

SPP_EXTERN_VARIANTS_MORE(BvhMedianSplitHeap, 0, void)
SPP_EXTERN_VARIANTS_MORE(BvhMedianSplitHeap, 1, void)

SPP_EXTERN_VARIANTS_MORE(BvhMedianSplitHeap, 0, int32_t)
SPP_EXTERN_VARIANTS_MORE(BvhMedianSplitHeap, 1, int32_t)

} // namespace spp
