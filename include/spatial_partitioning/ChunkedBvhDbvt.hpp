// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <cstdint>

#include "../../glm/glm/ext/vector_uint3_sized.hpp"

#include "./BvhMedianSplitHeap.hpp"
#include "BroadPhaseBase.hpp"

template <> struct std::hash<glm::u16vec3> {
	inline size_t operator()(const glm::i16vec3 &k) const
	{
		if constexpr (sizeof(size_t) > 4) {
			union {
				glm::u16vec3 u;
				size_t h;
			};
			h = 0;
			u = k;
			return h;
		} else {
			assert(!"sizeof(size_t) needs to be 8 bytes");
		}
	}
};

template <> struct std::hash<glm::i16vec3> {
	inline size_t operator()(const glm::i16vec3 &k) const
	{
		return std::hash<glm::u16vec3>()(k);
	}
};

namespace spp
{
SPP_TEMPLATE_DECL_NO_AABB
class ChunkedBvhDbvt final : public BroadphaseBase<SPP_TEMPLATE_ARGS>

{
public:
	using AabbCallback = spp::AabbCallback<SPP_TEMPLATE_ARGS>;
	using RayCallback = spp::RayCallback<SPP_TEMPLATE_ARGS>;
	using BroadphaseBaseIterator =
		spp::BroadphaseBaseIterator<SPP_TEMPLATE_ARGS>;

	using EntitiesOffsetsMapType_Reference =
		DenseSparseSegmentOffsetMapReference<EntityType, int32_t, int32_t,
											 true>;
	using MapType = EntitiesOffsetsMapType_Reference::MapType;

	ChunkedBvhDbvt(EntityType denseEntityRange);
	virtual ~ChunkedBvhDbvt();

	virtual const char *GetName() const override;

	virtual void Clear() override;
	virtual size_t GetMemoryUsage() const override;
	virtual void ShrinkToFit() override;
	void ShrinkToFitIncremental();

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

	virtual void Rebuild() override;

	virtual BroadphaseBaseIterator *RestartIterator() override;

public:
	void RebuildIncremental();

private:
	struct ChunkAabbId {
		glm::ivec3 chunkOffsets;
		glm::vec3 size;
	};

	ChunkAabbId GetChunkIdFromAabb(Aabb aabb) const;

private:
	struct Chunk {
		BvhMedianSplitHeap<Aabb_i16, EntityType, MaskType, 0, 1, int32_t>
			bvh[2];
		glm::vec3 offset;
		glm::vec3 scale;
		glm::vec3 invScale;
		Aabb aabb;

		void Rebuild();
		void ShrinkToFit();
		size_t GetMemoryUsage();
	};

private:
	float chunkSize = 64.0f;
	float chunkSizeMultiplier = 256.0f;
	float chunkSizeFactor = 4.0f;
	float maxChunkedEntitySize = 32.0f;

	uint32_t entitiesCount = 0;

	HashMap<glm::i16vec3, Chunk> chunks;

	BvhMedianSplitHeap<Aabb, int32_t, int32_t, 0, 0> chunksBvh;
	BvhMedianSplitHeap<Aabb, int32_t, int32_t, 0, 0> outerObjects;

	MapType entitiesOffsets;

	class Iterator final : public BroadphaseBaseIterator
	{
	public:
		Iterator(ChunkedBvhDbvt &bp);
		virtual ~Iterator();

		Iterator &operator=(Iterator &&other) = default;

		virtual bool Next() override;
		virtual bool Valid() override;
		bool FetchData();

		MapType::Iterator it;
		MapType::Iterator end;
	} iterator;
};

SPP_EXTERN_VARIANTS_NO_AABB(ChunkedBvhDbvt)

} // namespace spp
