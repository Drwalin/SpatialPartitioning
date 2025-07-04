// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <cstdint>

#include "../../glm/glm/ext/vector_uint3_sized.hpp"

#include "./BvhMedianSplitHeap.hpp"
#include "./BroadPhaseBase.hpp"

template <> struct std::hash<glm::u16vec3> {
	inline size_t operator()(const glm::i16vec3 &k) const
	{
		if constexpr (sizeof(size_t) == 8) {
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

private: // callbacks
	struct Chunk;

	class AabbCallbacks
	{
	public:
		class IntraChunkCb : public spp::AabbCallback<Aabb_i16, EntityType,
													  MaskType, EMPTY_ENTITY>
		{
		public:
			using BaseCb =
				spp::AabbCallback<Aabb_i16, EntityType, MaskType, EMPTY_ENTITY>;

			ChunkedBvhDbvt::AabbCallback *orgCb;
			ChunkedBvhDbvt *dbvt;
			Chunk *chunk;

			static void CallbackImpl(BaseCb *, EntityType entity);
		};

		class InterChunkCb
			: public spp::AabbCallback<Aabb, uint32_t, uint32_t, 0>
		{
		public:
			ChunkedBvhDbvt::AabbCallback *orgCb;
			ChunkedBvhDbvt *dbvt;
			AabbCallbacks::IntraChunkCb intraCb;

			void InitFrom(ChunkedBvhDbvt::AabbCallback &cb,
						  ChunkedBvhDbvt *dbvt);

			static void CallbackImpl(AabbCallback *, uint32_t chunkId);
		};
	};

	class RayCallbacks
	{
	public:
		class IntraChunkCb : public spp::RayCallback<Aabb_i16, EntityType,
													 MaskType, EMPTY_ENTITY>
		{
		public:
			using BaseCb =
				spp::RayCallback<Aabb_i16, EntityType, MaskType, EMPTY_ENTITY>;

			ChunkedBvhDbvt::RayCallback *orgCb;
			ChunkedBvhDbvt *dbvt;
			Chunk *chunk;

			static RayPartialResult CallbackImpl(BaseCb *, EntityType entity);
		};

		class InterChunkCb
			: public spp::RayCallback<Aabb, uint32_t, uint32_t, 0>
		{
		public:
			ChunkedBvhDbvt::RayCallback *orgCb;
			ChunkedBvhDbvt *dbvt;
			RayCallbacks::IntraChunkCb intraCb;

			void InitFrom(ChunkedBvhDbvt::RayCallback &cb,
						  ChunkedBvhDbvt *dbvt);

			static RayPartialResult CallbackImpl(RayCallback *,
												 uint32_t chunkId);
		};
	};

	friend class AabbCallbacks::InterChunkCb;
	friend class AabbCallbacks::IntraChunkCb;
	friend class RayCallbacks::InterChunkCb;
	friend class RayCallbacks::IntraChunkCb;

private:
	struct Chunk {
		Chunk();
		Chunk(Chunk &&chunk);
		Chunk &operator=(Chunk &&chunk);
		~Chunk();

		void Init(ChunkedBvhDbvt *bp, int32_t chunkId, float chunkSize,
				  Aabb aabb);

		ChunkedBvhDbvt *bp = nullptr;

		BvhMedianSplitHeap<Aabb_i16, EntityType, MaskType, 0, 1, int32_t> *bvh =
			nullptr;
		glm::i16vec3 chunkOffset;
		glm::vec3 centerGlobalOffset;
		glm::vec3 scale;
		glm::vec3 invScale;

		Aabb globalAabb;
		Aabb_i16 localAabb;

		int changes = 0;

		void Add(EntityType entity, Aabb aabb, MaskType mask);
		void Update(EntityType entity, Aabb aabb, int32_t oldSegment);
		void Remove(EntityType entity, int32_t segment);

		int32_t GetCount() const;

		void RebuildIfNeeded();
		void Rebuild();
		void ShrinkToFit();
		size_t GetMemoryUsage() const;
		Aabb GetAabb(EntityType entity, int32_t segment, int32_t offset) const;
		MaskType GetMask(EntityType entity, int32_t segment,
						 int32_t offset) const;

		Aabb_i16 ToLocalAabb(Aabb aabb) const;
		Aabb_i16 ToLocalAabbUnbound(Aabb aabb) const;
		Aabb ToGlobalAabb(Aabb_i16 aabb) const;

		glm::vec3 ToLocalVec(glm::vec3 p) const;

		void IntersectAabb(AabbCallbacks::InterChunkCb *cb);
		void IntersectRay(RayCallbacks::InterChunkCb *cb);
	};

	Chunk *GetChunkOfEntity(EntityType entity, int32_t &segment,
							int32_t &offset);
	const Chunk *GetChunkOfEntity(EntityType entity, int32_t &segment,
								  int32_t &offset) const;
	int32_t GetChunkIdOfEntity(EntityType entity, int32_t &offset);

	int32_t GetChunkIdFromAabb(Aabb aabb) const;

	Chunk *GetOrInitChunk(int32_t chunkId, Aabb aabb);

	Chunk *GetChunkById(uint32_t chunkId);

private:
	float chunkSize = 64.0f;
	float chunkSizeMultiplier = 256.0f;
	float chunkSizeFactor = 4.0f;
	float maxChunkedEntitySize = 32.0f;

	int limitChunkOffset = 512 - 2;

	uint32_t entitiesCount = 0;

	/*
	 * Segment:
	 *    lsb >> 1   -   chunk id (glm::i16vec3 serialized by 10 lower bits each
	 *                       component)
	 *    lsb & 1    -   which bvh inside chunk
	 * Segment = -1  -   outer objects
	 *
	 */
	MapType entitiesOffsets;

	HashMap<int32_t, Chunk> chunks;

	BvhMedianSplitHeap<Aabb, uint32_t, uint32_t, 0, 0> chunksBvh;
	BvhMedianSplitHeap<Aabb, EntityType, MaskType, 0, 0, int32_t> outerObjects;

	class Iterator final : public BroadphaseBaseIterator
	{
	public:
		Iterator(ChunkedBvhDbvt &bp);
		virtual ~Iterator();

		Iterator &operator=(Iterator &&other) = default;

		virtual bool Next() override;
		virtual bool Valid() override;
		bool FetchData();

		ChunkedBvhDbvt *bp;
		MapType::Iterator it;
		MapType::Iterator end;
	} iterator;
};

SPP_EXTERN_VARIANTS_NO_AABB(ChunkedBvhDbvt)

} // namespace spp
