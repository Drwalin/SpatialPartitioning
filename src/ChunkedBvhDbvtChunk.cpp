// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../glm/glm/common.hpp"

#include "../include/spatial_partitioning/ChunkedBvhDbvt.hpp"

namespace spp
{
SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Chunk() {}

SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Chunk(Chunk &&c)
{
	scale = c.scale;
	invScale = c.invScale;
	localAabb = c.localAabb;
	localAabbInner = c.localAabbInner;
	globalAabb = c.globalAabb;
	globalAabbInner = c.globalAabbInner;
	bvh = c.bvh;
	c.bvh = nullptr;
}

SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk &
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::operator=(Chunk &&c)
{
	scale = c.scale;
	invScale = c.invScale;
	localAabb = c.localAabb;
	localAabbInner = c.localAabbInner;
	globalAabb = c.globalAabb;
	globalAabbInner = c.globalAabbInner;
	bvh = c.bvh;
	c.bvh = nullptr;
	return *this;
}

SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::~Chunk()
{
	if (bvh) {
		delete[] bvh;
		bvh = nullptr;
	}
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Init(ChunkedBvhDbvt *bp,
															int32_t chunkId,
															float chunkSize,
															Aabb aabb)
{
	this->chunkId = chunkId;
	this->bp = bp;
	bvh = new BvhMedianSplitHeap<Aabb_i16, EntityType, MaskType, 0, 1,
								 int32_t>[2]{
		EntitiesOffsetsMapType_Reference(&(bp->entitiesOffsets),
										 chunkId & (-2)),
		EntitiesOffsetsMapType_Reference(&(bp->entitiesOffsets), chunkId | 1)};

	glm::ivec3 chunkOffset = glm::floor(aabb.GetCenter() / chunkSize);
	glm::vec3 minGlobalOffset = ((glm::vec3)chunkOffset) * chunkSize;
	localAabbInner = {-glm::ivec3(chunkSize * bp->chunkSizeMultiplier),
					  glm::ivec3(chunkSize * bp->chunkSizeMultiplier)};
	localAabb = {localAabbInner.min * 2, localAabbInner.max * 2};
	globalAabbInner.min = minGlobalOffset;
	globalAabbInner.max = minGlobalOffset + chunkSize;
	globalAabb.min = minGlobalOffset - chunkSize * 0.5f;
	globalAabb.max = minGlobalOffset + chunkSize * 1.5f;
	invScale = glm::vec3(256.0f);
	scale = glm::vec3(1.0f) / invScale;
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Add(EntityType entity,
														   Aabb aabb,
														   MaskType mask)
{
	Aabb_i16 b2 = ToLocalAabb(aabb);
	bvh[1].Add(entity, b2, mask);
	RebuildIfNeeded();
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Update(
	EntityType entity, Aabb _aabb, int32_t oldSegment)
{
	Aabb_i16 aabb = ToLocalAabb(_aabb);
	int bid = oldSegment & 1;
	if (bid == 0) {
		changes++;
		MaskType mask = bvh[0].GetMask(entity);
		bvh[0].Remove(entity);
		bvh[1].Add(entity, aabb, mask);
	} else {
		bvh[1].Update(entity, aabb);
	}
	RebuildIfNeeded();
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Remove(EntityType entity,
															  int32_t segment)
{
	int bid = segment & 1;
	if (bid == 0) {
		changes++;
	}
	bvh[bid].Remove(entity);
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::RebuildIfNeeded()
{
	if (bvh[1].GetCount() > 100 || changes > 50) {
		Rebuild();
	}
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Rebuild()
{
	bvh[0].StartFastAdding();
	for (auto it = bvh[1].RestartIterator(); it->Valid(); it->Next()) {
		bp->entitiesOffsets.Set(it->entity, {-1, -1});
		bvh[0].Add(it->entity, it->aabb, it->mask);
	}
	bvh[1].ClearWithoutOffsets();
	bvh[0].StopFastAdding();
	bvh[0].Rebuild();
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::ShrinkToFit()
{
	bvh[0].ShrinkToFit();
	bvh[1].ShrinkToFit();
}

SPP_TEMPLATE_DECL_NO_AABB
size_t ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::GetMemoryUsage() const
{
	assert(bvh != nullptr);
	return bvh[0].GetMemoryUsage() + bvh[1].GetMemoryUsage();
}

SPP_TEMPLATE_DECL_NO_AABB
Aabb ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::GetAabb(
	EntityType entity, int32_t segment, int32_t offset) const
{
	return ToGlobalAabb(bvh[segment & 1].GetAabb(entity));
}

SPP_TEMPLATE_DECL_NO_AABB
MaskType ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::GetMask(
	EntityType entity, int32_t segment, int32_t offset) const
{
	return bvh[segment & 1].GetMask(entity);
}

SPP_TEMPLATE_DECL_NO_AABB
Aabb_i16
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::ToLocalAabb(Aabb aabb) const
{
	Aabb org = aabb;
	aabb.min = glm::floor(ToLocalVec(aabb.min));
	aabb.max = glm::ceil(ToLocalVec(aabb.max));
	Aabb glob = ToGlobalAabb(aabb);
	assert(glob.IsIn(org.min));
	assert(glob.IsIn(org.max));
	assert(glob.ContainsAll(org));
	/*
	assert(localAabb.ContainsAll(aabb));
	assert(localAabbInner.IsIn(aabb.GetCenter()));
	assert(globalAabbInner.IsIn(glob.GetCenter()));
	assert(globalAabbInner.IsIn(org.GetCenter()));
	*/
	return aabb;
}

SPP_TEMPLATE_DECL_NO_AABB
Aabb_i16 ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::ToLocalAabbUnbound(
	Aabb aabb) const
{
	aabb.min = glm::floor(ToLocalVec(aabb.min));
	aabb.max = glm::ceil(ToLocalVec(aabb.max));
	return aabb;
}

SPP_TEMPLATE_DECL_NO_AABB
Aabb ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::ToGlobalAabb(
	Aabb_i16 _aabb) const
{
	Aabb aabb = _aabb;
	aabb.min = glm::fma(aabb.min, scale, globalAabbInner.min);
	aabb.max = glm::fma(aabb.max, scale, globalAabbInner.min);
	/*
	aabb.min *= scale;
	aabb.max *= scale;
	aabb.min += globalAabbInner.min;
	aabb.max += globalAabbInner.min;
	*/
	return aabb;
}

SPP_TEMPLATE_DECL_NO_AABB
glm::vec3
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::ToLocalVec(glm::vec3 p) const
{
	return (p - globalAabbInner.min) * invScale;
}

SPP_TEMPLATE_DECL_NO_AABB
int32_t ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::GetCount() const
{
	return bvh[0].GetCount() | bvh[1].GetCount();
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::IntersectAabb(
	AabbCallbacks::InterChunkCb *cb)
{
	cb->intraCb.chunk = this;
	cb->intraCb.aabb = ToLocalAabbUnbound(cb->aabb);

	for (int i = 0; i < 2; ++i) {
		bvh[i].IntersectAabb(cb->intraCb);
	}
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::IntersectRay(
	RayCallbacks::InterChunkCb *cb)
{
	cb->intraCb.chunk = this;

	cb->intraCb.start = ToLocalVec(cb->start);
	cb->intraCb.end = ToLocalVec(cb->end);

	for (int i = 0; i < 2; ++i) {
		bvh[i].IntersectRay(cb->intraCb);
	}
}

SPP_DEFINE_VARIANTS_NO_AABB(ChunkedBvhDbvt)

} // namespace spp
