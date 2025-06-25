// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/ChunkedBvhDbvt.hpp"

namespace spp
{
SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Chunk() {}

SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Chunk(Chunk &&c)
{
	offset = c.offset;
	scale = c.scale;
	invScale = c.invScale;
	localAabb = c.localAabb;
	globalAabb = c.globalAabb;
	bvh = c.bvh;
	c.bvh = nullptr;
}

SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk &
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::operator=(Chunk &&c)
{
	offset = c.offset;
	scale = c.scale;
	invScale = c.invScale;
	localAabb = c.localAabb;
	globalAabb = c.globalAabb;
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
	this->bp = bp;
	bvh = new BvhMedianSplitHeap<Aabb_i16, EntityType, MaskType, 0, 1,
								 int32_t>[2]{
		EntitiesOffsetsMapType_Reference(&(bp->entitiesOffsets),
										 chunkId & (-2)),
		EntitiesOffsetsMapType_Reference(&(bp->entitiesOffsets), chunkId | 1)};
	
	offset = (glm::ivec3)(aabb.GetCenter() / chunkSize);
	localAabb = {{-256*64, -256*64, -256*64}, {256*64, 256*64, 256*64}};
	globalAabb.max = globalAabb.min = offset * chunkSize;
	globalAabb.min -= chunkSize / 2.0f;
	globalAabb.max += chunkSize / 2.0f;
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
	aabb.min = ToLocalVec(aabb.min);
	aabb.max = ToLocalVec(aabb.max);
	assert(localAabb.ContainsAll(aabb));
	return aabb;
}

SPP_TEMPLATE_DECL_NO_AABB
Aabb ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::ToGlobalAabb(
	Aabb_i16 _aabb) const
{
	Aabb aabb = _aabb;
	aabb.min *= invScale;
	aabb.max *= invScale;
	aabb.min += offset;
	aabb.max += offset;
	return aabb;
}

SPP_TEMPLATE_DECL_NO_AABB
glm::vec3 ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::ToLocalVec(glm::vec3 p) const
{
	return (p - offset) * scale;
}

SPP_TEMPLATE_DECL_NO_AABB
int32_t ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::GetCount() const
{
	return bvh[0].GetCount() | bvh[1].GetCount();
}

SPP_TEMPLATE_DECL_NO_AABB
void
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::
IntersectAabb(AabbCallbacks::InterChunkCb *cb)
{
	cb->intraCb.chunk = this;
	cb->intraCb.aabb = ToLocalAabb(cb->aabb);
	
	for (int i=0; i<2; ++i) {
		bvh[i].IntersectAabb(cb->intraCb);
	}
}

SPP_TEMPLATE_DECL_NO_AABB
void
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::
IntersectRay(RayCallbacks::InterChunkCb *cb)
{
	cb->intraCb.chunk = this;
	
	cb->intraCb.start = ToLocalVec(cb->start);
	cb->intraCb.end = ToLocalVec(cb->end);
	
	for (int i=0; i<2; ++i) {
		bvh[i].IntersectRay(cb->intraCb);
	}
}


SPP_DEFINE_VARIANTS_NO_AABB(ChunkedBvhDbvt)
	
} // namespace spp
