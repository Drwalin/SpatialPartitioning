// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../glm/glm/common.hpp"

#include "../include/spatial_partitioning/ChunkedBvhDbvt.hpp"

namespace spp
{
SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Chunk() {}

/*
SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Chunk(Chunk &&c)
{
	scale = c.scale;
	invScale = c.invScale;
	localAabb = c.localAabb;
	localAabbInner = c.localAabbInner;
	globalAabb = c.globalAabb;
	globalAabbInner = c.globalAabbInner;
	new (&bvh) InternalBvhHeap(std::move(c.bvh));
	bp = c.bp;
	c.bp = nullptr;
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
	if (bp) {
		bvh.~InternalBvhHeap();
		bvh = std::move(c.bvh);
	} else {
		new (&bvh) InternalBvhHeap(std::move(c.bvh));
	}
	bp = c.bp;
	c.bp = nullptr;
	return *this;
}
*/

SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::~Chunk()
{
	if (bp) {
		bvh.~InternalBvhHeap();
		bp = nullptr;
	}
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Init(ChunkedBvhDbvt *bp,
															int32_t chunkId,
															float chunkSize,
															Aabb aabb)
{
	if (this->bp) {
		bvh.~InternalBvhHeap();
	}
	this->chunkId = chunkId;
	this->bp = bp;
	new (&bvh) InternalBvhHeap{
		EntitiesOffsetsMapType_Reference(&(bp->entitiesOffsets), chunkId)};

	glm::ivec3 chunkOffset = glm::floor(aabb.GetCenter() / chunkSize);
	glm::vec3 minGlobalOffset = ((glm::vec3)chunkOffset) * chunkSize;
	localAabbInner = {-glm::ivec3(chunkSize * bp->chunkSizeMultiplier),
					  glm::ivec3(chunkSize * bp->chunkSizeMultiplier)};
	localAabb = {localAabbInner.min * 2, localAabbInner.max * 2};
	globalAabbInner.min = minGlobalOffset;
	globalAabbInner.max = minGlobalOffset + chunkSize;
	globalAabb.min = minGlobalOffset - chunkSize * 0.5f;
	globalAabb.max = minGlobalOffset + chunkSize * 1.5f;
	invScale = glm::vec3(bp->chunkSizeMultiplier);
	scale = glm::vec3(1.0f) / invScale;
	globalCenter = globalAabb.GetCenter();
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Add(EntityType entity,
														   Aabb aabb,
														   MaskType mask)
{
	Aabb_i16 b2 = ToLocalAabb(aabb);
	bvh.Add(entity, b2, mask);
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Update(EntityType entity,
															  Aabb _aabb)
{
	Aabb_i16 aabb = ToLocalAabb(_aabb);
	bvh.Update(entity, aabb);
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Remove(EntityType entity)
{
	changes++;
	bvh.Remove(entity);
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::RebuildIfNeeded()
{
	if (bvh.GetCount() > 100 || changes > 50) {
		Rebuild();
	}
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Rebuild()
{
	bvh.Rebuild();
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::ShrinkToFit()
{
	bvh.ShrinkToFit();
}

SPP_TEMPLATE_DECL_NO_AABB
size_t ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::GetMemoryUsage() const
{
	assert(bp != nullptr);
	return bvh.GetMemoryUsage();
}

SPP_TEMPLATE_DECL_NO_AABB
Aabb ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::GetAabb(
	EntityType entity, int32_t offset) const
{
	return ToGlobalAabb(bvh.GetAabb(entity));
}

SPP_TEMPLATE_DECL_NO_AABB
MaskType
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::GetMask(EntityType entity,
														  int32_t offset) const
{
	return bvh.GetMask(entity);
}

SPP_TEMPLATE_DECL_NO_AABB
Aabb_i16 ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::ToLocalAabb(
	const Aabb _aabb) const
{
	const Aabb org = _aabb;
	Aabb aabb = ToLocalAabbUnbound(org);
	/*
	Aabb glob = ToGlobalAabb(aabb);
	assert(glob.IsIn(org.min));
	assert(glob.IsIn(org.max));
	assert(glob.ContainsAll(org));
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
	return Aabb{glm::floor(ToLocalVec(aabb.min)) - 1.0f,
				 glm::ceil(ToLocalVec(aabb.max)) + 1.0f};
}

SPP_TEMPLATE_DECL_NO_AABB
Aabb ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::ToGlobalAabb(
	Aabb_i16 _aabb) const
{
	Aabb aabb = _aabb;
	aabb.min = glm::fma(aabb.min, scale, globalCenter);
	aabb.max = glm::fma(aabb.max, scale, globalCenter);
	return aabb;
}

SPP_TEMPLATE_DECL_NO_AABB
glm::vec3
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::ToLocalVec(glm::vec3 p) const
{
	return (p - globalCenter) * invScale;
}

SPP_TEMPLATE_DECL_NO_AABB
int32_t ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::GetCount() const
{
	return bvh.GetCount();
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::IntersectAabb(
	AabbCallbacks::InterChunkCb *cb)
{
	cb->intraCb.chunk = this;
	cb->intraCb.aabb = ToLocalAabbUnbound(cb->aabb);

	bvh.IntersectAabb(cb->intraCb);
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::IntersectRay(
	RayCallbacks::InterChunkCb *cb)
{
	auto &intraCb = cb->intraCb;
	intraCb.chunk = this;

	intraCb.start = ToLocalVec(cb->start);
	intraCb.end = ToLocalVec(cb->end);

	bvh.IntersectRay(intraCb);
}

// SPP_DEFINE_VARIANTS_NO_AABB(ChunkedBvhDbvt)

} // namespace spp
