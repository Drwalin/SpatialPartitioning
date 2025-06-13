// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/ChunkedBvhDbvt.hpp"

namespace spp
{
SPP_TEMPLATE_DECL_NO_AABB
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Chunk(ChunkedBvhDbvt *bp,
														int32_t chunkId,
														float chunkSize,
														Aabb aabb) :
	bvh{{EntitiesOffsetsMapType_Reference(&(bp->entitiesOffsets), chunkId&(-2))}, {EntitiesOffsetsMapType_Reference(&(bp->entitiesOffsets), chunkId|1)}}
{
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Add(EntityType entity,
														   Aabb aabb,
														   MaskType mask)
{
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Update(
	EntityType entity, Aabb aabb, int32_t oldSegment)
{
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Remove(EntityType entity,
															  int32_t segment)
{
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::RebuildIfNeeded() {}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::Rebuild() {}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::ShrinkToFit() {}

SPP_TEMPLATE_DECL_NO_AABB
size_t ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::GetMemoryUsage() const
{
	 return {};
}

SPP_TEMPLATE_DECL_NO_AABB
Aabb
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::
GetAabb(
	EntityType entity, int32_t segment, int32_t offset) const
{
	 return {};
}

SPP_TEMPLATE_DECL_NO_AABB
MaskType ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::GetMask(
	EntityType entity, int32_t segment, int32_t offset) const
{
	 return {};
}

SPP_TEMPLATE_DECL_NO_AABB
Aabb_i16
ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::ToLocalAabb(Aabb aabb) const
{
	 return {};
}

SPP_TEMPLATE_DECL_NO_AABB
int32_t ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::Chunk::GetCount() const {
	 return {};
}
} // namespace spp
