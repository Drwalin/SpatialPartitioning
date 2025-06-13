// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/ChunkedBvhDbvt.hpp"

namespace spp
{
SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::AabbCallbacks::InterChunkCb::
	CallbackImpl(AabbCallback *, uint32_t chunkId)
{
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::AabbCallbacks::IntraChunkCb::
	CallbackImpl(AabbCallback *, EntityType entity)
{
}

SPP_TEMPLATE_DECL_NO_AABB
RayPartialResult ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::RayCallbacks::
	InterChunkCb::CallbackImpl(RayCallback *, uint32_t chunkId)
{
	 return {};
}

SPP_TEMPLATE_DECL_NO_AABB
RayPartialResult ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::RayCallbacks::
	IntraChunkCb::CallbackImpl(RayCallback *, EntityType entity)
{
	 return {};
}

} // namespace spp
