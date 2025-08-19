// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/ChunkedBvhDbvt.hpp"

namespace spp
{
SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::AabbCallbacks::InterChunkCb::
	InitFrom(ChunkedBvhDbvt::AabbCallback &cb, ChunkedBvhDbvt *dbvt)
{
	orgCb = &cb;
	intraCb.orgCb = &cb;
	this->dbvt = dbvt;

	callback = CallbackImpl;
	intraCb.callback = AabbCallbacks::IntraChunkCb::CallbackImpl;

	intraCb.mask = cb.mask;
	mask = cb.mask;

	aabb = cb.aabb;
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::AabbCallbacks::InterChunkCb::
	CallbackImpl(AabbCallback *cb, uint32_t chunkId)
{
	InterChunkCb *self = (InterChunkCb *)cb;
	Chunk *chunk = self->dbvt->GetChunkById(chunkId);

	chunk->IntersectAabb(self);
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::AabbCallbacks::IntraChunkCb::
	CallbackImpl(BaseCb *cb, EntityType entity)
{
	IntraChunkCb *self = (IntraChunkCb *)cb;
	self->orgCb->ExecuteCallback(entity);
}

SPP_TEMPLATE_DECL_NO_AABB
void ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::RayCallbacks::InterChunkCb::
	InitFrom(ChunkedBvhDbvt::RayCallback &cb, ChunkedBvhDbvt *dbvt)
{
	orgCb = &cb;
	intraCb.orgCb = &cb;
	this->dbvt = dbvt;

	callback = CallbackImpl;
	intraCb.callback = RayCallbacks::IntraChunkCb::CallbackImpl;

	intraCb.mask = cb.mask;
	mask = cb.mask;

	this->cutFactor = cb.cutFactor;
	intraCb.cutFactor = cb.cutFactor;

	memcpy(this->signs, cb.signs, sizeof(cb.signs));
	this->dir = cb.dir;
	this->dirNormalized = cb.dirNormalized;
	this->invDir = cb.invDir;
	this->length = cb.length;
	this->start = cb.start;
	this->end = cb.end;

	memcpy(intraCb.signs, cb.signs, sizeof(cb.signs));
	intraCb.dir = cb.dir * dbvt->chunkSizeMultiplier;
	intraCb.dirNormalized = cb.dirNormalized;
	intraCb.invDir = cb.invDir;
	intraCb.length = cb.length * dbvt->chunkSizeMultiplier;

	intraCb.start = cb.start;
	intraCb.end = cb.end;
}

SPP_TEMPLATE_DECL_NO_AABB
RayPartialResult ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::RayCallbacks::
	InterChunkCb::CallbackImpl(RayCallback *cb, uint32_t chunkId)
{
	InterChunkCb *self = (InterChunkCb *)cb;
	Chunk *chunk = self->dbvt->GetChunkById(chunkId);
	chunk->IntersectRay(self);
	self->cutFactor = self->orgCb->cutFactor;
	return {1, false};
}

SPP_TEMPLATE_DECL_NO_AABB
RayPartialResult ChunkedBvhDbvt<SPP_TEMPLATE_ARGS_NO_AABB>::RayCallbacks::
	IntraChunkCb::CallbackImpl(BaseCb *cb, EntityType entity)
{
	IntraChunkCb *self = (IntraChunkCb *)cb;
	RayPartialResult ret = self->orgCb->ExecuteCallback(entity);
	self->cutFactor = self->orgCb->cutFactor;
	return ret;
}

// SPP_DEFINE_VARIANTS_NO_AABB(ChunkedBvhDbvt)

} // namespace spp
