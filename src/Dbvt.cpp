// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/Dbvt.hpp"

namespace spp
{
SPP_TEMPLATE_DECL_OFFSET
Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::Dbvt() : dbvt(this), iterator(*this) {}
SPP_TEMPLATE_DECL_OFFSET
Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::~Dbvt() { Clear(); }

SPP_TEMPLATE_DECL_OFFSET
const char *Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::GetName() const { return "Dbvt"; }

SPP_TEMPLATE_DECL_OFFSET
void Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::Clear()
{
	ents.Clear();
	dbvt.clear();
}

SPP_TEMPLATE_DECL_OFFSET
size_t Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::GetMemoryUsage() const
{
	return ents.GetMemoryUsage() + dbvt.GetMemoryUsage();
}

SPP_TEMPLATE_DECL_OFFSET
void Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::ShrinkToFit() { ents.ShrinkToFit(); }

SPP_TEMPLATE_DECL_OFFSET
void Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::SmallRebuildIfNeeded()
{
	if (requiresRebuild > 1000) {
		IncrementalOptimize(requiresRebuild / 133 + 1);
		requiresRebuild = 0;
	}
}

SPP_TEMPLATE_DECL_OFFSET
void Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::IncrementalOptimize(int iterations)
{
	dbvt.optimizeIncremental(iterations);
}

SPP_TEMPLATE_DECL_OFFSET
void Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	assert(Exists(entity) == false);

	uint32_t offset = ents.Add(entity, Data{aabb, 0, entity, mask});
	dbvt.insert(aabb, offset);
	requiresRebuild++;
}

SPP_TEMPLATE_DECL_OFFSET
void Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::Update(EntityType entity, Aabb aabb)
{
	uint32_t offset = ents.GetOffset(entity);
	if (offset > 0) {
		ents[offset].aabb = aabb;
		dbvt.updateEntityOffset(offset, aabb);
		requiresRebuild++;
	} else {
		assert(Exists(entity) == true);
	}
}

SPP_TEMPLATE_DECL_OFFSET
void Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::Remove(EntityType entity)
{
	uint32_t offset = ents.GetOffset(entity);
	if (offset > 0) {
		dbvt.remove(offset);
		ents.RemoveByKey(entity);
		requiresRebuild++;
	} else {
		assert(Exists(entity) == true);
	}
}

SPP_TEMPLATE_DECL_OFFSET
void Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::SetMask(EntityType entity, MaskType mask)
{
	uint32_t offset = ents.GetOffset(entity);
	if (offset > 0) {
		ents[offset].mask = mask;
	} else {
		assert(Exists(entity) == true);
	}
}

SPP_TEMPLATE_DECL_OFFSET
int32_t Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::GetCount() const { return ents.Size(); }

SPP_TEMPLATE_DECL_OFFSET
bool Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::Exists(EntityType entity) const
{
	return ents.GetOffset(entity) > 0;
}

SPP_TEMPLATE_DECL_OFFSET
Aabb Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::GetAabb(EntityType entity) const
{
	uint32_t offset = ents.GetOffset(entity);
	if (offset > 0) {
		return ents[offset].aabb;
	}
	assert(Exists(entity) == true);
	return {};
}

SPP_TEMPLATE_DECL_OFFSET
MaskType Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::GetMask(EntityType entity) const
{
	assert(Exists(entity) == true);
	uint32_t offset = ents.GetOffset(entity);
	if (offset > 0) {
		return ents[offset].mask;
	}
	return 0;
}

SPP_TEMPLATE_DECL_OFFSET
void Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::Rebuild()
{
	requiresRebuild += 3000;
	SmallRebuildIfNeeded();
}

SPP_TEMPLATE_DECL_OFFSET
void Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::IntersectAabb(AabbCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	SmallRebuildIfNeeded();

	cb.broadphase = this;

	dbvt.collideTV(cb);
}

SPP_TEMPLATE_DECL_OFFSET
void Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::IntersectRay(RayCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	SmallRebuildIfNeeded();

	cb.broadphase = this;
	cb.InitVariables();

	dbvt.rayTestInternal(cb);
}

SPP_TEMPLATE_DECL_OFFSET
BroadphaseBaseIterator<SPP_TEMPLATE_ARGS> *Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::RestartIterator()
{
	iterator = {*this};
	return &iterator;
}

SPP_TEMPLATE_DECL_OFFSET
Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::Iterator::Iterator(Dbvt &bp)
{
	data = &(bp.ents._Data()._Data());
	it = 0;
	Next();
}

SPP_TEMPLATE_DECL_OFFSET
Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::Iterator::~Iterator() {}

SPP_TEMPLATE_DECL_OFFSET
bool Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::Iterator::Next()
{
	do {
		++it;
	} while (Valid() && (*data)[it].entity == EMPTY_ENTITY);
	return FetchData();
}

SPP_TEMPLATE_DECL_OFFSET
bool Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::Iterator::FetchData()
{
	if (Valid()) {
		Data d = (*data)[it];
		this->entity = d.entity;
		this->aabb = d.aabb;
		this->mask = d.mask;
		return true;
	}
	return false;
}

SPP_TEMPLATE_DECL_OFFSET
bool Dbvt<SPP_TEMPLATE_ARGS_OFFSET>::Iterator::Valid() { return it < data->size(); }

SPP_DEFINE_VARIANTS_OFFSET(Dbvt)

} // namespace spp
