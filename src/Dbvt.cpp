// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/Dbvt.hpp"

namespace spp
{
Dbvt::Dbvt() : iterator(*this) {}
Dbvt::~Dbvt() { Clear(); }

const char *Dbvt::GetName() const { return "Dbvt"; }

void Dbvt::Clear()
{
	ents.Clear();
	dbvt.clear();
}

size_t Dbvt::GetMemoryUsage() const
{
	return ents.GetMemoryUsage() + (GetCount() * 2 - 1) * sizeof(btDbvtNode);
}

void Dbvt::ShrinkToFit() { ents.ShrinkToFit(); }

void Dbvt::SmallRebuildIfNeeded()
{
	if (requiresRebuild > 1000) {
		IncrementalOptimize(requiresRebuild / 133 + 1);
		requiresRebuild = 0;
	}
}

void Dbvt::IncrementalOptimize(int iterations)
{
	dbvt.optimizeIncremental(iterations);
}

void Dbvt::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	assert(Exists(entity) == false);

	int32_t offset = ents.Add(entity, Data{aabb, nullptr, entity, mask});
	Aabb volume = aabb.Expanded(BIG_EPSILON);
	btDbvtNode *node = dbvt.insert(volume, entity);
	ents[offset].node = node;
	requiresRebuild++;
}

void Dbvt::Update(EntityType entity, Aabb aabb)
{
	int32_t offset = ents.GetOffset(entity);
	if (offset > 0) {
		ents[offset].aabb = aabb;
		Aabb volume = aabb.Expanded(BIG_EPSILON);
		dbvt.update(ents[offset].node, volume);
		requiresRebuild++;
	} else {
		assert(Exists(entity) == true);
	}
}

void Dbvt::Remove(EntityType entity)
{
	int32_t offset = ents.GetOffset(entity);
	if (offset > 0) {
		dbvt.remove(ents[offset].node);
		ents.RemoveByKey(entity);
		requiresRebuild++;
	} else {
		assert(Exists(entity) == true);
	}
}

void Dbvt::SetMask(EntityType entity, MaskType mask)
{
	int32_t offset = ents.GetOffset(entity);
	if (offset > 0) {
		ents[offset].mask = mask;
	} else {
		assert(Exists(entity) == true);
	}
}

int32_t Dbvt::GetCount() const { return ents.Size(); }

bool Dbvt::Exists(EntityType entity) const
{
	return ents.GetOffset(entity) > 0;
}

Aabb Dbvt::GetAabb(EntityType entity) const
{
	int32_t offset = ents.GetOffset(entity);
	if (offset > 0) {
		return ents[offset].aabb;
	}
	assert(Exists(entity) == true);
	return {};
}

MaskType Dbvt::GetMask(EntityType entity) const
{
	assert(Exists(entity) == true);
	int32_t offset = ents.GetOffset(entity);
	if (offset > 0) {
		return ents[offset].mask;
	}
	return 0;
}

void Dbvt::Rebuild()
{
	requiresRebuild += 3000;
	SmallRebuildIfNeeded();
}

void Dbvt::IntersectAabb(IntersectionCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	SmallRebuildIfNeeded();

	cb.broadphase = this;

	dbvt.collideTV(cb);
}

void Dbvt::IntersectRay(RayCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	SmallRebuildIfNeeded();

	cb.broadphase = this;
	cb.InitVariables();

	dbvt.rayTestInternal(cb);
}

BroadphaseBaseIterator *Dbvt::RestartIterator()
{
	iterator = {*this};
	return &iterator;
}

Dbvt::Iterator::Iterator(Dbvt &bp)
{
	data = &(bp.ents._Data()._Data());
	it = 0;
	Next();
}

Dbvt::Iterator::~Iterator() {}

bool Dbvt::Iterator::Next()
{
	do {
		++it;
	} while (Valid() && (*data)[it].entity == EMPTY_ENTITY);
	return FetchData();
}

bool Dbvt::Iterator::FetchData()
{
	if (Valid()) {
		Data d = (*data)[it];
		entity = d.entity;
		aabb = d.aabb;
		mask = d.mask;
		return true;
	}
	return false;
}

bool Dbvt::Iterator::Valid() { return it < data->size(); }
} // namespace spp
