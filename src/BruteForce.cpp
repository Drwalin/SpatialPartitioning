// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/BruteForce.hpp"

namespace spp
{
BruteForce::BruteForce() : iterator(*this) { entitiesData[0] = {{}, 0, 0}; }
BruteForce::~BruteForce() {}

const char *BruteForce::GetName() const { return "BruteForce"; }

void BruteForce::Clear() { entitiesData.Clear(); }

size_t BruteForce::GetMemoryUsage() const
{
	return entitiesData.GetMemoryUsage();
}

void BruteForce::ShrinkToFit() { entitiesData.ShrinkToFit(); }

void BruteForce::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	assert(Exists(entity) == false);
	entitiesData.Add(entity, Data{aabb, entity, mask});
}

void BruteForce::Update(EntityType entity, Aabb aabb)
{
	assert(Exists(entity) == true);
	int32_t offset = entitiesData.GetOffset(entity);
	if (offset > 0) {
		entitiesData[offset].aabb = aabb;
	}
}

void BruteForce::Remove(EntityType entity)
{
	assert(Exists(entity) == true);
	int32_t offset = entitiesData.GetOffset(entity);
	if (offset > 0) {
		entitiesData[offset].entity = 0;
		entitiesData[offset].mask = 0;
		entitiesData.RemoveByKey(entity);
	}
}

void BruteForce::SetMask(EntityType entity, MaskType mask)
{
	assert(Exists(entity) == true);
	int32_t offset = entitiesData.GetOffset(entity);
	if (offset > 0) {
		entitiesData[offset].mask = mask;
	}
}

int32_t BruteForce::GetCount() const { return entitiesData.Size(); }

bool BruteForce::Exists(EntityType entity) const
{
	return entitiesData.GetOffset(entity) > 0;
}

Aabb BruteForce::GetAabb(EntityType entity) const
{
	assert(Exists(entity) == true);
	int32_t offset = entitiesData.GetOffset(entity);
	if (offset > 0) {
		return entitiesData[offset].aabb;
	}
	return {};
}

MaskType BruteForce::GetMask(EntityType entity) const
{
	assert(Exists(entity) == true);
	int32_t offset = entitiesData.GetOffset(entity);
	if (offset > 0) {
		return entitiesData[offset].mask;
	}
	return {};
}

void BruteForce::Rebuild() {}

void BruteForce::IntersectAabb(IntersectionCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	cb.broadphase = this;

	for (const auto &it : entitiesData._Data()._Data()) {
		if (it.entity > 0) {
			if (it.mask & cb.mask) {
				if (it.aabb && cb.aabb) {
					cb.callback(&cb, it.entity);
					++cb.testedCount;
				}
				++cb.nodesTestedCount;
			}
		}
	}
}

void BruteForce::IntersectRay(RayCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	cb.broadphase = this;
	cb.InitVariables();

	for (const auto &it : entitiesData._Data()._Data()) {
		if (it.entity > 0) {
			if (it.mask & cb.mask) {
				cb.ExecuteIfRelevant(it.aabb, it.entity);
			}
		}
	}
}

BroadphaseBaseIterator *BruteForce::RestartIterator()
{
	iterator = {*this};
	return &iterator;
}

BruteForce::Iterator::Iterator(BruteForce &bp)
{
	map = &bp.entitiesData;
	it = 0;
	Next();
}

BruteForce::Iterator::~Iterator() {}

bool BruteForce::Iterator::Next()
{
	do {
		++it;
	} while (Valid() && (*map)[it].entity == EMPTY_ENTITY);
	return FetchData();
}

bool BruteForce::Iterator::FetchData()
{
	if (Valid()) {
		entity = (*map)[it].entity;
		aabb = (*map)[it].aabb;
		mask = (*map)[it].mask;
		return true;
	}
	return false;
}

bool BruteForce::Iterator::Valid() { return it < map->_Data()._Data().size(); }
} // namespace spp
