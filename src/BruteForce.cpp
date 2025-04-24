// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/BruteForce.hpp"

namespace spp
{
BruteForce::BruteForce() : iterator(*this) {}
BruteForce::~BruteForce() {}

const char *BruteForce::GetName() const { return "BruteForce"; }

void BruteForce::Clear() { entitiesData.clear(); }

size_t BruteForce::GetMemoryUsage() const
{
	return entitiesData.bucket_count() * sizeof(void *) +
		   entitiesData.size() *
			   (sizeof(void *) * 2lu + sizeof(Data) + sizeof(EntityType));
}

void BruteForce::ShrinkToFit() {}

void BruteForce::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	assert(Exists(entity) == false);
	entitiesData[entity] = Data{aabb, entity, mask};
}

void BruteForce::Update(EntityType entity, Aabb aabb)
{
	entitiesData[entity].aabb = aabb;
}

void BruteForce::Remove(EntityType entity) { entitiesData.erase(entity); }

void BruteForce::SetMask(EntityType entity, MaskType mask)
{
	entitiesData[entity].mask = mask;
}

int32_t BruteForce::GetCount() const
{
	return entitiesData.size();
}

bool BruteForce::Exists(EntityType entity) const
{
	return entitiesData.find(entity) != entitiesData.end();
}

Aabb BruteForce::GetAabb(EntityType entity) const
{
	auto it = entitiesData.find(entity);
	if (it != entitiesData.end()) {
		return it->second.aabb;
	}
	return {};
}

MaskType BruteForce::GetMask(EntityType entity) const
{
	auto it = entitiesData.find(entity);
	if (it != entitiesData.end()) {
		return it->second.mask;
	}
	return 0;
}

void BruteForce::Rebuild() {}

void BruteForce::IntersectAabb(IntersectionCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	cb.broadphase = this;

	for (const auto &it : entitiesData) {
		if (it.second.mask & cb.mask) {
			if (it.second.aabb && cb.aabb) {
				cb.callback(&cb, it.first);
				++cb.testedCount;
			}
			++cb.nodesTestedCount;
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

	for (const auto &it : entitiesData) {
		if (it.second.mask & cb.mask) {
			cb.ExecuteIfRelevant(it.second.aabb, it.first);
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
	it = map->begin();
	FetchData();
}

BruteForce::Iterator::~Iterator() {}

bool BruteForce::Iterator::Next()
{
	++it;
	return FetchData();
}

bool BruteForce::Iterator::FetchData()
{
	if (Valid()) {
		entity = it->second.entity;
		aabb = it->second.aabb;
		mask = it->second.mask;
		return true;
	}
	return false;
}

bool BruteForce::Iterator::Valid()
{
	return it != map->end();
}
} // namespace spp
