// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/BruteForce.hpp"

namespace spp
{
SPP_TEMPLATE_DECL
BruteForce<SPP_TEMPLATE_ARGS>::BruteForce() : iterator(*this) { entitiesData[0] = {{}, 0, 0}; }
SPP_TEMPLATE_DECL
BruteForce<SPP_TEMPLATE_ARGS>::~BruteForce() {}

SPP_TEMPLATE_DECL
const char *BruteForce<SPP_TEMPLATE_ARGS>::GetName() const { return "BruteForce"; }

SPP_TEMPLATE_DECL
void BruteForce<SPP_TEMPLATE_ARGS>::Clear() { entitiesData.Clear(); }

SPP_TEMPLATE_DECL
size_t BruteForce<SPP_TEMPLATE_ARGS>::GetMemoryUsage() const
{
	return entitiesData.GetMemoryUsage();
}

SPP_TEMPLATE_DECL
void BruteForce<SPP_TEMPLATE_ARGS>::ShrinkToFit() { entitiesData.ShrinkToFit(); }

SPP_TEMPLATE_DECL
void BruteForce<SPP_TEMPLATE_ARGS>::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	assert(Exists(entity) == false);
	entitiesData.Add(entity, Data{aabb, entity, mask});
}

SPP_TEMPLATE_DECL
void BruteForce<SPP_TEMPLATE_ARGS>::Update(EntityType entity, Aabb aabb)
{
	assert(Exists(entity) == true);
	int32_t offset = entitiesData.GetOffset(entity);
	if (offset > 0) {
		entitiesData[offset].aabb = aabb;
	}
}

SPP_TEMPLATE_DECL
void BruteForce<SPP_TEMPLATE_ARGS>::Remove(EntityType entity)
{
	assert(Exists(entity) == true);
	int32_t offset = entitiesData.GetOffset(entity);
	if (offset > 0) {
		entitiesData[offset].entity = 0;
		entitiesData[offset].mask = 0;
		entitiesData.RemoveByKey(entity);
	}
}

SPP_TEMPLATE_DECL
void BruteForce<SPP_TEMPLATE_ARGS>::SetMask(EntityType entity, MaskType mask)
{
	assert(Exists(entity) == true);
	int32_t offset = entitiesData.GetOffset(entity);
	if (offset > 0) {
		entitiesData[offset].mask = mask;
	}
}

SPP_TEMPLATE_DECL
int32_t BruteForce<SPP_TEMPLATE_ARGS>::GetCount() const { return entitiesData.Size(); }

SPP_TEMPLATE_DECL
bool BruteForce<SPP_TEMPLATE_ARGS>::Exists(EntityType entity) const
{
	return entitiesData.GetOffset(entity) > 0;
}

SPP_TEMPLATE_DECL
Aabb BruteForce<SPP_TEMPLATE_ARGS>::GetAabb(EntityType entity) const
{
	assert(Exists(entity) == true);
	int32_t offset = entitiesData.GetOffset(entity);
	if (offset > 0) {
		return entitiesData[offset].aabb;
	}
	return {};
}

SPP_TEMPLATE_DECL
MaskType BruteForce<SPP_TEMPLATE_ARGS>::GetMask(EntityType entity) const
{
	assert(Exists(entity) == true);
	int32_t offset = entitiesData.GetOffset(entity);
	if (offset > 0) {
		return entitiesData[offset].mask;
	}
	return {};
}

SPP_TEMPLATE_DECL
void BruteForce<SPP_TEMPLATE_ARGS>::Rebuild() {}

SPP_TEMPLATE_DECL
void BruteForce<SPP_TEMPLATE_ARGS>::IntersectAabb(AabbCallback &cb)
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

SPP_TEMPLATE_DECL
void BruteForce<SPP_TEMPLATE_ARGS>::IntersectRay(RayCallback &cb)
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

SPP_TEMPLATE_DECL
BroadphaseBaseIterator<SPP_TEMPLATE_ARGS> *BruteForce<SPP_TEMPLATE_ARGS>::RestartIterator()
{
	iterator = {*this};
	return &iterator;
}

SPP_TEMPLATE_DECL
BruteForce<SPP_TEMPLATE_ARGS>::Iterator::Iterator(BruteForce &bp)
{
	map = &bp.entitiesData;
	it = 0;
	Next();
}

SPP_TEMPLATE_DECL
BruteForce<SPP_TEMPLATE_ARGS>::Iterator::~Iterator() {}

SPP_TEMPLATE_DECL
bool BruteForce<SPP_TEMPLATE_ARGS>::Iterator::Next()
{
	do {
		++it;
	} while (Valid() && (*map)[it].entity == EMPTY_ENTITY);
	return FetchData();
}

SPP_TEMPLATE_DECL
bool BruteForce<SPP_TEMPLATE_ARGS>::Iterator::FetchData()
{
	if (Valid()) {
		this->entity = (*map)[it].entity;
		this->aabb = (*map)[it].aabb;
		this->mask = (*map)[it].mask;
		return true;
	}
	return false;
}

SPP_TEMPLATE_DECL
bool BruteForce<SPP_TEMPLATE_ARGS>::Iterator::Valid() { return it < map->_Data()._Data().size(); }

SPP_DEFINE_VARIANTS(BruteForce)

} // namespace spp
