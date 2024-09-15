// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <unordered_map>

#include "BroadPhaseBase.hpp"

namespace spp
{
class BruteForce final : public BroadphaseBase
{
public:
	BruteForce();
	virtual ~BruteForce();
	
	virtual void Clear() override;
	virtual size_t GetMemoryUsage() const override;
	virtual void ShrinkToFit() override;

	virtual void Add(EntityType entity, Aabb aabb,
					 MaskType mask) override;
	virtual void Update(EntityType entity, Aabb aabb) override;
	virtual void Remove(EntityType entity) override;
	virtual void SetMask(EntityType entity, MaskType mask) override;
	
	virtual void Rebuild() override;

	virtual void IntersectAabb(IntersectionCallback &callback) override;
	virtual void IntersectRay(RayCallback &callback) override;

private:
	struct Data {
		Aabb aabb;
		EntityType entity;
		MaskType mask;
	};

	std::unordered_map<EntityType, Data> entitiesData;
};
} // namespace spp
