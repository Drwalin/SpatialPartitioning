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
	BruteForce() = default;
	virtual ~BruteForce() = default;
	
	virtual void Clear() override;
	virtual size_t GetMemoryUsage() const override;

	virtual void Add(EntityType entity, AABB aabb,
					 MaskType mask) override;
	virtual void Update(EntityType entity, AABB aabb) override;
	virtual void Remove(EntityType entity) override;
	virtual void SetMask(EntityType entity, MaskType mask) override;

	virtual void IntersectAABB(IntersectionCallback &callback) const override;
	virtual void IntersectRay(RayCallback &callback) const override;

private:
	struct Data {
		EntityType entity;
		AABB aabb;
		MaskType mask;
	};

	std::unordered_map<EntityType, Data> entities;
};
} // namespace spp
