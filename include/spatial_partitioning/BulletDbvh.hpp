// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <cstdint>

#include "../../src/bullet/btDbvtBroadphase.h"

#include "AssociativeArray.hpp"
#include "BroadPhaseBase.hpp"

namespace spp
{
class BulletDbvh final : public BroadphaseBase
{
public:
	BulletDbvh();
	virtual ~BulletDbvh();

	virtual const char *GetName() const override;

	virtual void Clear() override;
	virtual size_t GetMemoryUsage() const override;
	virtual void ShrinkToFit() override;

	void IncrementalOptimize(int iterations);

	virtual void Add(EntityType entity, Aabb aabb, MaskType mask) override;
	virtual void Update(EntityType entity, Aabb aabb) override;
	virtual void Remove(EntityType entity) override;
	virtual void SetMask(EntityType entity, MaskType mask) override;

	virtual int32_t GetCount() const override;
	virtual bool Exists(EntityType entity) const override;

	virtual Aabb GetAabb(EntityType entity) const override;
	virtual MaskType GetMask(EntityType entity) const override;

	virtual void IntersectAabb(IntersectionCallback &callback) override;
	virtual void IntersectRay(RayCallback &callback) override;

	virtual void Rebuild() override;

	friend class btAabbCb;
	friend class btRayCb;

	virtual BroadphaseBaseIterator *RestartIterator() override;

private:
	void SmallRebuildIfNeeded();

private:
	struct Data {
		EntityType entity;
		MaskType mask;
		bullet::btBroadphaseProxy *proxy;
	};
	AssociativeArray<EntityType, int32_t, Data> ents;
	bullet::btNullPairCache cache;
	bullet::btDbvtBroadphase broadphase;

	size_t requiresRebuild = 0;

	class Iterator final : public BroadphaseBaseIterator
	{
	public:
		Iterator(BulletDbvh &bp);
		virtual ~Iterator();

		Iterator &operator=(Iterator &&other) = default;

		virtual bool Next() override;
		virtual bool Valid() override;
		bool FetchData();

		std::vector<Data> *data;
		int it;
	} iterator;
};
} // namespace spp
