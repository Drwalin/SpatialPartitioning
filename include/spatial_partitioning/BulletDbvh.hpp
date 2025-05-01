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
SPP_TEMPLATE_DECL
class btAabbCb;

SPP_TEMPLATE_DECL
class btRayCb;
	
SPP_TEMPLATE_DECL
class BulletDbvh final : public BroadphaseBase<SPP_TEMPLATE_ARGS>
{
public:
	
	using AabbCallback = spp::AabbCallback<SPP_TEMPLATE_ARGS>;
	using RayCallback = spp::RayCallback<SPP_TEMPLATE_ARGS>;
	using BroadphaseBaseIterator = spp::BroadphaseBaseIterator<SPP_TEMPLATE_ARGS>;
	
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

	virtual void IntersectAabb(AabbCallback &callback) override;
	virtual void IntersectRay(RayCallback &callback) override;

	virtual void Rebuild() override;

	friend btAabbCb<SPP_TEMPLATE_ARGS>;
	friend btRayCb<SPP_TEMPLATE_ARGS>;

	virtual BroadphaseBaseIterator *RestartIterator() override;

private:
	void SmallRebuildIfNeeded();

private:
	struct Data {
		Aabb aabb;
		EntityType entity = 0;
		MaskType mask = 0;
		bullet::btBroadphaseProxy *proxy = 0;
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

SPP_EXTERN_VARIANTS(BulletDbvh)
	
} // namespace spp
