// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/BulletDbvt.hpp"

static bullet::btVector3 bt(glm::vec3 v) { return {v.x, v.y, v.z}; }
static bullet::btDbvtAabbMm bt(spp::Aabb v)
{
	return bullet::btDbvtVolume::FromMM(bt(v.min), bt(v.max));
}

namespace spp
{
SPP_TEMPLATE_DECL
BulletDbvt<SPP_TEMPLATE_ARGS>::BulletDbvt() : iterator(*this) {}
SPP_TEMPLATE_DECL
BulletDbvt<SPP_TEMPLATE_ARGS>::~BulletDbvt() { Clear(); }

SPP_TEMPLATE_DECL
const char *BulletDbvt<SPP_TEMPLATE_ARGS>::GetName() const
{
	return "BulletDbvt";
}

SPP_TEMPLATE_DECL
void BulletDbvt<SPP_TEMPLATE_ARGS>::Clear()
{
	ents.Clear();
	dbvt.clear();
}

SPP_TEMPLATE_DECL
size_t BulletDbvt<SPP_TEMPLATE_ARGS>::GetMemoryUsage() const
{
	return ents.GetMemoryUsage() +
		   (GetCount() * 2 - 1) * sizeof(bullet::btDbvtNode) +
		   dbvt.m_stkStack.capacity() * sizeof(bullet::btDbvt::sStkNN) +
		   stack.capacity() * sizeof(const bullet::btDbvtNode *);
}

SPP_TEMPLATE_DECL
void BulletDbvt<SPP_TEMPLATE_ARGS>::ShrinkToFit() { ents.ShrinkToFit(); }

SPP_TEMPLATE_DECL
void BulletDbvt<SPP_TEMPLATE_ARGS>::SmallRebuildIfNeeded()
{
	if (requiresRebuild > 1000) {
		IncrementalOptimize(requiresRebuild / 133 + 1);
		requiresRebuild = 0;
	}
}

SPP_TEMPLATE_DECL
void BulletDbvt<SPP_TEMPLATE_ARGS>::IncrementalOptimize(int iterations)
{
	dbvt.optimizeIncremental(iterations);
}

SPP_TEMPLATE_DECL
void BulletDbvt<SPP_TEMPLATE_ARGS>::Add(EntityType entity, Aabb aabb,
										MaskType mask)
{
	assert(Exists(entity) == false);

	int32_t offset = ents.Add(entity, Data{aabb, nullptr, entity, mask});
	bullet::btDbvtAabbMm volume = bt(aabb.Expanded(BIG_EPSILON));
	bullet::btDbvtNode *node = dbvt.insert(volume, (void *)(int64_t)offset);
	ents[offset].node = node;
	requiresRebuild++;
}

SPP_TEMPLATE_DECL
void BulletDbvt<SPP_TEMPLATE_ARGS>::Update(EntityType entity, Aabb aabb)
{
	int32_t offset = ents.GetOffset(entity);
	if (offset > 0) {
		ents[offset].aabb = aabb;
		bullet::btDbvtAabbMm volume = bt(aabb.Expanded(BIG_EPSILON));
		dbvt.update(ents[offset].node, volume);
		requiresRebuild++;
	} else {
		assert(Exists(entity) == true);
	}
}

SPP_TEMPLATE_DECL
void BulletDbvt<SPP_TEMPLATE_ARGS>::Remove(EntityType entity)
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

SPP_TEMPLATE_DECL
void BulletDbvt<SPP_TEMPLATE_ARGS>::SetMask(EntityType entity, MaskType mask)
{
	int32_t offset = ents.GetOffset(entity);
	if (offset > 0) {
		ents[offset].mask = mask;
	} else {
		assert(Exists(entity) == true);
	}
}

SPP_TEMPLATE_DECL
int32_t BulletDbvt<SPP_TEMPLATE_ARGS>::GetCount() const { return ents.Size(); }

SPP_TEMPLATE_DECL
bool BulletDbvt<SPP_TEMPLATE_ARGS>::Exists(EntityType entity) const
{
	return ents.GetOffset(entity) > 0;
}

SPP_TEMPLATE_DECL
Aabb BulletDbvt<SPP_TEMPLATE_ARGS>::GetAabb(EntityType entity) const
{
	int32_t offset = ents.GetOffset(entity);
	if (offset > 0) {
		return ents[offset].aabb;
	}
	assert(Exists(entity) == true);
	return {};
}

SPP_TEMPLATE_DECL
MaskType BulletDbvt<SPP_TEMPLATE_ARGS>::GetMask(EntityType entity) const
{
	assert(Exists(entity) == true);
	int32_t offset = ents.GetOffset(entity);
	if (offset > 0) {
		return ents[offset].mask;
	}
	return 0;
}

SPP_TEMPLATE_DECL
void BulletDbvt<SPP_TEMPLATE_ARGS>::Rebuild()
{
	requiresRebuild += 3000;
	SmallRebuildIfNeeded();
}

SPP_TEMPLATE_DECL
void BulletDbvt<SPP_TEMPLATE_ARGS>::IntersectAabb(AabbCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	SmallRebuildIfNeeded();

	cb.broadphase = this;

	class btDbvtAabbCb final : public bullet::btDbvt::ICollide
	{
	public:
		btDbvtAabbCb(BulletDbvt *bp, AabbCallback *cb) : bp(bp), cb(cb) {}
		virtual ~btDbvtAabbCb() {}
		virtual void Process(const bullet::btDbvtNode *leaf) override
		{
			int32_t offset = (int32_t)(int64_t)(leaf->data);
			if (bp->ents[offset].mask & cb->mask) {
				cb->callback(cb, bp->ents[offset].entity);
				cb->testedCount++;
			}
		}

		BulletDbvt<SPP_TEMPLATE_ARGS> *bp;
		AabbCallback *cb;
	};

	btDbvtAabbCb btCb{this, &cb};

	bullet::btDbvtVolume bounds = bt(cb.aabb);
	dbvt.collideTVNoStackAlloc(dbvt.m_root, bounds, stack, btCb);
}

SPP_TEMPLATE_DECL
void BulletDbvt<SPP_TEMPLATE_ARGS>::IntersectRay(RayCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	SmallRebuildIfNeeded();

	cb.broadphase = this;
	cb.InitVariables();

	class btDbvtRayCb final : public bullet::btDbvt::ICollide
	{
	public:
		btDbvtRayCb(BulletDbvt *bp, RayCallback *cb) : bp(bp), cb(cb)
		{
			bullet::btVector3 rayDir = bt(cb->end - cb->start);

			rayDir.normalize();
			m_rayDirectionInverse[0] = rayDir[0] == bullet::btScalar(0.0)
										   ? bullet::btScalar(BT_LARGE_FLOAT)
										   : bullet::btScalar(1.0) / rayDir[0];
			m_rayDirectionInverse[1] = rayDir[1] == bullet::btScalar(0.0)
										   ? bullet::btScalar(BT_LARGE_FLOAT)
										   : bullet::btScalar(1.0) / rayDir[1];
			m_rayDirectionInverse[2] = rayDir[2] == bullet::btScalar(0.0)
										   ? bullet::btScalar(BT_LARGE_FLOAT)
										   : bullet::btScalar(1.0) / rayDir[2];
			m_signs[0] = m_rayDirectionInverse[0] < 0.0;
			m_signs[1] = m_rayDirectionInverse[1] < 0.0;
			m_signs[2] = m_rayDirectionInverse[2] < 0.0;

			m_lambda_max = lambdaOrig = rayDir.dot(bt(cb->end - cb->start));
		}
		virtual ~btDbvtRayCb() {}
		virtual void Process(const bullet::btDbvtNode *leaf) override
		{
			if (cb->cutFactor == bullet::btScalar(0.f))
				return;

			int32_t offset = (int32_t)(uint64_t)(leaf->data);
			BulletDbvt::Data &data = bp->ents[offset];
			if (cb->mask & data.mask) {
				auto res = cb->ExecuteCallback(data.entity);
				if (res.intersection) {
					assert(res.dist >= 0);
					assert(res.dist <= 1);
					m_lambda_max = lambdaOrig * res.dist;
				}
			}
		}

		BulletDbvt *bp;
		RayCallback *cb;

		bullet::btVector3 m_rayDirectionInverse;
		unsigned int m_signs[3];
		float m_lambda_max;
		float lambdaOrig;
	};

	btDbvtRayCb btCb{this, &cb};

	dbvt.rayTestInternal(dbvt.m_root, bt(cb.start), bt(cb.end),
						 btCb.m_rayDirectionInverse, btCb.m_signs,
						 btCb.m_lambda_max, bullet::btVector3(0, 0, 0),
						 bullet::btVector3(0, 0, 0), stack, btCb);
}

SPP_TEMPLATE_DECL
BroadphaseBaseIterator<SPP_TEMPLATE_ARGS> *
BulletDbvt<SPP_TEMPLATE_ARGS>::RestartIterator()
{
	iterator = {*this};
	return &iterator;
}

SPP_TEMPLATE_DECL
BulletDbvt<SPP_TEMPLATE_ARGS>::Iterator::Iterator(BulletDbvt &bp)
{
	data = &(bp.ents._Data()._Data());
	it = 0;
	Next();
}

SPP_TEMPLATE_DECL
BulletDbvt<SPP_TEMPLATE_ARGS>::Iterator::~Iterator() {}

SPP_TEMPLATE_DECL
bool BulletDbvt<SPP_TEMPLATE_ARGS>::Iterator::Next()
{
	do {
		++it;
	} while (Valid() && (*data)[it].entity == EMPTY_ENTITY);
	return FetchData();
}

SPP_TEMPLATE_DECL
bool BulletDbvt<SPP_TEMPLATE_ARGS>::Iterator::FetchData()
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

SPP_TEMPLATE_DECL
bool BulletDbvt<SPP_TEMPLATE_ARGS>::Iterator::Valid()
{
	return it < data->size();
}

SPP_DEFINE_VARIANTS(BulletDbvt)

} // namespace spp
