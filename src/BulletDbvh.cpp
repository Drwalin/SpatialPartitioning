// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/BulletDbvh.hpp"

static bullet::btVector3 bt(glm::vec3 v) { return {v.x, v.y, v.z}; }
static glm::vec3 gl(bullet::btVector3 v) { return {v.x(), v.y(), v.z()}; }

namespace spp
{

SPP_TEMPLATE_DECL
BulletDbvh<SPP_TEMPLATE_ARGS>::BulletDbvh()
	: broadphase(&cache), iterator(*this)
{
	broadphase.m_deferedcollide = true;
}
SPP_TEMPLATE_DECL
BulletDbvh<SPP_TEMPLATE_ARGS>::~BulletDbvh() { Clear(); }

SPP_TEMPLATE_DECL
const char *BulletDbvh<SPP_TEMPLATE_ARGS>::GetName() const
{
	return "BulletDbvh";
}

SPP_TEMPLATE_DECL
void BulletDbvh<SPP_TEMPLATE_ARGS>::Clear()
{
	auto &of = ents._Offsets();

	for (auto it : of) {
		if (ents[it.second].proxy) {
			broadphase.destroyProxy(ents[it.second].proxy, nullptr);
		}
	}
	ents.Clear();
}

SPP_TEMPLATE_DECL
size_t BulletDbvh<SPP_TEMPLATE_ARGS>::GetMemoryUsage() const
{
	return ents.GetMemoryUsage() +
		   (broadphase.m_sets[0].m_leaves * 2 - 1) *
			   sizeof(bullet::btDbvtNode) +
		   broadphase.m_sets[0].m_stkStack.capacity() *
			   sizeof(bullet::btDbvt::sStkNN) +
		   (broadphase.m_sets[1].m_leaves * 2 - 1) *
			   sizeof(bullet::btDbvtNode) +
		   broadphase.m_sets[1].m_stkStack.capacity() *
			   sizeof(bullet::btDbvt::sStkNN) +
		   64 * 64 + ents.Size() * sizeof(bullet::btDbvtProxy);
}

SPP_TEMPLATE_DECL
void BulletDbvh<SPP_TEMPLATE_ARGS>::ShrinkToFit() {}

SPP_TEMPLATE_DECL
void BulletDbvh<SPP_TEMPLATE_ARGS>::SmallRebuildIfNeeded()
{
	if (requiresRebuild > 0) {
		IncrementalOptimize(requiresRebuild / 100 + 1);
		requiresRebuild = 0;
	}
}

SPP_TEMPLATE_DECL
void BulletDbvh<SPP_TEMPLATE_ARGS>::IncrementalOptimize(int iterations)
{
	broadphase.m_sets[0].optimizeIncremental(iterations);
	broadphase.m_sets[1].optimizeIncremental(iterations);
}

SPP_TEMPLATE_DECL
void BulletDbvh<SPP_TEMPLATE_ARGS>::Add(EntityType entity, Aabb aabb,
										MaskType mask)
{
	int32_t offset = ents.Add(entity, Data{aabb, entity, mask});
	aabb = aabb.Expanded(BIG_EPSILON);
	bullet::btBroadphaseProxy *proxy =
		broadphase.createProxy(bt(aabb.min), bt(aabb.max), 0,
							   (void *)(uint64_t)offset, mask, mask, nullptr);
	ents[offset].proxy = proxy;
	requiresRebuild++;
}

SPP_TEMPLATE_DECL
void BulletDbvh<SPP_TEMPLATE_ARGS>::Update(EntityType entity, Aabb aabb)
{
	int32_t offset = ents.GetOffset(entity);
	ents[offset].aabb = aabb;
	aabb = aabb.Expanded(BIG_EPSILON);
	broadphase.setAabb(ents[offset].proxy, bt(aabb.min), bt(aabb.max), nullptr);
	requiresRebuild++;
}

SPP_TEMPLATE_DECL
void BulletDbvh<SPP_TEMPLATE_ARGS>::Remove(EntityType entity)
{
	int32_t offset = ents.GetOffset(entity);
	broadphase.destroyProxy(ents[offset].proxy, nullptr);
	ents.RemoveByKey(entity);
	requiresRebuild++;
}

SPP_TEMPLATE_DECL
void BulletDbvh<SPP_TEMPLATE_ARGS>::SetMask(EntityType entity, MaskType mask)
{
	int32_t offset = ents.GetOffset(entity);
	ents[offset].proxy->m_collisionFilterMask = mask;
	ents[offset].proxy->m_collisionFilterGroup = mask;
	ents[offset].mask = mask;
}

SPP_TEMPLATE_DECL
int32_t BulletDbvh<SPP_TEMPLATE_ARGS>::GetCount() const { return ents.Size(); }

SPP_TEMPLATE_DECL
bool BulletDbvh<SPP_TEMPLATE_ARGS>::Exists(EntityType entity) const
{
	return ents.GetOffset(entity) > 0;
}

SPP_TEMPLATE_DECL
Aabb BulletDbvh<SPP_TEMPLATE_ARGS>::GetAabb(EntityType entity) const
{
	const Data &data = ents[ents.GetOffset(entity)];
	return data.aabb;
}

SPP_TEMPLATE_DECL
MaskType BulletDbvh<SPP_TEMPLATE_ARGS>::GetMask(EntityType entity) const
{
	return ents[ents.GetOffset(entity)].mask;
}

SPP_TEMPLATE_DECL
void BulletDbvh<SPP_TEMPLATE_ARGS>::Rebuild()
{
	// 	broadphase.optimize();
	requiresRebuild = false;
	IncrementalOptimize(100);
}

SPP_TEMPLATE_DECL
class btAabbCb final : public bullet::btBroadphaseAabbCallback
{
public:
	btAabbCb(BulletDbvh<SPP_TEMPLATE_ARGS> *bp,
			 AabbCallback<SPP_TEMPLATE_ARGS> *cb)
		: bp(bp), cb(cb)
	{
	}
	virtual ~btAabbCb() {}
	virtual bool process(const bullet::btBroadphaseProxy *p) override
	{
		int32_t offset = (int32_t)(uint64_t)(p->m_clientObject);
		auto &data = bp->ents[offset];
		if (cb->mask & data.mask) {
			cb->callback(cb, data.entity);
			cb->testedCount++;
		}
		// TODO: test return false
		return true;
	}

	BulletDbvh<SPP_TEMPLATE_ARGS> *bp;
	AabbCallback<SPP_TEMPLATE_ARGS> *cb;
};

SPP_TEMPLATE_DECL
void BulletDbvh<SPP_TEMPLATE_ARGS>::IntersectAabb(AabbCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	SmallRebuildIfNeeded();

	cb.broadphase = this;
	btAabbCb btCb{this, &cb};
	broadphase.aabbTest(bt(cb.aabb.min), bt(cb.aabb.max), btCb);
}

SPP_TEMPLATE_DECL
class btRayCb final : public bullet::btBroadphaseRayCallback
{
public:
	btRayCb(BulletDbvh<SPP_TEMPLATE_ARGS> *bp,
			RayCallback<SPP_TEMPLATE_ARGS> *cb)
		: bp(bp), cb(cb)
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
	virtual ~btRayCb() {}
	virtual bool process(const bullet::btBroadphaseProxy *p) override
	{
		if (cb->cutFactor == bullet::btScalar(0.f))
			return false;

		bool hasHit = false;
		int32_t offset = (int32_t)(uint64_t)(p->m_clientObject);
		auto &data = bp->ents[offset];
		if (cb->mask & data.mask) {
			auto res = cb->ExecuteCallback(data.entity);
			if (res.intersection) {
				hasHit = true;
				m_lambda_max = lambdaOrig * res.dist;
			}
		}
		// TODO: test return false
		return !hasHit;
	}

	BulletDbvh<SPP_TEMPLATE_ARGS> *bp;
	RayCallback<SPP_TEMPLATE_ARGS> *cb;
	float lambdaOrig;
};

SPP_TEMPLATE_DECL
void BulletDbvh<SPP_TEMPLATE_ARGS>::IntersectRay(RayCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	SmallRebuildIfNeeded();

	cb.broadphase = this;
	cb.InitVariables();

	btRayCb btCb{this, &cb};
	broadphase.rayTest(bt(cb.start), bt(cb.end), btCb);
}

SPP_TEMPLATE_DECL
BroadphaseBaseIterator<SPP_TEMPLATE_ARGS> *
BulletDbvh<SPP_TEMPLATE_ARGS>::RestartIterator()
{
	iterator = {*this};
	return &iterator;
}

SPP_TEMPLATE_DECL
BulletDbvh<SPP_TEMPLATE_ARGS>::Iterator::Iterator(BulletDbvh &bp)
{
	data = &(bp.ents._Data()._Data());
	it = 0;
	Next();
}

SPP_TEMPLATE_DECL
BulletDbvh<SPP_TEMPLATE_ARGS>::Iterator::~Iterator() {}

SPP_TEMPLATE_DECL
bool BulletDbvh<SPP_TEMPLATE_ARGS>::Iterator::Next()
{
	do {
		++it;
	} while (Valid() && (*data)[it].entity == EMPTY_ENTITY);
	return FetchData();
}

SPP_TEMPLATE_DECL
bool BulletDbvh<SPP_TEMPLATE_ARGS>::Iterator::FetchData()
{
	if (Valid()) {
		this->entity = (*data)[it].entity;
		Data d = (*data)[it];
		this->aabb = {gl(d.proxy->m_aabbMin), gl(d.proxy->m_aabbMax)};
		this->mask = (*data)[it].mask;
		return true;
	}
	return false;
}

SPP_TEMPLATE_DECL
bool BulletDbvh<SPP_TEMPLATE_ARGS>::Iterator::Valid()
{
	return it < data->size();
}

SPP_DEFINE_VARIANTS(BulletDbvh)

} // namespace spp
