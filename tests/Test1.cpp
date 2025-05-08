#include <cstdio>

#include <string>
#include <thread>
#include <mutex>
#include <queue>
#include <algorithm>
#include <random>
#include <vector>
#include <chrono>
#include <map>

#include "../include/spatial_partitioning/BroadPhaseBase.hpp"
#include "../include/spatial_partitioning/BruteForce.hpp"
#include "../include/spatial_partitioning/BvhMedianSplitHeap.hpp"
#include "../include/spatial_partitioning/Dbvh.hpp"
#include "../include/spatial_partitioning/HashLooseOctree.hpp"
#include "../include/spatial_partitioning/LooseOctree.hpp"
#include "../include/spatial_partitioning/BulletDbvh.hpp"
#include "../include/spatial_partitioning/BulletDbvt.hpp"
#include "../include/spatial_partitioning/Dbvt.hpp"
#include "../include/spatial_partitioning/ThreeStageDbvh.hpp"

int32_t TOTAL_ENTITIES = 1000000;
int32_t ADDITIONAL_ENTITIES = 10000;
int32_t MAX_ENTITIES = TOTAL_ENTITIES + ADDITIONAL_ENTITIES;
size_t TOTAL_AABB_TESTS = 200000;
size_t TOTAL_AABB_MOVEMENTS = 1000000;
size_t TOTAL_MOVES_AND_TESTS = 1000000;
size_t MAX_MOVING_ENTITIES = 2500;
bool ENABLE_VERIFICATION = true;
bool BENCHMARK = false;
size_t MIXED_AABB_COUNT = 2;
size_t MIXED_RAY_COUNT = 2;
size_t MIXED_UPDATE_COUNT = 1;

bool disable_benchmark_report = false;

std::mt19937_64 mt(12345);

enum TestType {
	TEST_AABB = 1,
	TEST_RAY_FIRST = 2,
	TEST_RAY_ALL = 3,
	TEST_MIXED = 4,
};

const char *testTypeNames[] = {"[NULL-NONE]", "TEST_AABB", "TEST_RAY_FIRST",
							   "TEST_ALL_RAYS", "MIXED"};

using EntityType = uint64_t;


struct EntityData {
	spp::Aabb aabb;
	EntityType id = 0;
	uint32_t mask = ~0;
};

std::vector<std::pair<double, double>> broadphasePerformances;
std::vector<EntityData> *globalEntityData = nullptr;
std::vector<uint64_t> totalErrorsInBroadphase;

struct SingleTestResult {
	size_t nodesTestedCount = 0;
	size_t testedCount = 0;
	size_t hitCount = 0;
};

struct StartEndPoint {
	spp::Aabb aabb;
	spp::Aabb aabb2;
	glm::vec3 start, end, point;
	float n = -3;
	EntityType e = -3;
	bool isAabbTest;

	bool operator<(const StartEndPoint &o) const { return e < o.e; }
};

std::vector<std::vector<spp::Aabb>> currentEntitiesAabbs;
void _SetEntityAabb(std::vector<spp::Aabb> &aabbs, EntityType entity,
					spp::Aabb aabb)
{
	if (aabbs.size() <= entity) {
		aabbs.resize(entity + 1);
	}
	aabbs[entity] = aabb;
}

std::vector<EntityType> ee;
std::vector<glm::vec3> vv;

uint64_t TEST_RANDOM_SEED = 0;

std::vector<spp::Aabb> aabbsToTest;
SingleTestResult SingleTest(spp::BroadphaseBase<spp::Aabb, EntityType, uint32_t, 0> *broadphase,
							const std::vector<spp::Aabb> &aabbsToTest,
							size_t testsCount,
							std::vector<uint64_t> &offsetOfPatch,
							TestType testType,
							std::vector<StartEndPoint> &hitPoints,
							std::vector<spp::Aabb> &currentEntitiesAabbs)
{
	static SingleTestResult ret = {};
	ret.nodesTestedCount = 0;
	ret.testedCount = 0;
	ret.hitCount = 0;
	switch (testType) {
	case TEST_AABB: {

		struct _Cb : public spp::AabbCallback<spp::Aabb, EntityType, uint32_t, 0> {
			std::vector<StartEndPoint> *hitPoints = nullptr;
		} cb;
		cb.hitPoints = &hitPoints;
		cb.mask = ~(uint32_t)0;
		typedef void (*CbT)(spp::AabbCallback<spp::Aabb, EntityType, uint32_t, 0> *, EntityType);
		cb.callback = (CbT) + [](_Cb *cb, EntityType entity) {
			spp::Aabb aabb = cb->broadphase->GetAabb(entity);
			if (cb->IsRelevant(aabb)) {
				ret.hitCount++;
				if (ENABLE_VERIFICATION) {
					cb->hitPoints->push_back(
						StartEndPoint{cb->broadphase->GetAabb(entity),
									  cb->aabb,
									  {},
									  {},
									  {0, 0, 0},
									  -2,
									  entity,
									  true});
				} else {
					static thread_local volatile uint64_t HOLDER_ENT = 0;
					HOLDER_ENT += entity;
				}
			}
		};
		testsCount = std::min(testsCount, aabbsToTest.size());
		for (size_t i = 0; i < testsCount; ++i) {
			if (ENABLE_VERIFICATION) {
				offsetOfPatch.push_back(hitPoints.size());
			}
			cb.aabb = aabbsToTest[i];
			cb.aabb = {glm::min(cb.aabb.min, cb.aabb.max),
					   glm::max(cb.aabb.min, cb.aabb.max)};
			broadphase->IntersectAabb(cb);
		}

		ret.nodesTestedCount = cb.nodesTestedCount;
		ret.testedCount = cb.testedCount;

		return ret;
	} break;
	case TEST_RAY_ALL: {

		struct _Cb : public spp::RayCallback<spp::Aabb, EntityType, uint32_t, 0> {
			std::vector<StartEndPoint> *hitPoints = nullptr;
		} cb;
		cb.hitPoints = &hitPoints;
		cb.mask = ~(uint32_t)0;
		typedef spp::RayPartialResult (*CbT)(spp::RayCallback<spp::Aabb, EntityType, uint32_t, 0> *,
											 EntityType);
		cb.callback =
			(CbT) +
			[](_Cb *cb, EntityType entity) -> spp::RayPartialResult {
			assert(entity > 0);
			float n, f;
			spp::Aabb aabb = cb->broadphase->GetAabb(entity);
			if (cb->IsRelevant(aabb, n, f)) {
				ret.hitCount++;
				assert(n >= 0);
				if (ENABLE_VERIFICATION) {
					cb->hitPoints->push_back(
						StartEndPoint{aabb,
									  {},
									  cb->start,
									  cb->end,
									  cb->start + cb->dir * n,
									  n,
									  entity,
									  false});
				} else {
					static thread_local volatile uint64_t HOLDER_ENT = 0;
					HOLDER_ENT += entity;
				}
				return {1.0f, true};
			}
			return {1.0f, false};
		};
		testsCount = std::min(testsCount, aabbsToTest.size());
		for (size_t i = 0; i < testsCount; ++i) {
			if (ENABLE_VERIFICATION) {
				offsetOfPatch.push_back(hitPoints.size());
			}
			cb.start = aabbsToTest[i].GetCenter();
			cb.end = aabbsToTest[(i + 17) % testsCount].GetCenter();
			cb.initedVars = false;
			broadphase->IntersectRay(cb);
		}

		ret.nodesTestedCount = cb.nodesTestedCount;
		ret.testedCount = cb.testedCount;

		return ret;
	} break;
	case TEST_RAY_FIRST: {
		struct _Cb : public spp::RayCallbackFirstHit<spp::Aabb, EntityType, uint32_t, 0> {
		} cb;
		cb.mask = ~(uint32_t)0;
		typedef spp::RayPartialResult (*CbT)(spp::RayCallback<spp::Aabb, EntityType, uint32_t, 0> *,
											 EntityType);
		cb.callback =
			(CbT) +
			[](_Cb *cb, EntityType entity) -> spp::RayPartialResult {
			float n, f;
			spp::Aabb aabb = cb->broadphase->GetAabb(entity);
			if (cb->IsRelevant(aabb, n, f)) {
				if (n < 0.0f) {
					n = 0.0f;
				}
				if (cb->hasHit == false) {
					if (n < cb->cutFactor) {
						cb->cutFactor = n;
						cb->hitPoint = cb->start + cb->dir * n;
						cb->hitEntity = entity;
						cb->hasHit = true;
						return {n, true};
					}
				} else if (n < cb->cutFactor) {
					cb->cutFactor = n;
					cb->hitPoint = cb->start + cb->dir * n;
					cb->hitEntity = entity;
					cb->hasHit = true;
					return {n, true};
				}
			}
			return {1.0f, false};
		};
		testsCount = std::min(testsCount, aabbsToTest.size());
		for (size_t i = 0; i < testsCount; ++i) {
			cb.hasHit = false;
			if (ENABLE_VERIFICATION) {
				offsetOfPatch.push_back(hitPoints.size());
			}
			cb.start = aabbsToTest[i].GetCenter();
			glm::vec3 end = cb.end =
				aabbsToTest[(i + 1) % testsCount].GetCenter();
			cb.initedVars = false;
			broadphase->IntersectRay(cb);
			if (ENABLE_VERIFICATION) {
				if (cb.hasHit) {
					ret.hitCount++;
					hitPoints.push_back(
						StartEndPoint{broadphase->GetAabb(cb.hitEntity),
									  {},
									  cb.start,
									  end,
									  cb.hitPoint,
									  cb.cutFactor,
									  cb.hitEntity,
									  false});
				} else {
					hitPoints.push_back(
						StartEndPoint{{}, {}, cb.start, end, {}, -1, 0, false});
				}
			} else {
				if (cb.hasHit) {
					ret.hitCount++;
					static thread_local volatile uint64_t HOLDER_ENT = 0;
					HOLDER_ENT += cb.hitEntity;
				}
			}
		}

		ret.nodesTestedCount = cb.nodesTestedCount;
		ret.testedCount = cb.testedCount;

		return ret;
	} break;
	case TEST_MIXED: {

		struct _CbAabb : public spp::AabbCallback<spp::Aabb, EntityType, uint32_t, 0> {
			std::vector<StartEndPoint> *hitPoints = nullptr;
		} cbAabb;
		cbAabb.hitPoints = &hitPoints;
		cbAabb.mask = ~(uint32_t)0;
		typedef void (*CbTAabb)(spp::AabbCallback<spp::Aabb, EntityType, uint32_t, 0> *, EntityType);
		cbAabb.callback = (CbTAabb) + [](_CbAabb *cb, EntityType entity) {
			spp::Aabb aabb = cb->broadphase->GetAabb(entity);
			if (cb->IsRelevant(aabb)) {
				ret.hitCount++;
				if (ENABLE_VERIFICATION) {
					cb->hitPoints->push_back(
						StartEndPoint{cb->broadphase->GetAabb(entity),
									  cb->aabb,
									  {},
									  {},
									  {0, 0, 0},
									  -2,
									  entity,
									  true});
				} else {
					static thread_local volatile uint64_t HOLDER_ENT = 0;
					HOLDER_ENT += entity;
				}
			}
		};

		struct _CbRay : public spp::RayCallbackFirstHit<spp::Aabb, EntityType, uint32_t, 0> {
		} cbRay;
		cbRay.mask = ~(uint32_t)0;
		typedef spp::RayPartialResult (*CbTRay)(spp::RayCallback<spp::Aabb, EntityType, uint32_t, 0> *,
												EntityType);
		cbRay.callback =
			(CbTRay) +
			[](_CbRay *cb, EntityType entity) -> spp::RayPartialResult {
			float n, f;
			spp::Aabb aabb = cb->broadphase->GetAabb(entity);
			if (cb->IsRelevant(aabb, n, f)) {
				if (n < 0.0f) {
					n = 0.0f;
				}
				if (cb->hasHit == false) {
					if (n < cb->cutFactor) {
						cb->cutFactor = n;
						cb->hitPoint = cb->start + cb->dir * n;
						cb->hitEntity = entity;
						cb->hasHit = true;
						return {n, true};
					}
				} else if (n < cb->cutFactor) {
					cb->cutFactor = n;
					cb->hitPoint = cb->start + cb->dir * n;
					cb->hitEntity = entity;
					cb->hasHit = true;
					return {n, true};
				} else if (n - 0.000001f < cb->cutFactor &&
						   entity > cb->hitEntity) {
					cb->cutFactor = n;
					cb->hitPoint = cb->start + cb->dir * n;
					cb->hitEntity = entity;
					cb->hasHit = true;
					return {n, true};
				}
			}
			return {1.0f, false};
		};

		testsCount = std::min(testsCount / 5, vv.size() / 5) * 5;

		static std::vector<EntityType> removeEntities;
		removeEntities.reserve(10000);
		removeEntities.clear();

		/*
		static std::mt19937_64 _mt;
		_mt = std::mt19937_64(TEST_RANDOM_SEED);
		*/

		uint64_t s = 14695981039346656037lu;
		static uint64_t (*Random)(uint64_t &s, size_t i) =
			+[](uint64_t &s, size_t i) -> uint64_t {
			/*
			if ((i & 255) == 0) {
				s = _mt();
			} else {
			*/
			s ^= i;
			s *= 1099511628211lu;
			/*
			}
			*/
			return s;
		};

		static EntityType (*popRandom)(
			uint64_t &s, size_t i,
			std::vector<EntityType> &removeEntities) =
			+[](uint64_t &s, size_t i,
				std::vector<EntityType> &removeEntities)
			-> EntityType {
			if (!removeEntities.empty() && (Random(s, i) % 13) > 10) {
				size_t x = Random(s, i) % removeEntities.size();
				auto r = removeEntities[x];
				removeEntities[x] = removeEntities.back();
				removeEntities.resize(removeEntities.size() - 1);
				return r;
			} else {
				return (((Random(s, i) % MAX_ENTITIES) + 1 +
						 (Random(s, i) % MAX_ENTITIES) + 1 +
						 (Random(s, i) % MAX_ENTITIES) + 1 +
						 (Random(s, i) % MAX_ENTITIES) + 1 +
						 (Random(s, i) % MAX_ENTITIES) + 1 +
						 (Random(s, i) % MAX_ENTITIES) + 1 +
						 (Random(s, i) % MAX_ENTITIES) + 1 +
						 (Random(s, i) % MAX_ENTITIES) + 1) /
						8) %
					   MAX_ENTITIES;
			}
		};

		const size_t stride =
			MIXED_AABB_COUNT + MIXED_RAY_COUNT + MIXED_UPDATE_COUNT;
		
		for (size_t i = 0; i + stride < testsCount;) {
			for (int j = 0; j < MIXED_AABB_COUNT; ++j, ++i) {
				if (ENABLE_VERIFICATION) {
					offsetOfPatch.push_back(hitPoints.size());
				}
				cbAabb.aabb = aabbsToTest[i];
				cbAabb.aabb = {glm::min(cbAabb.aabb.min, cbAabb.aabb.max),
							   glm::max(cbAabb.aabb.min, cbAabb.aabb.max)};
				broadphase->IntersectAabb(cbAabb);
			}

			for (int j = 0; j < MIXED_UPDATE_COUNT; ++j, ++i) {
				auto e = ee[i];
				if (Random(s, i) % 17 > 15) {
					e = (Random(s, i) % (MAX_ENTITIES-1)) + 1;
				}
				if (((i * MIXED_UPDATE_COUNT) / stride + j) % 55 == 0 ||
					broadphase->Exists(e) == false) {
					if (broadphase->Exists(e) == false) {
						spp::Aabb aabb = aabbsToTest[i];
						aabb.max = aabb.min + ((vv[i] + 1000.0f) / 400.0f);
						assert(e > 0);
						broadphase->Add(e, aabb, ~0);
						_SetEntityAabb(currentEntitiesAabbs, e, aabb);
					}
					e = popRandom(s, i, removeEntities);

					if (broadphase->Exists(e)) {
						broadphase->Remove(e);
						removeEntities.push_back(e);
					} else {
						spp::Aabb aabb = aabbsToTest[i];
						aabb.max = aabb.min + ((vv[i] + 1000.0f) / 400.0f);
						assert(e > 0);
						broadphase->Add(e, aabb, ~0);
						_SetEntityAabb(currentEntitiesAabbs, e, aabb);
					}
				} else {
					spp::Aabb aabb = broadphase->GetAabb(e);
					aabb.min += vv[i];
					aabb.max += vv[i];
					broadphase->Update(e, aabb);
					_SetEntityAabb(currentEntitiesAabbs, e, aabb);
				}
			}

			for (int j = 0; j < MIXED_RAY_COUNT; ++j, ++i) {
				cbRay.hasHit = false;
				if (ENABLE_VERIFICATION) {
					offsetOfPatch.push_back(hitPoints.size());
				}
				cbRay.start = aabbsToTest[i].GetCenter();
				glm::vec3 end = cbRay.end = vv[i];
				cbRay.initedVars = false;
				broadphase->IntersectRay(cbRay);
				if (ENABLE_VERIFICATION) {
					if (cbRay.hasHit) {
						ret.hitCount++;
						hitPoints.push_back(
							StartEndPoint{broadphase->GetAabb(cbRay.hitEntity),
										  {},
										  cbRay.start,
										  cbRay.end,
										  cbRay.hitPoint,
										  cbRay.cutFactor,
										  cbRay.hitEntity,
										  false});
					} else {
						hitPoints.push_back(StartEndPoint{
							{}, {}, cbRay.start, end, {}, -1, 0, false});
					}
				} else {
					if (cbRay.hasHit) {
						ret.hitCount++;
						static thread_local volatile uint64_t HOLDER_ENT = 0;
						HOLDER_ENT += cbRay.hitEntity;
					}
				}
			}
		}

		ret.nodesTestedCount = cbRay.nodesTestedCount + cbAabb.nodesTestedCount;
		ret.testedCount = cbRay.testedCount + cbAabb.testedCount;

		return ret;
	} break;
	}
	return ret;
}

std::vector<spp::Aabb> globalAabbs;
void Test(std::vector<spp::BroadphaseBase<spp::Aabb, EntityType, uint32_t, 0> *> broadphases, size_t testsCount,
		  TestType testType)
{
	broadphasePerformances.resize(broadphases.size(), {0, 0});
	TEST_RANDOM_SEED = mt();
	totalErrorsInBroadphase.resize(broadphases.size(), 0);

	std::uniform_real_distribution<float> distPos(-520, 520);
	std::uniform_real_distribution<float> distSize(pow(2.0 / 16.0, 1.0 / 3.0),
												   1.0);
	std::vector<spp::Aabb> &aabbs = globalAabbs;
	{
		size_t i = aabbs.size();
		aabbs.resize(testsCount);
		for (; i < aabbs.size(); ++i) {
			spp::Aabb &aabb = aabbs[i];
			glm::vec3 p = {distPos(mt), distPos(mt) / 8.0f, distPos(mt)};
			glm::vec3 s = {distSize(mt), distSize(mt), distSize(mt)};
			s = s * s * 16.0f;
			aabb = {p, p + s};
		}
	}

	static std::vector<std::vector<StartEndPoint>> hitPoints;
	static std::vector<std::vector<uint64_t>> offsetOfPatch;
	offsetOfPatch.resize(broadphases.size());
	hitPoints.resize(broadphases.size());
	for (int i = 0; i < broadphases.size(); ++i) {
		offsetOfPatch[i].clear();
		hitPoints[i].clear();
		if (ENABLE_VERIFICATION) {
			hitPoints[i].reserve(testsCount * 30);
			offsetOfPatch[i].reserve(testsCount);
			size_t S = (80000000 * 4) / broadphases.size();
			for (int j = 0; j < S && j < hitPoints[i].capacity(); j += 50) {
				hitPoints[i].data()[j] = {};
			}
		}
		auto &it = broadphases[i];
		size_t tC = aabbs.size();
		auto beg = std::chrono::steady_clock::now();
		auto vec = SingleTest(it, aabbs, tC, offsetOfPatch[i], testType,
							  hitPoints[i], currentEntitiesAabbs[i]);
		auto end = std::chrono::steady_clock::now();
		if (ENABLE_VERIFICATION) {
			for (size_t j = 0; j < offsetOfPatch[i].size(); ++j) {
				size_t of = offsetOfPatch[i][j];
				size_t en = hitPoints[i].size();
				if (j + 1 < offsetOfPatch[i].size()) {
					en = offsetOfPatch[i][j + 1];
				}
				std::sort(hitPoints[i].begin() + of, hitPoints[i].begin() + en);
			}
		}
		auto diff = end - beg;
		int64_t ns =
			std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(diff)
				.count();
		double us = double(ns) / 1000.0;
		broadphasePerformances[i].first += us;
		broadphasePerformances[i].second += tC;
		if (BENCHMARK == false || disable_benchmark_report == false) {
			printf("intersection test [count: %lu]: %8.3f us/op",
				   tC, us / double(tC));
			printf("    \t   nodesTested: %11lu,   testedCount: %10lu     "
				   "[count = %lu]    %s \n",
				   vec.nodesTestedCount, vec.testedCount, vec.hitCount,
				   it->GetName());
			fflush(stdout);
		}
	}

	if (BENCHMARK == false || disable_benchmark_report == false) {
		printf("Number of entities %i\n", broadphases[0]->GetCount());
	}

	if (ENABLE_VERIFICATION == false) {
		return;
	}

	printf("\n");
	for (int i = 1; i < broadphases.size(); ++i) {
		size_t cardinalErrors = 0;
		{
			auto &bp0 = *(broadphases[0]);
			auto &bpi = *(broadphases[i]);
			auto it0 = bp0.RestartIterator();
			auto iti = bpi.RestartIterator();

			if (bp0.GetCount() != bpi.GetCount()) {
				printf("0. CARDINAL ERROR: DIFFERENT NUMBER OF ELEMENTS IN "
					   "BROADPHASES: %i != %i\n",
					   bp0.GetCount(), bpi.GetCount());
				++cardinalErrors;
			}

			int I = 0;
			for (; it0->Valid(); it0->Next()) {
				if (bpi.Exists(it0->entity) == false) {
					++cardinalErrors;
					if (I < 10) {
						printf(
							"1. CARDINAL ERROR: ENTITY DOES NOT EXIST: %lu\n",
							it0->entity);
					}
					++I;
				} else {
					spp::Aabb aabb0 = it0->aabb;
					spp::Aabb aabbi = bpi.GetAabb(it0->entity);
					glm::vec3 a = aabb0.min - aabbi.min;
					glm::vec3 b = aabb0.max - aabbi.max;
					float sum = glm::length(a) + glm::length(b);
					if (sum > 0.1) {
						++cardinalErrors;
						if (I < 10) {
							printf("2. CARDINAL ERROR: ENTITY AABBS DOES NOT "
								   "MATCH: %lu (sum diff dist corners: %.1f)\n",
								   it0->entity, sum);
						}
						++I;
					}
				}
			}

			I = 0;
			for (; iti->Valid(); iti->Next()) {
				if (bp0.Exists(iti->entity) == false) {
					++cardinalErrors;
					if (I < 10) {
						printf("3. CARDINAL ERROR: ENTITY SHOULD NOT EXIST BUT "
							   "DOES: %lu\n",
							   iti->entity);
					}
					++I;
				} else {
					spp::Aabb aabb0 = bp0.GetAabb(iti->entity);
					spp::Aabb aabbi = iti->aabb;
					glm::vec3 a = aabb0.min - aabbi.min;
					glm::vec3 b = aabb0.max - aabbi.max;
					float sum = glm::length(a) + glm::length(b);
					if (sum > 0.1) {
						++cardinalErrors;
						if (I < 10) {
							printf("4. CARDINAL ERROR: ENTITY AABBS DOES NOT "
								   "MATCH: %lu (sum diff dist corners: %.1f)\n",
								   iti->entity, sum);
						}
						++I;
					}
				}
			}

			for (int j = 0; j < hitPoints[i].size(); ++j) {
				auto hp = hitPoints[i][j];
				if (hp.e <= 0) {
					continue;
				}
				if (!hp.isAabbTest && !(hp.aabb.Expanded(0.01f) && hp.point)) {
					++cardinalErrors;
					if (I < 10) {

						float near;
						float far;
						bool r = hp.aabb.SlowRayTestCenter(hp.start, hp.end,
														   near, far);

						printf("5. CARDINAL ERROR: AABB RAY TEST ERROR VALUE, "
							   "HITPOINT OUTSIDE AABB");

						glm::vec3 a, b;
						a = hp.aabb.min;
						b = hp.aabb.max;

						printf("  %7i: %7lu    :     ", j, hp.e);
						printf(
							"{{%7.2f, %7.2f, %7.2f} , {%7.2f, %7.2f, %7.2f}}",
							a.x, a.y, a.z, b.x, b.y, b.z);

						a = hp.point;
						printf("     hit point: {%7.2f, %7.2f, %7.2f}", a.x,
							   a.y, a.z);

						a = hp.start;
						printf("     start: {%7.2f, %7.2f, %7.2f}", a.x, a.y,
							   a.z);

						a = hp.end;
						printf("     end: {%7.2f, %7.2f, %7.2f}", a.x, a.y,
							   a.z);

						printf("     dist: %11.6f", hp.n);

						printf("     second result %s     near: %f   far: %f",
							   r ? "HIT" : "MISS", near, far);

						printf("\n");
						++I;
					}
				} else if (hp.isAabbTest &&
						   (!(hp.aabb.Expanded(0.01f) && hp.aabb2))) {
					++cardinalErrors;
					if (I < 10) {
						printf("6. CARDINAL ERROR: AABB TEST ERROR VALUE, "
							   "AABBS DOES NOT INTERSECT");

						glm::vec3 a, b;
						a = hp.aabb.min;
						b = hp.aabb.max;

						printf("  %7i: %7lu    :     ", j, hp.e);
						printf(
							"{{%7.2f, %7.2f, %7.2f} , {%7.2f, %7.2f, %7.2f}}",
							a.x, a.y, a.z, b.x, b.y, b.z);

						a = hp.aabb2.min;
						b = hp.aabb2.max;
						printf("     aabb2: {{%7.2f, %7.2f, %7.2f} , {%7.2f, "
							   "%7.2f, %7.2f}}",
							   a.x, a.y, a.z, b.x, b.y, b.z);

						printf("\n");
						++I;
					}
				}
			}
		}

		size_t errs = 0;
		int JJ = 0;

		for (size_t patch = 0; patch < offsetOfPatch[i].size(); ++patch) {
			const size_t patchStarti = offsetOfPatch[i][patch];
			const size_t patchStart0 = offsetOfPatch[0][patch];
			const size_t patchEndi = patch + 1 < offsetOfPatch[i].size()
										 ? offsetOfPatch[i][patch + 1]
										 : hitPoints[i].size();
			const size_t patchEnd0 = patch + 1 < offsetOfPatch[0].size()
										 ? offsetOfPatch[0][patch + 1]
										 : hitPoints[0].size();
			for (size_t entry = patchStarti; entry < patchEndi; ++entry) {
				const size_t ji = entry;
				auto pi = hitPoints[i][ji];
				auto p0 = StartEndPoint{};

				if ((patchEnd0 - patchStart0) == 1 &&
					(patchEndi - patchStarti) == 1) {
					p0 = hitPoints[0][patchStart0];
				} else {
					auto start = hitPoints[0].begin() + patchStart0;
					auto end = hitPoints[0].begin() + patchEnd0;
					const static auto comp =
						+[](StartEndPoint a, StartEndPoint b) -> bool {
						return a.e < b.e;
					};
					auto it = std::lower_bound(start, end, pi, comp);
					if (it != end) {
						p0 = *it;
						if (p0.e != pi.e) {
							continue;
						}
					} else {
						continue;
					}
				}

				if (p0.e <= 0 && pi.e <= 0) {
					continue;
				}

				if (pi.isAabbTest == false && p0.isAabbTest == false) {
					spp::Aabb aabb0 = p0.aabb.Expanded(0.0025);
					spp::Aabb aabbi = pi.aabb.Expanded(0.0025);

					bool er = true;
					if (p0.e && pi.e) {
						if ((aabb0 && p0.point) && (aabbi && p0.point) &&
							(aabb0 && pi.point) && (aabbi && pi.point) &&
							(aabb0 && aabbi)) {
							if (glm::length(pi.point - p0.point) < 0.01) {
								if (fabs(p0.n - pi.n) < 0.001) {
									er = false;
								}
							}
						}
					}

					if (er == true) {
						aabb0 = p0.aabb;
						aabbi = pi.aabb;

						++JJ;
						++errs;
						if (JJ < 10) {
							glm::vec3 a, b, c, d;
							a = aabb0.min;
							b = aabb0.max;
							c = aabbi.min;
							d = aabbi.max;
							printf("ray (%li<>%li)   %7ld: %7lu == %7lu  ", 
									patchEndi-patchStarti, patchEnd0-patchStart0,
									ji, p0.e,
								   pi.e);
							printf("{{%7.2f, %7.2f, %7.2f} , {%7.2f, %7.2f, "
								   "%7.2f}} <-> {{%7.2f, %7.2f, %7.2f} , "
								   "{%7.2f, "
								   "%7.2f, %7.2f}}",
								   a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z,
								   d.x, d.y, d.z);

							a = p0.point;
							b = pi.point;
							printf("     hit points: {%7.2f, %7.2f, %7.2f} "
								   "<-> "
								   "{%7.2f, %7.2f, %7.2f}",
								   a.x, a.y, a.z, b.x, b.y, b.z);

							a = p0.start;
							b = pi.start;
							printf("     start: {%7.2f, %7.2f, %7.2f}  <->  {%7.2f, %7.2f, %7.2f}", a.x,
								   a.y, a.z, b.x, b.y, b.z);

							a = p0.end;
							b = pi.end;
							printf("     end: {%7.2f, %7.2f, %7.2f}  <->  {%7.2f, %7.2f, %7.2f}", a.x,
								   a.y, a.z, b.x, b.y, b.z);

							printf("     dist: %11.6f <-> %11.6f", p0.n, pi.n);

							printf("\n");
						}
					}
				} else {
					// TODO: check aabb equality

					bool er = false;

					if (p0.e != pi.e) {
						er = true;
					} else {
						if (glm::length(pi.aabb.min - p0.aabb.min) > 0.001 &&
							glm::length(pi.aabb.max - p0.aabb.max) > 0.001) {
							er = false;
						}
					}

					if (er == true) {
						spp::Aabb aabb0 = p0.aabb;
						spp::Aabb aabbi = pi.aabb;

						++JJ;
						++errs;
						glm::vec3 a, b, c, d;
						a = aabb0.min;
						b = aabb0.max;
						c = aabbi.min;
						d = aabbi.max;
						if (JJ < 10) {
							printf("aabb  %7ld: %7lu == %7lu  ", ji, p0.e,
								   pi.e);
							printf("{{%7.2f, %7.2f, %7.2f} , {%7.2f, %7.2f, "
								   "%7.2f}} <-> {{%7.2f, %7.2f, %7.2f} , "
								   "{%7.2f, "
								   "%7.2f, %7.2f}}",
								   a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z,
								   d.x, d.y, d.z);

							a = p0.aabb2.min;
							b = p0.aabb2.max;
							printf("      test: {{%7.2f, %7.2f, %7.2f} , "
								   "{%7.2f, %7.2f, %7.2f}}",
								   a.x, a.y, a.z, b.x, b.y, b.z);

							printf("\n");
						}
					}
				}
			}

			std::map<EntityType, size_t> e0, ei;
			for (size_t entry = patchStarti; entry < patchEndi; ++entry) {
				ei[hitPoints[i][entry].e] = entry;
			}
			for (size_t entry = patchStart0; entry < patchEnd0; ++entry) {
				e0[hitPoints[0][entry].e] = entry;
			}

			if ((patchEnd0 - patchStart0) == 1 &&
				(patchEndi - patchStarti) == 1) {
				e0.clear();
				ei.clear();
			}

			for (auto it : e0) {
				if (ei.find(it.first) == ei.end()) {
					++JJ;
					++errs;
					glm::vec3 a, b;
					auto hp = hitPoints[0][it.second];
					if (JJ < 10) {
						if (hp.isAabbTest) {
							a = hp.aabb.min;
							b = hp.aabb.max;
							printf("aabb missing       ji %7ld[%lu] (j0 "
								   "%7ld[%lu]): "
								   "%7lu  ",
								   patchStart0, patchEnd0 - patchStart0,
								   it.second, patchEndi - patchStarti, hp.e);
							printf("{{%7.2f, %7.2f, %7.2f} , {%7.2f, %7.2f, "
								   "%7.2f}}",
								   a.x, a.y, a.z, b.x, b.y, b.z);

							a = hp.aabb2.min;
							b = hp.aabb2.max;
							printf("      test: {{%7.2f, %7.2f, %7.2f} , "
								   "{%7.2f, %7.2f, %7.2f}}",
								   a.x, a.y, a.z, b.x, b.y, b.z);

							printf("\n");
						} else {
							glm::vec3 a, b;
							a = hp.aabb.min;
							b = hp.aabb.max;
							printf("ray missing        ji %7ld[%lu] (j0 "
								   "%7ld[%lu]): "
								   "%7lu  ",
								   patchStart0, patchEnd0 - patchStart0,
								   it.second, patchEndi - patchStarti, hp.e);
							printf("{{%7.2f, %7.2f, %7.2f} , {%7.2f, %7.2f, "
								   "%7.2f}}",
								   a.x, a.y, a.z, b.x, b.y, b.z);

							a = hp.point;
							printf("     hit point: {%7.2f, %7.2f, %7.2f}", a.x,
								   a.y, a.z);

							a = hp.start;
							printf("     start: {%7.2f, %7.2f, %7.2f}", a.x,
								   a.y, a.z);

							a = hp.end;
							printf("     end: {%7.2f, %7.2f, %7.2f}", a.x, a.y,
								   a.z);

							printf("     dist: %11.9f", hp.n);

							printf("\n");
						}
					}
				}
			}

			for (auto it : ei) {
				if (e0.find(it.first) == e0.end()) {
					++JJ;
					++errs;
					glm::vec3 a, b;
					auto hp = hitPoints[i][it.second];
					if (JJ < 10) {
						if (hp.isAabbTest) {
							a = hp.aabb.min;
							b = hp.aabb.max;
							printf("aabb shouldn't be  ji %7ld[%lu] (j0 "
								   "%7ld[%lu]): "
								   "%7lu  ",
								   patchStart0, patchEnd0 - patchStart0,
								   it.second, patchEndi - patchStarti, hp.e);
							printf("{{%7.2f, %7.2f, %7.2f} , {%7.2f, %7.2f, "
								   "%7.2f}}",
								   a.x, a.y, a.z, b.x, b.y, b.z);

							a = hp.aabb2.min;
							b = hp.aabb2.max;
							printf("      test: {{%7.2f, %7.2f, %7.2f} , "
								   "{%7.2f, %7.2f, %7.2f}}",
								   a.x, a.y, a.z, b.x, b.y, b.z);

							printf("\n");
						} else {
							glm::vec3 a, b;
							a = hp.aabb.min;
							b = hp.aabb.max;
							printf("ray shouldn't be   ji %7ld[%lu] (j0 "
								   "%7ld[%lu]): "
								   "%7lu  ",
								   patchStart0, patchEnd0 - patchStart0,
								   it.second, patchEndi - patchStarti, hp.e);
							printf("{{%7.2f, %7.2f, %7.2f} , {%7.2f, %7.2f, "
								   "%7.2f}}",
								   a.x, a.y, a.z, b.x, b.y, b.z);

							a = hp.point;
							printf("     hit point: {%7.2f, %7.2f, %7.2f}", a.x,
								   a.y, a.z);

							a = hp.start;
							printf("     start: {%7.2f, %7.2f, %7.2f}", a.x,
								   a.y, a.z);

							a = hp.end;
							printf("     end: {%7.2f, %7.2f, %7.2f}", a.x, a.y,
								   a.z);

							printf("     dist: %11.9f", hp.n);

							printf("\n");
						}
					}
				}
			}
		}
		errs += cardinalErrors;
		totalErrorsInBroadphase[i] += errs;
		printf(" %s: (cardinal err: %lu) errors count: %lu ... %s",
			   broadphases[i]->GetName(), cardinalErrors, errs,
			   errs ? "ERRORS" : "OK");
		if (totalErrorsInBroadphase[i] > 0) {
			printf("    TOTAL ERRORS: %lu", totalErrorsInBroadphase[i]);
		}
		printf("\n");
		fflush(stdout);
	}
}

void EnqueueRebuildThreaded(std::shared_ptr<std::atomic<bool>> fin,
							std::shared_ptr<spp::BroadphaseBase<spp::Aabb, EntityType, uint32_t, 0>> dbvh,
							std::shared_ptr<void> data)
{
	static std::atomic<int> size = 0;
	static std::mutex mutex;
	static bool done = false;
	using Pair = std::pair<std::shared_ptr<std::atomic<bool>>,
						   std::shared_ptr<spp::BroadphaseBase<spp::Aabb, EntityType, uint32_t, 0>>>;
	static std::queue<Pair> queue;
	if (done == false) {
		done = true;
		std::thread thread = std::thread([]() {
			while (true) {
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
				if (size.load() > 0) {
					Pair p;
					bool has = false;
					{
						std::lock_guard lock(mutex);
						if (queue.empty() == false) {
							p = queue.front();
							queue.pop();
							has = true;
							--size;
						}
					}
					if (has && p.second.use_count() > 1) {
						// auto beg = std::chrono::steady_clock::now();
						p.second->Rebuild();
						/*
						auto end = std::chrono::steady_clock::now();
						auto diff = end - beg;
						int64_t ns =
							std::chrono::duration_cast<std::chrono::nanoseconds,
													   int64_t>(diff)
								.count();
						double us = double(ns) / 1000.0;
						if (false) {
							printf("                        "
								   "%s (%i) "
								   "Async rebuild of : %.3f us/op\n",
								   p.second->GetName(), p.second->GetCount(),
								   us / double(TOTAL_AABB_MOVEMENTS));
							fflush(stdout);
						}
						*/

						p.first->store(true);
					}
				}
			}
		});
		thread.detach();
	}
	if (fin.get() != nullptr) {
		std::lock_guard lock(mutex);
		queue.push({fin, dbvh});
		++size;
	}
}

int main(int argc, char **argv)
{
	const auto START_MAIN = std::chrono::steady_clock::now();
	EnqueueRebuildThreaded(nullptr, nullptr, nullptr);

	bool enablePrepass = true;

	bool customizeStructures = false;

	size_t mixedTestsCount = 1;

	for (int i = 1; i < argc; ++i) {
		if (std::string(argv[i]).starts_with("-disable-prepass")) {
			enablePrepass = false;
		} else if (std::string(argv[i]).starts_with("-disable-verification")) {
			ENABLE_VERIFICATION = false;
		} else if (std::string(argv[i]).starts_with("-benchmark")) {
			ENABLE_VERIFICATION = false;
			BENCHMARK = true;
		} else if (std::string(argv[i]).starts_with("-total-entities=")) {
			TOTAL_ENTITIES = atoi(argv[i] + strlen("-total-entities="));
			MAX_ENTITIES = TOTAL_ENTITIES + ADDITIONAL_ENTITIES;
		} else if (std::string(argv[i]).starts_with("-additional-entities=")) {
			ADDITIONAL_ENTITIES =
				atoi(argv[i] + strlen("-additional-entities="));
			MAX_ENTITIES = TOTAL_ENTITIES + ADDITIONAL_ENTITIES;
		} else if (std::string(argv[i]).starts_with("-aabb-tests=")) {
			TOTAL_AABB_TESTS = atoll(argv[i] + strlen("-aabb-tests="));
		} else if (std::string(argv[i]).starts_with("-aabb-movements=")) {
			TOTAL_AABB_MOVEMENTS = atoll(argv[i] + strlen("-aabb-movements="));
		} else if (std::string(argv[i]).starts_with("-mixed-tests=")) {
			TOTAL_MOVES_AND_TESTS = atoll(argv[i] + strlen("-mixed-tests="));
		} else if (std::string(argv[i]).starts_with("-moving-entities=")) {
			MAX_MOVING_ENTITIES = atoll(argv[i] + strlen("-moving-entities="));
		} else if (argv[i][0] != '-') {
			customizeStructures = true;
		} else if (std::string(argv[i]).starts_with("-help")) {
			printf("Available options:\n"
				   "\t-disable-prepass\n"
				   "\t-disable-verification\n"
				   "\t-total-entities=\n"
				   "\t-additional-entities=\n"
				   "\t-aabb-tests=\n"
				   "\t-aabb-movements=\n"
				   "\t-mixed-tests=\n"
				   "\t-moving-entities=\n"
				   "\t-random-seed=random\n"
				   "\t-random-seed=$NUMBER\n"
				   "\t-mixed-tests-count=\n"
				   "\t-mixed-aabb-count=\n"
				   "\t-mixed-ray-count=\n"
				   "\t-mixed-update-count=\n"
				   "\t-benchmark\n"
				   "\tBF              - BruteForce\n"
				   "\tBVH             - BvhMedianSplitHeap\n"
				   "\tBVH1            - BvhMedianSplitHeap1\n"
				   "\tDBVT            - Rewritten btDbvt from Bullet\n"
				   "\tDBVH            - Dbvh (DynamicBoundingVolumeHierarchy)\n"
				   "\tBTDBVH          - BulletDbvh (Bullet dbvh - two stages)\n"
				   "\tBTDBVT          - BulletDbvt (Bullet dbvt one stage)\n"
				   "\tTSH_BF          - ThreeStageDbvh BvhMedian + BruteForce\n"
				   "\tTSH_BTDBVT      - ThreeStageDbvh BvhMedian + BulletDbvt\n"
				   "\tTSH_BTDBVT3     - ThreeStageDbvh BulletDbvt + BulletDbvt\n"
				   "\tTSH_DBVH        - ThreeStageDbvh BvhMedian + Dbvh\n"
				   "\tTSH_DBVT        - ThreeStageDbvh BvhMedian + Dbvt\n"
				   "\tTSH_DBVT1       - ThreeStageDbvh BvhMedian1 + Dbvt\n"
				   "\tTSH_BVH         - ThreeStageDbvh BvhMedian + BvhMedian\n"
				   "\tTSH_BVH1        - ThreeStageDbvh BvhMedian1 + BvhMedian1 (no schedule)\n"
				   "\tTSH_DBVT2       - ThreeStageDbvh Dbvt + Dbvt (no schedule)\n"
				   "\tTSH_BTDBVT2     - ThreeStageDbvh BulletDbvt + BulletDbvt (no schedule)\n"
				   "\tTSH_1_BVH1_DBVT - ThreeStageDbvh BvhMedian1 + BvhMedian1 (no schedule)\n"
				   "\tHLO             - HashedLooseOctree\n"
				   "\tLO              - LooseOctree\n");
			return 0;
		} else if (std::string(argv[i]).starts_with("-random-seed=random")) {
			std::random_device rd;
			mt = std::mt19937_64(rd());
		} else if (std::string(argv[i]).starts_with("-random-seed=")) {
			mt = std::mt19937_64(atoll(argv[i] + strlen("-random-seed=")));
		} else if (std::string(argv[i]).starts_with("-mixed-tests-count=")) {
			mixedTestsCount = atoll(argv[i] + strlen("-mixed-tests-count="));
		} else if (std::string(argv[i]).starts_with("-mixed-aabb-count=")) {
			MIXED_AABB_COUNT = atoll(argv[i] + strlen("-mixed-aabb-count="));
		} else if (std::string(argv[i]).starts_with("-mixed-ray-count=")) {
			MIXED_RAY_COUNT = atoll(argv[i] + strlen("-mixed-ray-count="));
		} else if (std::string(argv[i]).starts_with("-mixed-update-count=")) {
			MIXED_UPDATE_COUNT =
				atoll(argv[i] + strlen("-mixed-update-count="));
		}
	}

	std::vector<EntityData> entities;
	globalEntityData = &entities;
	entities.resize(TOTAL_ENTITIES);
	std::uniform_real_distribution<float> distPos(-10, 10);
	std::uniform_real_distribution<float> distSize(0.4, 5);
	EntityType id = 1;
	for (auto &e : entities) {
		e.aabb.min = {distPos(mt), distPos(mt) / 8.0f, distPos(mt)};
		e.aabb.max = {distSize(mt), distSize(mt) * 2, distSize(mt)};
		e.aabb.max += e.aabb.min;
		e.id = id;
		++id;
	}
	entities[0].aabb = {{-400, -10, -400}, {400, 100, 400}};
	entities[1].aabb = {{150, 0, 150}, {200, 10, 200}};
	for (int i = 2; i < 50 && i < entities.size(); ++i) {
		auto &e = entities[i];
		e.aabb.min = {distPos(mt), distPos(mt) / 16.0f, distPos(mt)};
		e.aabb.max = {distSize(mt) * 6, distSize(mt) * 2, distSize(mt) * 6};
		e.aabb.max += e.aabb.min;
	}

	std::vector<spp::BroadphaseBase<spp::Aabb, EntityType, uint32_t, 0> *> broadphases;

	if (customizeStructures) {
		broadphases.clear();
		for (int i = 1; i < argc; ++i) {
			char *str = argv[i];
			if (false) {
			} else if (strcmp(str, "BF") == false) {
				broadphases.push_back(new spp::BruteForce<spp::Aabb, EntityType, uint32_t, 0>());
			} else if (strcmp(str, "BVH") == false) {
				spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0> *bvh;
				bvh = new spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0>(TOTAL_ENTITIES);
				broadphases.push_back(bvh);
			} else if (strcmp(str, "BVH1") == false) {
				spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0, 1> *bvh;
				bvh = new spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0, 1>(TOTAL_ENTITIES);
				broadphases.push_back(bvh);
			} else if (strcmp(str, "DBVT") == false) {
				broadphases.push_back(new spp::Dbvt<spp::Aabb, EntityType, uint32_t, 0, uint32_t>);
			} else if (strcmp(str, "DBVH") == false) {
				broadphases.push_back(new spp::Dbvh<spp::Aabb, EntityType, uint32_t, 0>);
			} else if (strcmp(str, "BTDBVH") == false) {
				broadphases.push_back(new spp::BulletDbvh<spp::Aabb, EntityType, uint32_t, 0>);
			} else if (strcmp(str, "BTDBVT") == false) {
				broadphases.push_back(new spp::BulletDbvt<spp::Aabb, EntityType, uint32_t, 0>);
			} else if (strcmp(str, "TSH_BF") == false) {
				spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0> *tsdbvh = new spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0>(
					std::make_shared<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0>>(TOTAL_ENTITIES),
					std::make_shared<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0>>(TOTAL_ENTITIES),
					std::make_unique<spp::BruteForce<spp::Aabb, EntityType, uint32_t, 0>>());
				tsdbvh->SetRebuildSchedulerFunction(EnqueueRebuildThreaded);
				broadphases.push_back(tsdbvh);
			} else if (strcmp(str, "TSH_BTDBVT") == false) {
				spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0> *tsdbvh = new spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0>(
					std::make_shared<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0>>(TOTAL_ENTITIES),
					std::make_shared<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0>>(TOTAL_ENTITIES),
					std::make_unique<spp::BulletDbvt<spp::Aabb, EntityType, uint32_t, 0>>());
				tsdbvh->SetRebuildSchedulerFunction(EnqueueRebuildThreaded);
				broadphases.push_back(tsdbvh);
			} else if (strcmp(str, "TSH_BTDBVT3") == false) {
				spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0> *tsdbvh = new spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0>(
					std::make_shared<spp::BulletDbvt<spp::Aabb, EntityType, uint32_t, 0>>(),
					std::make_shared<spp::BulletDbvt<spp::Aabb, EntityType, uint32_t, 0>>(),
					std::make_unique<spp::BulletDbvt<spp::Aabb, EntityType, uint32_t, 0>>());
				tsdbvh->SetRebuildSchedulerFunction(EnqueueRebuildThreaded);
				broadphases.push_back(tsdbvh);
			} else if (strcmp(str, "TSH_DBVH") == false) {
				spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0> *tsdbvh = new spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0>(
					std::make_shared<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0>>(TOTAL_ENTITIES),
					std::make_shared<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0>>(TOTAL_ENTITIES),
					std::make_unique<spp::Dbvh<spp::Aabb, EntityType, uint32_t, 0>>());
				tsdbvh->SetRebuildSchedulerFunction(EnqueueRebuildThreaded);
				broadphases.push_back(tsdbvh);
			} else if (strcmp(str, "TSH_DBVT") == false) {
				spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0> *tsdbvh = new spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0>(
					std::make_shared<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0>>(TOTAL_ENTITIES),
					std::make_shared<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0>>(TOTAL_ENTITIES),
					std::make_unique<spp::Dbvt<spp::Aabb, EntityType, uint32_t, 0, uint32_t>>());
				tsdbvh->SetRebuildSchedulerFunction(EnqueueRebuildThreaded);
				broadphases.push_back(tsdbvh);
			} else if (strcmp(str, "TSH_DBVT1") == false) {
				spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0> *tsdbvh = new spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0>(
					std::make_shared<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0, 1>>(TOTAL_ENTITIES),
					std::make_shared<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0, 1>>(TOTAL_ENTITIES),
					std::make_unique<spp::Dbvt<spp::Aabb, EntityType, uint32_t, 0, uint32_t>>());
				tsdbvh->SetRebuildSchedulerFunction(EnqueueRebuildThreaded);
				broadphases.push_back(tsdbvh);
			} else if (strcmp(str, "TSH_BVH") == false) {
				spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0> *tsdbvh = new spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0>(
					std::make_shared<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0>>(TOTAL_ENTITIES),
					nullptr,//std::make_shared<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0>>(TOTAL_ENTITIES),
					std::make_unique<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0>>(0));
				broadphases.push_back(tsdbvh);
			} else if (strcmp(str, "TSH_BVH1") == false) {
				spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0> *tsdbvh = new spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0>(
					std::make_shared<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0, 1>>(TOTAL_ENTITIES),
					nullptr,//std::make_shared<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0, 1>>(TOTAL_ENTITIES),
					std::make_unique<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0, 1>>(0));
				broadphases.push_back(tsdbvh);
			} else if (strcmp(str, "TSH_1_BVH1_DBVT") == false) {
				spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0> *tsdbvh = new spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0>(
					std::make_shared<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0, 1>>(TOTAL_ENTITIES),
					nullptr,//std::make_shared<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0, 1>>(TOTAL_ENTITIES),
					std::make_unique<spp::Dbvt<spp::Aabb, EntityType, uint32_t, 0, uint32_t>>());
				broadphases.push_back(tsdbvh);
			} else if (strcmp(str, "TSH_DBVT2") == false) {
				spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0> *tsdbvh = new spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0>(
					std::make_shared<spp::Dbvt<spp::Aabb, EntityType, uint32_t, 0, uint32_t>>(),
					nullptr,//std::make_shared<spp::BulletDbvt<spp::Aabb, EntityType, uint32_t, 0>>(),
					std::make_unique<spp::Dbvt<spp::Aabb, EntityType, uint32_t, 0, uint32_t>>());
				broadphases.push_back(tsdbvh);
			} else if (strcmp(str, "TSH_BTDBVT2") == false) {
				spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0> *tsdbvh = new spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0>(
					std::make_shared<spp::BulletDbvt<spp::Aabb, EntityType, uint32_t, 0>>(),
					nullptr,//std::make_shared<spp::BulletDbvt<spp::Aabb, EntityType, uint32_t, 0>>(),
					std::make_unique<spp::BulletDbvt<spp::Aabb, EntityType, uint32_t, 0>>());
				broadphases.push_back(tsdbvh);
			} else if (strcmp(str, "HLO") == false) {
				broadphases.push_back(
					new spp::experimental::HashLooseOctree<spp::Aabb, EntityType, uint32_t, 0>(1.0, 13, 1.6));
			} else if (strcmp(str, "LO") == false) {
				broadphases.push_back(new spp::experimental::LooseOctree<spp::Aabb, EntityType, uint32_t, 0>(
					-glm::vec3(1, 1, 1) * (1024 * 16.f), 15, 1.6));
			} else if (strcmp(str, "CLO") == false) {
				printf("ChunkedLooseOctree not implemented\n");
			}
			fflush(stdout);
		}
	} else {
		spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0> *s;
		broadphases = {
			new spp::Dbvh<spp::Aabb, EntityType, uint32_t, 0>,
			(s = new spp::ThreeStageDbvh<spp::Aabb, EntityType, uint32_t, 0>(
				 std::make_shared<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0>>(TOTAL_ENTITIES),
				 std::make_shared<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0>>(TOTAL_ENTITIES),
				 std::make_unique<spp::BulletDbvt<spp::Aabb, EntityType, uint32_t, 0>>()),
			 s->SetRebuildSchedulerFunction(EnqueueRebuildThreaded), s)};
	}
	
	currentEntitiesAabbs.resize(broadphases.size());

	if (enablePrepass) {

		for (int II = 0; II < broadphases.size(); ++II) {
			auto bp = broadphases[II];
			auto beg = std::chrono::steady_clock::now();
			bp->StartFastAdding();
			for (const auto &e : entities) {
				assert(e.id > 0);
				bp->Add(e.id, e.aabb, e.mask);
				_SetEntityAabb(currentEntitiesAabbs[II], e.id, e.aabb);
			}
			bp->StopFastAdding();
			auto end = std::chrono::steady_clock::now();
			auto diff = end - beg;
			int64_t ns =
				std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(
					diff)
					.count();
			double us = double(ns) / 1000.0;
			printf("adding time: %10.3f us      %s\n", us, bp->GetName());
			fflush(stdout);
		}
		printf("\n");

		for (auto bp : broadphases) {
			auto beg = std::chrono::steady_clock::now();
			bp->Rebuild();
			auto end = std::chrono::steady_clock::now();
			auto diff = end - beg;
			int64_t ns =
				std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(
					diff)
					.count();
			double us = double(ns) / 1000.0;
			printf("build time: %10.3f us      %s\n", us, bp->GetName());
			fflush(stdout);
		}
		printf("\n");

		for (auto bp : broadphases) {
			printf("memory: %10lu B , %11.6f MiB,   %7.2f B/entity      %s\n",
				   bp->GetMemoryUsage(), bp->GetMemoryUsage() / (1024.0 * 1024.0),
				   (double)bp->GetMemoryUsage() / (double)bp->GetCount(), bp->GetName());
			fflush(stdout);
		}
		printf("\n");

		/*
		std::uniform_real_distribution<float> distDisp(-50, 50);
		for (int i = 0; i < 300; ++i) {
			auto &e = entities[mt() % entities.size()];
			glm::vec3 disp = {distPos(mt), distPos(mt) / 4.0f, distPos(mt)};
			e.aabb.min += disp;
			e.aabb.max += disp;
		}

		for (int II = 0; II < broadphases.size(); ++II) {
			auto bp = broadphases[II];
			auto beg = std::chrono::steady_clock::now();
			for (int i = 0; i < 300 && i < entities.size(); ++i) {
				bp->Update(entities[i].id, entities[i].aabb);
				_SetEntityAabb(currentEntitiesAabbs[II], entities[i].id,
		entities[i].aabb);
			}
			auto end = std::chrono::steady_clock::now();
			auto diff = end - beg;
			int64_t ns =
				std::chrono::duration_cast<std::chrono::nanoseconds,
		int64_t>(diff) .count(); double us = double(ns) / 1000.0; printf("%s
		update data: %.3f us\n", bp->GetName(), us); fflush(stdout);
		}
		printf("\n");
		*/

		for (auto bp : broadphases) {
			auto beg = std::chrono::steady_clock::now();
			bp->Rebuild();
			auto end = std::chrono::steady_clock::now();
			auto diff = end - beg;
			int64_t ns =
				std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(
					diff)
					.count();
			double us = double(ns) / 1000.0;
			printf("rebuild time: %10.3f us      %s\n", us, bp->GetName());
			fflush(stdout);
		}
		printf("\n");

		printf("\nAfter rebuild:\n\n");

		for (int i = 1; i <= 3; ++i) {
			printf("\n     TestType: %s\n", testTypeNames[i]);
			Test(broadphases, TOTAL_AABB_TESTS, (TestType)i);
		}

		ee.reserve(TOTAL_AABB_MOVEMENTS);
		vv.reserve(TOTAL_AABB_MOVEMENTS);
		for (size_t i = 0; i < TOTAL_AABB_MOVEMENTS; ++i) {
			ee.push_back(((mt() % MAX_MOVING_ENTITIES) % entities.size()) + 1);
			vv.push_back({distPos(mt), distPos(mt) / 4.0f, distPos(mt)});
		}

		printf("\n");

		const auto old = entities;
		for (int II = 0; II < broadphases.size(); ++II) {
			auto bp = broadphases[II];
			entities = old;
			auto beg = std::chrono::steady_clock::now();
			for (size_t i = 0; i < ee.size(); ++i) {
				spp::Aabb &a = entities[ee[i] - 1].aabb;
				a.min += vv[i];
				a.max += vv[i];
				bp->Update(ee[i], a);
				_SetEntityAabb(currentEntitiesAabbs[II], ee[i], a);
			}
			auto end = std::chrono::steady_clock::now();
			auto diff = end - beg;
			int64_t ns =
				std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(
					diff)
					.count();
			double us = double(ns) / 1000.0;
			printf("update data: %.3f us/op     %s\n",
				   us / double(TOTAL_AABB_MOVEMENTS), bp->GetName());
			fflush(stdout);
		}
		printf("\n");

		printf("\nAfter updated WITHOUT rebuild:\n\n");

		for (int i = 1; i <= 3; ++i) {
			printf("\n     TestType: %s\n", testTypeNames[i]);
			Test(broadphases, TOTAL_AABB_TESTS, (TestType)i);
		}

		printf("\nRebuild:\n\n");

		for (auto bp : broadphases) {
			double us;
			if (auto b = dynamic_cast<spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0> *>(bp)) {
				std::multimap<double, int32_t, std::greater<double>> timestage;
				std::vector<char> bytesCacheTrashing;
				bytesCacheTrashing.resize(1024);
				spp::BvhMedianSplitHeap<spp::Aabb, EntityType, uint32_t, 0>::RebuildProgress pr;
				while (pr.done == false) {
					{
						char *_ptr = bytesCacheTrashing.data();
						char *volatile ptr = _ptr;
						for (int i = 0; i < bytesCacheTrashing.size();
							 i += 64) {
							ptr[i] = i;
						}
					}
					const int32_t s = pr.stage;
					auto beg = std::chrono::steady_clock::now();
					b->RebuildStep(pr);
					auto end = std::chrono::steady_clock::now();
					auto diff = end - beg;
					int64_t ns =
						std::chrono::duration_cast<std::chrono::nanoseconds,
												   int64_t>(diff)
							.count();
					us = double(ns) / 1000.0;
					timestage.insert({us, s});
				}
				int i = 0;
				double sum = 0;
				std::vector<std::pair<double, int32_t>> tst(timestage.begin(),
															timestage.end());
				for (auto it = tst.begin(); i < tst.size();
					 i = ((i * 14 / 13) + 1), it = tst.begin() + i) {
					sum += it->first;
					printf("times[%i:%i]: %.3f us\n", i, it->second, it->first);
				}
				printf("times.size = %lu\n", timestage.size());
				us = sum / timestage.size();
			} else {
				auto beg = std::chrono::steady_clock::now();
				bp->Rebuild();
				auto end = std::chrono::steady_clock::now();
				auto diff = end - beg;
				int64_t ns =
					std::chrono::duration_cast<std::chrono::nanoseconds,
											   int64_t>(diff)
						.count();
				us = double(ns) / 1000.0;
			}
			printf("rebuild: %10.3f us     %s\n", us, bp->GetName());
			fflush(stdout);
		}
		printf("\n");

		printf("\nAfter updated WITH rebuild:\n\n");

		for (int i = 1; i <= 3; ++i) {
			printf("\n     TestType: %s\n", testTypeNames[i]);
			Test(broadphases, TOTAL_AABB_TESTS, (TestType)i);
		}

		printf("\nAfter reconstruct and full rebuild:\n");

		for (auto bp : broadphases) {
			auto beg = std::chrono::steady_clock::now();
			bp->Clear();
			auto end = std::chrono::steady_clock::now();
			auto diff = end - beg;
			int64_t ns =
				std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(
					diff)
					.count();
			double us = double(ns) / 1000.0;
			printf("%s clearing time: %.3f us\n", bp->GetName(), us);
			fflush(stdout);
		}

		printf("\n");
	}

	for (int II = 0; II < broadphases.size(); ++II) {
		auto bp = broadphases[II];
		auto beg = std::chrono::steady_clock::now();
		bp->StartFastAdding();
		for (const auto &e : entities) {
			assert(e.id > 0);
			if (bp->Exists(e.id) == false) {
				bp->Add(e.id, e.aabb, e.mask);
				_SetEntityAabb(currentEntitiesAabbs[II], e.id, e.aabb);
			}
		}
		bp->StopFastAdding();
		auto end = std::chrono::steady_clock::now();
		auto diff = end - beg;
		int64_t ns =
			std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(diff)
				.count();
		double us = double(ns) / 1000.0;
		printf("adding time: %10.3f us       %s\n", us, bp->GetName());
		fflush(stdout);
	}
	printf("\n");

	for (auto bp : broadphases) {
		auto beg = std::chrono::steady_clock::now();
		bp->Rebuild();
		auto end = std::chrono::steady_clock::now();
		auto diff = end - beg;
		int64_t ns =
			std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(diff)
				.count();
		double us = double(ns) / 1000.0;
		printf("build: %10.3f us      %s\n", us, bp->GetName());
		fflush(stdout);
	}
	printf("\n");

	for (auto bp : broadphases) {
		printf("memory: %10lu B , %11.6f MiB,   %7.2f B/entity      %s\n",
			   bp->GetMemoryUsage(), bp->GetMemoryUsage() / (1024.0 * 1024.0),
			   (double)bp->GetMemoryUsage() / (double)bp->GetCount(), bp->GetName());
		fflush(stdout);
	}
	printf("\n");

	if (enablePrepass) {
		for (int i = 1; i <= 3; ++i) {
			printf("\n     TestType: %s\n", testTypeNames[i]);
			Test(broadphases, TOTAL_AABB_TESTS, (TestType)i);
		}
	}

	printf("\nMore realistic dynamic movement entagled with tests:\n");

	auto last = std::chrono::steady_clock::now();
	double timeDiff = 10.0;
	disable_benchmark_report = false;
	for (size_t i = 0; i < mixedTestsCount; ++i) {
		if (BENCHMARK == false || disable_benchmark_report == false || i == 0 ||
			i + 1 == mixedTestsCount) {
			disable_benchmark_report = false;
		}
		ee.resize(TOTAL_MOVES_AND_TESTS);
		vv.resize(TOTAL_MOVES_AND_TESTS);
		for (size_t i = 0; i < TOTAL_MOVES_AND_TESTS; ++i) {
			ee[i] = (((mt() % MAX_MOVING_ENTITIES) % entities.size()) + 1);
			vv[i] = {distPos(mt), distPos(mt) / 4.0f, distPos(mt)};
		}

		Test(broadphases, TOTAL_MOVES_AND_TESTS, TEST_MIXED);

		if (BENCHMARK == false || disable_benchmark_report == false) {
			printf("\n");
			for (auto bp : broadphases) {
				printf("memory: %10lu B , %11.6f MiB,   %7.2f B/entity      %s\n",
					   bp->GetMemoryUsage(),
					   bp->GetMemoryUsage() / (1024.0 * 1024.0),
					   (double)bp->GetMemoryUsage() / (double)bp->GetCount(), bp->GetName());
			}
			printf("\n");
			fflush(stdout);
		}

		if (BENCHMARK == false || disable_benchmark_report == false) {
			const auto now = std::chrono::steady_clock::now();
			auto diff = now - START_MAIN;
			int64_t ns =
				std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(
					diff)
					.count();
			double seconds = double(ns) / 1000'000'000.0;
			double minutes = seconds / 60.0;
			double hours = minutes / 60.0;
			if (seconds < 200.0) {
				printf("\n (elapsed: %8.2f seconds)", seconds);
			} else if (minutes < 200.0) {
				printf("\n (elapsed: %8.2f minutes)", minutes);
			} else {
				printf("\n (elapsed: %8.2f hours)", hours);
			}
			printf("   test %lu:/%lu  (%.2f%%)\n", i + 1, mixedTestsCount,
				   double((i + 1) * 100) / (double)mixedTestsCount);
		}

		disable_benchmark_report = true;
		if (last + std::chrono::seconds((int64_t)timeDiff) <
			std::chrono::steady_clock::now()) {
			last = std::chrono::steady_clock::now();
			disable_benchmark_report = false;
			timeDiff *= 1.1;
		}
	}

	printf("\n");
	for (int i = 0; i < broadphases.size(); ++i) {
		auto bp = broadphases[i];
		double usPerOp =
			broadphasePerformances[i].first / broadphasePerformances[i].second;
		printf("average ops/usec: %10.3f us/op     %s\n", usPerOp, bp->GetName());
	}
	fflush(stdout);

	for (auto bp : broadphases) {
		bp->Clear();
		delete bp;
	}

	return 0;
}
