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
#include "../include/spatial_partitioning/ThreeStageDbvh.hpp"

int32_t TOTAL_ENTITIES = 1000000;
int32_t ADDITIONAL_ENTITIES = 10000;
int32_t MAX_ENTITIES = TOTAL_ENTITIES + ADDITIONAL_ENTITIES;
size_t TOTAL_AABB_TESTS = 200000;
size_t TOTAL_AABB_MOVEMENTS = 1000000;
size_t TOTAL_MOVES_AND_TESTS = 1000000;
size_t MAX_MOVING_ENTITIES = 2500;
size_t BRUTE_FROCE_TESTS_COUNT_DIVISOR = 1;

std::mt19937_64 mt(12345);

enum TestType {
	TEST_AABB = 1,
	TEST_RAY_FIRST = 2,
	TEST_RAY = 3,
	TEST_MIXED = 4,
};

const char *testTypeNames[] = {"NULL-NONE", "TEST_AABB", "TEST_RAY_FIRST",
							   "TEST_ALL_RAYS", "MIXED"};

struct EntityData {
	spp::Aabb aabb;
	spp::EntityType id = 0;
	uint32_t mask = ~0;
};

std::vector<EntityData> *globalEntityData = nullptr;

struct SingleTestResult {
	std::vector<spp::EntityType> entities;
	size_t nodesTestedCount = 0;
	size_t testedCount = 0;
	size_t hitCount = 0;
};

struct StartEndPoint {
	spp::Aabb aabb;
	glm::vec3 start, end, point;
	float n;
	spp::EntityType e;
};

std::vector<spp::EntityType> ee;
std::vector<glm::vec3> vv;

uint64_t TEST_RANDOM_SEED = 0;

std::vector<spp::Aabb> aabbsToTest;
SingleTestResult &SingleTest(spp::BroadphaseBase *broadphase,
							 const std::vector<spp::Aabb> &aabbsToTest,
							 size_t testsCount,
							 std::vector<uint64_t> &offsetOfPatch,
							 TestType testType,
							 std::vector<StartEndPoint> &hitPoints)
{
	static SingleTestResult ret;
	ret.entities.reserve(10000000);
	ret.entities.clear();
	ret.nodesTestedCount = 0;
	ret.testedCount = 0;
	ret.hitCount = 0;
	switch (testType) {
	case TEST_AABB: {

		struct _Cb : public spp::IntersectionCallback {
		} static cb;
		cb.mask = ~(uint32_t)0;
		typedef void (*CbT)(spp::IntersectionCallback *, spp::EntityType);
		cb.callback = (CbT) + [](_Cb *cb, spp::EntityType entity) {
			ret.entities.push_back(entity);
		};
		testsCount = std::min(testsCount, aabbsToTest.size());
		for (size_t i = 0; i < testsCount; ++i) {
			offsetOfPatch.push_back(ret.entities.size());
			cb.aabb = aabbsToTest[i];
			broadphase->IntersectAabb(cb);
		}

		ret.nodesTestedCount = cb.nodesTestedCount;
		ret.testedCount = cb.testedCount;

		return ret;
	} break;
	case TEST_RAY: {

		struct _Cb : public spp::RayCallback {
		} static cb;
		cb.mask = ~(uint32_t)0;
		typedef spp::RayPartialResult (*CbT)(spp::RayCallback *,
											 spp::EntityType);
		cb.callback =
			(CbT) +
			[](_Cb *cb, spp::EntityType entity) -> spp::RayPartialResult {
			assert(entity > 0);
			spp::Aabb aabb = (*globalEntityData)[entity - 1].aabb;
			if (cb->IsRelevant((spp::AabbCentered)aabb)) {
				ret.entities.push_back(entity);
				return {1.0f, true};
			}
			return {1.0f, false};
		};
		testsCount = std::min(testsCount, aabbsToTest.size());
		for (size_t i = 0; i < testsCount; ++i) {
			offsetOfPatch.push_back(ret.entities.size());
			cb.start = aabbsToTest[i].GetCenter();
			cb.end = aabbsToTest[(i + 17) % testsCount].GetCenter();
			cb.initedVars = false;
			broadphase->IntersectRay(cb);
		}

		ret.nodesTestedCount = cb.nodesTestedCount;
		ret.testedCount = cb.testedCount;
		ret.hitCount = cb.hitCount;

		return ret;
	} break;
	case TEST_RAY_FIRST: {
		struct _Cb : public spp::RayCallbackFirstHit {
		} static cb;
		cb.mask = ~(uint32_t)0;
		typedef spp::RayPartialResult (*CbT)(spp::RayCallback *,
											 spp::EntityType);
		cb.callback =
			(CbT) +
			[](_Cb *cb, spp::EntityType entity) -> spp::RayPartialResult {
			float n, f;
			spp::Aabb aabb = (*globalEntityData)[entity - 1].aabb;
			if (cb->IsRelevant((spp::AabbCentered)aabb, n, f)) {
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
			offsetOfPatch.push_back(ret.entities.size());
			cb.start = aabbsToTest[i].GetCenter();
			glm::vec3 end = cb.end =
				aabbsToTest[(i + 1) % testsCount].GetCenter();
			cb.initedVars = false;
			broadphase->IntersectRay(cb);
			if (cb.hasHit) {
				ret.entities.push_back(cb.hitEntity);
				hitPoints.push_back({broadphase->GetAabb(cb.hitEntity),
									 cb.start, end, cb.hitPoint, cb.cutFactor,
									 cb.hitEntity});
			} else {
				ret.entities.push_back(0);
				hitPoints.push_back({{}, cb.start, end, {}, -1, 0});
			}
		}

		ret.nodesTestedCount = cb.nodesTestedCount;
		ret.testedCount = cb.testedCount;
		ret.hitCount = cb.hitCount;

		return ret;
	} break;
	case TEST_MIXED: {

		struct _CbAabb : public spp::IntersectionCallback {
		} cbAabb;
		cbAabb.mask = ~(uint32_t)0;
		typedef void (*CbTAabb)(spp::IntersectionCallback *, spp::EntityType);
		cbAabb.callback = (CbTAabb) + [](_CbAabb *cb, spp::EntityType entity) {
			ret.entities.push_back(entity);
		};

		struct _CbRay : public spp::RayCallbackFirstHit {
		} static cbRay;
		cbRay.mask = ~(uint32_t)0;
		typedef spp::RayPartialResult (*CbTRay)(spp::RayCallback *,
												spp::EntityType);
		cbRay.callback =
			(CbTRay) +
			[](_CbRay *cb, spp::EntityType entity) -> spp::RayPartialResult {
			float n, f;
			spp::Aabb aabb = cb->broadphase->GetAabb(entity);
			if (cb->IsRelevant((spp::AabbCentered)aabb, n, f)) {
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

		std::vector<spp::EntityType> removeEntities;
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

		static spp::EntityType (*popRandom)(
			uint64_t &s, size_t i,
			std::vector<spp::EntityType> &removeEntities) =
			+[](uint64_t &s, size_t i,
				std::vector<spp::EntityType> &removeEntities)
			-> spp::EntityType {
			if (!removeEntities.empty() && (Random(s, i) & 7) > 5) {
				size_t x = Random(s, i) % removeEntities.size();
				auto r = removeEntities[x];
				removeEntities[x] = removeEntities.back();
				removeEntities.resize(removeEntities.size() - 1);
				return r;
			} else {
				return (Random(s, i) % MAX_ENTITIES) + 1;
			}
		};

		for (size_t i = 0; i < testsCount; i += 5) {
			offsetOfPatch.push_back(ret.entities.size());
			cbAabb.aabb = aabbsToTest[i + 0];
			broadphase->IntersectAabb(cbAabb);

			auto e = ee[i + 1];
			if (i % 55 == 0 || broadphase->Exists(e) == false) {
				if (broadphase->Exists(e) == false) {
					spp::Aabb aabb = aabbsToTest[i + 1];
					aabb.max = aabb.min + ((vv[i + 1] + 1000.0f) / 400.0f);
					assert(e > 0);
					broadphase->Add(e, aabb, ~0);
				}
				e = popRandom(s, i, removeEntities);

				if (broadphase->Exists(e)) {
					broadphase->Remove(e);
					removeEntities.push_back(e);
				} else {
					spp::Aabb aabb = aabbsToTest[i + 1];
					aabb.max = aabb.min + ((vv[i + 1] + 1000.0f) / 400.0f);
					assert(e > 0);
					broadphase->Add(e, aabb, ~0);
				}
			} else {
				spp::Aabb aabb = broadphase->GetAabb(e);
				aabb.min += vv[i + 1];
				aabb.max += vv[i + 1];
				broadphase->Update(e, aabb);
			}

			offsetOfPatch.push_back(ret.entities.size());
			cbAabb.aabb = aabbsToTest[i + 2];
			broadphase->IntersectAabb(cbAabb);

			for (int j = 3; j < 5; ++j) {
				cbRay.hasHit = false;
				offsetOfPatch.push_back(ret.entities.size());
				cbRay.start = aabbsToTest[i + j].GetCenter();
				glm::vec3 end = cbRay.end = vv[i + j];
				cbRay.initedVars = false;
				broadphase->IntersectRay(cbRay);
				if (cbRay.hasHit) {
					ret.entities.push_back(cbRay.hitEntity);
					hitPoints.push_back({broadphase->GetAabb(cbRay.hitEntity),
										 cbRay.start, cbRay.end, cbRay.hitPoint,
										 cbRay.cutFactor, cbRay.hitEntity});
				} else {
					ret.entities.push_back(0);
					hitPoints.push_back({{}, cbRay.start, end, {}, -1, 0});
				}
			}
		}

		ret.nodesTestedCount = cbRay.nodesTestedCount + cbAabb.nodesTestedCount;
		ret.testedCount = cbRay.testedCount + cbAabb.testedCount;
		ret.hitCount = cbRay.hitCount;

		return ret;
	} break;
	}
	return ret;
}

std::vector<spp::Aabb> globalAabbs;
std::vector<std::vector<spp::EntityType>>
Test(std::vector<spp::BroadphaseBase *> broadphases, size_t testsCount,
	 TestType testType)
{
	TEST_RANDOM_SEED = mt();

	std::uniform_real_distribution<float> distPos(-520, 520);
	std::uniform_real_distribution<float> distSize(pow(2.0 / 16.0, 1.0 / 3.0),
												   1.0);
	std::vector<spp::Aabb> &aabbs = globalAabbs;
	size_t i = aabbs.size();
	aabbs.resize(testsCount);
	for (; i < aabbs.size(); ++i) {
		spp::Aabb &aabb = aabbs[i];
		glm::vec3 p = {distPos(mt), distPos(mt) / 8.0f, distPos(mt)};
		glm::vec3 s = {distSize(mt), distSize(mt), distSize(mt)};
		s = s * s * 16.0f;
		aabb = {p, p + s};
	}

	std::vector<std::vector<StartEndPoint>> hitPoints;
	std::vector<std::vector<spp::EntityType>> ents;
	std::vector<std::vector<uint64_t>> offsetOfPatch;
	for (int i = 0; i < broadphases.size(); ++i) {
		offsetOfPatch.push_back({});
		offsetOfPatch.back().reserve(testsCount * 5);
		hitPoints.push_back({});
		hitPoints.back().reserve(testsCount * 10);
		auto &it = broadphases[i];
		size_t tC =
			i ? aabbs.size() : aabbs.size() / BRUTE_FROCE_TESTS_COUNT_DIVISOR;
		auto beg = std::chrono::steady_clock::now();
		auto &vec = SingleTest(it, aabbs, tC, offsetOfPatch.back(), testType,
							   hitPoints.back());
		auto end = std::chrono::steady_clock::now();
		for (size_t i = 0; i < offsetOfPatch.back().size(); ++i) {
			size_t of = offsetOfPatch.back()[i];
			size_t en = vec.entities.size();
			if (i + 1 < offsetOfPatch.back().size()) {
				en = offsetOfPatch.back()[i + 1];
			}
			if (testType != TEST_RAY_FIRST) {
				std::sort(vec.entities.begin() + of, vec.entities.begin() + en);
			}
		}
		ents.push_back(std::vector<spp::EntityType>(vec.entities.begin(),
													vec.entities.end()));
		auto diff = end - beg;
		int64_t ns =
			std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(diff)
				.count();
		double us = double(ns) / 1000.0;
		printf("%20s intersection test [count: %lu]: %8.3f us/op",
			   it->GetName(), tC, us / double(tC));
		printf("    \t   nodesTested: %11lu,   testedCount: %10lu     [count = "
			   "%lu]:\n",
			   vec.nodesTestedCount, vec.testedCount, ents.back().size());
		fflush(stdout);
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
				++cardinalErrors;
			}

			int I = 0;
			int j = 0;
			for (; it0->Valid(); it0->Next()) {
				++j;
				if (bpi.Exists(it0->entity) == false) {
					++cardinalErrors;
					if (I < 10) {
						printf(
							"1. CARDINAL ERROR: ENTITY DOES NOT EXIST: %lu\n",
							it0->entity);
						printf("j = %i\n", j);
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
							printf("j = %i\n", j);
						}
						++I;
					}
				}
			}

			I = 0;
			for (; iti->Valid(); iti->Next()) {
				++j;
				if (bp0.Exists(iti->entity) == false) {
					++cardinalErrors;
					if (I < 10) {
						printf("3. CARDINAL ERROR: ENTITY SHOULD NOT EXIST BUT "
							   "DOES: %lu\n",
							   iti->entity);
						printf("j = %i\n", j);
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
							printf("j = %i\n", j);
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
				if (spp::Aabb{hp.aabb.min - 0.01f, hp.aabb.max + 0.01f}
						.ContainsAll({hp.point, hp.point}) == false) {
					++cardinalErrors;
					if (I < 10) {

						float near;
						float far;
						bool r =
							hp.aabb.SlowRayTest(hp.start, hp.end, near, far);

						printf("5. CARDINAL ERROR: AABB RAY TEST ERROR VALUE, "
							   "HITPOINT OUTSIDE AABB");

						glm::vec3 a, b;
						a = hp.aabb.min;
						b = hp.aabb.max;

						printf("  %7i: %7lu    :     ", j, hp.e);
						printf("%7.2f %7.2f %7.2f .. %7.2f %7.2f %7.2f", a.x,
							   a.y, a.z, b.x, b.y, b.z);

						a = hp.point;
						printf("     hit point: %7.2f %7.2f %7.2f", a.x, a.y,
							   a.z);

						a = hp.start;
						printf("     start: %7.2f %7.2f %7.2f", a.x, a.y, a.z);

						a = hp.end;
						printf("     end: %7.2f %7.2f %7.2f", a.x, a.y, a.z);

						printf("     dist: %7.2f", hp.n);

						printf("     second result %s     near: %f   far: %f",
							   r ? "HIT" : "MISS", near, far);

						printf("\n");
					}
					++I;
				}
			}
		}

		size_t errs = glm::abs<int64_t>(ents[i].size() - ents[0].size());
		int JJ = 0;
		for (int j = 0; j < hitPoints[0].size() && j < hitPoints[i].size();
			 ++j) {
			auto p0 = hitPoints[0][j];
			auto pi = hitPoints[i][j];
			if (p0.e != pi.e) {
				spp::Aabb aabb0 = p0.aabb;
				spp::Aabb aabbi = pi.aabb;

				aabb0.min -= 0.0025;
				aabbi.min -= 0.0025;
				aabb0.max += 0.0025;
				aabbi.max += 0.0025;

				bool er = true;
				if (p0.e && pi.e) {
					if (aabb0 && aabbi) {
						if (aabb0 && spp::Aabb{p0.point, p0.point}) {
							if (aabbi && spp::Aabb{p0.point, p0.point}) {
								if (aabb0 && spp::Aabb{pi.point, pi.point}) {
									if (aabbi &&
										spp::Aabb{pi.point, pi.point}) {
										if (glm::length(pi.point - p0.point) <
												0.01 &&
											fabs(p0.n - pi.n) < 0.0001) {
											er = false;
										}
									}
								}
							}
						}
					}
				}

				if (er == true) {
					aabb0 = p0.aabb;
					aabbi = pi.aabb;

					++JJ;
					++errs;
					glm::vec3 a, b, c, d;
					a = aabb0.min;
					b = aabb0.max;
					c = aabbi.min;
					d = aabbi.max;
					if (JJ < 10) {
						printf("  %7i: %7lu == %7lu  ", j, p0.e, pi.e);
						printf(
							"%7.2f %7.2f %7.2f .. %7.2f %7.2f %7.2f <-> %7.2f "
							"%7.2f "
							"%7.2f .. %7.2f %7.2f %7.2f",
							a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z, d.x,
							d.y, d.z);

						a = p0.point;
						b = pi.point;
						printf("     hit points: %7.2f %7.2f %7.2f <-> %7.2f "
							   "%7.2f "
							   "%7.2f",
							   a.x, a.y, a.z, b.x, b.y, b.z);

						a = p0.start;
						b = pi.start;
						printf("     start: %7.2f %7.2f %7.2f <-> %7.2f %7.2f "
							   "%7.2f",
							   a.x, a.y, a.z, b.x, b.y, b.z);

						a = p0.end;
						b = pi.end;
						printf(
							"     end: %7.2f %7.2f %7.2f <-> %7.2f %7.2f %7.2f",
							a.x, a.y, a.z, b.x, b.y, b.z);

						printf("     dist: %7.2f <-> %7.2f", p0.n, pi.n);

						printf("\n");
					}
				}
			}
		}
		errs += cardinalErrors;
		printf(" %s: (cardinal err: %lu) errors count: %lu ... %s\n",
			   broadphases[i]->GetName(), cardinalErrors, errs,
			   errs ? "ERRORS" : "OK");
	}

	return ents;
}

void EnqueueRebuildThreaded(std::shared_ptr<std::atomic<bool>> fin,
							std::shared_ptr<spp::BroadphaseBase> dbvh,
							std::shared_ptr<void> data)
{

	if (false) {
		auto beg = std::chrono::steady_clock::now();
		dbvh->Rebuild();
		auto end = std::chrono::steady_clock::now();
		auto diff = end - beg;
		int64_t ns =
			std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(diff)
				.count();
		double us = double(ns) / 1000.0;
		printf("                        "
			   "%s (%i) "
			   "Async rebuild of : %.3f us/op\n",
			   dbvh->GetName(), dbvh->GetCount(),
			   us / double(TOTAL_AABB_MOVEMENTS));
		fflush(stdout);

		fin->store(true);
		return;
	}

	static std::atomic<int> size = 0;
	static std::mutex mutex;
	static bool done = false;
	using Pair = std::pair<std::shared_ptr<std::atomic<bool>>,
						   std::shared_ptr<spp::BroadphaseBase>>;
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
						}
					}
					--size;
					if (has && p.second.use_count() > 1) {
						auto beg = std::chrono::steady_clock::now();
						p.second->Rebuild();
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

						p.first->store(true);
					}
				}
			}
		});
		thread.detach();
	}
	std::lock_guard lock(mutex);
	queue.push({fin, dbvh});
	++size;
}

int main(int argc, char **argv)
{
	bool enablePrepass = true;

	bool customizeStructures = false;

	for (int i = 1; i < argc; ++i) {
		if (std::string(argv[i]).starts_with("-disable-prepass")) {
			enablePrepass = false;
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
				   "\t-total-entities=\n"
				   "\t-additional-entities=\n"
				   "\t-aabb-tests=\n"
				   "\t-aabb-movements=\n"
				   "\t-mixed-tests=\n"
				   "\t-moving-entities=\n"
				   "\tBF         - BruteForce\n"
				   "\tBVH        - BvhMedianSplitHeap\n"
				   "\tDBVH       - Dbvh (DynamicBoundingVolumeHierarchy)\n"
				   "\tBTDBVH     - BulletDbvh (Bullet dbvh - two stages)\n"
				   "\tBTDBVT     - BulletDbvt (Bullet dbvt one stage)\n"
				   "\tTSH_BT     - ThreeStageDbvh BvhMedian + BruteForce\n"
				   "\tTSH_BTDBVT - ThreeStageDbvh BvhMedian + BulletDbvt\n"
				   "\tTSH_DBVH   - ThreeStageDbvh BvhMedian + Dbvh\n"
				   "\tHLO        - HashedLooseOctree\n"
				   "\tLO         - LooseOctree\n");

			return 0;
		}
	}

	std::vector<EntityData> entities;
	globalEntityData = &entities;
	entities.resize(TOTAL_ENTITIES);
	std::uniform_real_distribution<float> distPos(-10, 10);
	std::uniform_real_distribution<float> distSize(0.4, 5);
	spp::EntityType id = 1;
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

	spp::BvhMedianSplitHeap bvh;
	bvh.SetAabbUpdatePolicy(spp::BvhMedianSplitHeap::ON_UPDATE_EXTEND_AABB);

	spp::BruteForce bf;
	spp::Dbvh dbvh;
	spp::HashLooseOctree hlo(1.0, 13, 1.6);
	spp::LooseOctree lo(-glm::vec3(1, 1, 1) * (1024 * 16.f), 15, 1.6);
	spp::BulletDbvh btDbvh;
	spp::BulletDbvt btDbvt;

	spp::ThreeStageDbvh tsdbvh(std::make_shared<spp::BvhMedianSplitHeap>(),
							   std::make_shared<spp::BvhMedianSplitHeap>(),
							   std::make_unique<spp::BulletDbvt>());
	tsdbvh.SetRebuildSchedulerFunction(EnqueueRebuildThreaded);

	spp::ThreeStageDbvh tsdbvh2(std::make_shared<spp::BvhMedianSplitHeap>(),
								std::make_shared<spp::BvhMedianSplitHeap>(),
								std::make_unique<spp::BruteForce>());
	tsdbvh2.SetRebuildSchedulerFunction(EnqueueRebuildThreaded);

	spp::ThreeStageDbvh tsdbvh3(std::make_shared<spp::BvhMedianSplitHeap>(),
								std::make_shared<spp::BvhMedianSplitHeap>(),
								std::make_unique<spp::Dbvh>());
	tsdbvh3.SetRebuildSchedulerFunction(EnqueueRebuildThreaded);

	std::vector<spp::BroadphaseBase *> broadphases;

	if (customizeStructures) {
		broadphases.clear();
		for (int i = 1; i < argc; ++i) {
			char *str = argv[i];
			if (false) {
			} else if (strcmp(str, "BF") == false) {
				broadphases.push_back(new spp::BruteForce);
			} else if (strcmp(str, "BVH") == false) {
				spp::BvhMedianSplitHeap *bvh;
				bvh = new spp::BvhMedianSplitHeap;
				broadphases.push_back(bvh);
				bvh->SetAabbUpdatePolicy(
					spp::BvhMedianSplitHeap::ON_UPDATE_EXTEND_AABB);
			} else if (strcmp(str, "DBVH") == false) {
				broadphases.push_back(new spp::Dbvh);
			} else if (strcmp(str, "BTDBVH") == false) {
				broadphases.push_back(new spp::BulletDbvh);
			} else if (strcmp(str, "BTDBVT") == false) {
				broadphases.push_back(new spp::BulletDbvt);
			} else if (strcmp(str, "TSH_BT") == false) {
				spp::ThreeStageDbvh *tsdbvh = new spp::ThreeStageDbvh(
					std::make_shared<spp::BvhMedianSplitHeap>(),
					std::make_shared<spp::BvhMedianSplitHeap>(),
					std::make_unique<spp::BruteForce>());
				tsdbvh->SetRebuildSchedulerFunction(EnqueueRebuildThreaded);
				broadphases.push_back(tsdbvh);
			} else if (strcmp(str, "TSH_BTDBVT") == false) {
				spp::ThreeStageDbvh *tsdbvh = new spp::ThreeStageDbvh(
					std::make_shared<spp::BvhMedianSplitHeap>(),
					std::make_shared<spp::BvhMedianSplitHeap>(),
					std::make_unique<spp::BulletDbvt>());
				tsdbvh->SetRebuildSchedulerFunction(EnqueueRebuildThreaded);
				broadphases.push_back(tsdbvh);
			} else if (strcmp(str, "TSH_DBVH") == false) {
				spp::ThreeStageDbvh *tsdbvh = new spp::ThreeStageDbvh(
					std::make_shared<spp::BvhMedianSplitHeap>(),
					std::make_shared<spp::BvhMedianSplitHeap>(),
					std::make_unique<spp::Dbvh>());
				tsdbvh->SetRebuildSchedulerFunction(EnqueueRebuildThreaded);
				broadphases.push_back(tsdbvh);
			} else if (strcmp(str, "HLO") == false) {
				broadphases.push_back(new spp::HashLooseOctree(1.0, 13, 1.6));
			} else if (strcmp(str, "LO") == false) {
				broadphases.push_back(new spp::LooseOctree(
					-glm::vec3(1, 1, 1) * (1024 * 16.f), 15, 1.6));
			} else if (strcmp(str, "CLO") == false) {
				printf("ChunkedLooseOctree not implemented\n");
			}
			fflush(stdout);
		}
	} else {
		spp::ThreeStageDbvh *s;
		broadphases = {new spp::Dbvh,
					   (s = new spp::ThreeStageDbvh(
							std::make_shared<spp::BvhMedianSplitHeap>(),
							std::make_shared<spp::BvhMedianSplitHeap>(),
							std::make_unique<spp::BulletDbvt>()),
						s->SetRebuildSchedulerFunction(EnqueueRebuildThreaded),
						s)};
	}

	if (enablePrepass) {

		for (auto bp : broadphases) {
			auto beg = std::chrono::steady_clock::now();
			bp->StartFastAdding();
			for (const auto &e : entities) {
				assert(e.id > 0);
				bp->Add(e.id, e.aabb, e.mask);
			}
			bp->StopFastAdding();
			auto end = std::chrono::steady_clock::now();
			auto diff = end - beg;
			int64_t ns =
				std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(
					diff)
					.count();
			double us = double(ns) / 1000.0;
			printf("%s adding time: %.3f us\n", bp->GetName(), us);
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
			printf("%s build: %.3f us\n", bp->GetName(), us);
			fflush(stdout);
		}
		printf("\n");

		for (auto bp : broadphases) {
			printf("%s memory: %lu B , %.6f MiB\n", bp->GetName(),
				   bp->GetMemoryUsage(),
				   bp->GetMemoryUsage() / (1024.0 * 1024.0));
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

		for (auto bp : broadphases) {
			auto beg = std::chrono::steady_clock::now();
			for (int i = 0; i < 300 && i < entities.size(); ++i) {
				bp->Update(entities[i].id, entities[i].aabb);
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
			printf("%s rebuild: %.3f us\n", bp->GetName(), us);
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
		for (auto bp : broadphases) {
			entities = old;
			auto beg = std::chrono::steady_clock::now();
			for (size_t i = 0; i < ee.size(); ++i) {
				spp::Aabb &a = entities[ee[i] - 1].aabb;
				a.min += vv[i];
				a.max += vv[i];
				bp->Update(ee[i], a);
			}
			auto end = std::chrono::steady_clock::now();
			auto diff = end - beg;
			int64_t ns =
				std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(
					diff)
					.count();
			double us = double(ns) / 1000.0;
			printf("%s update data: %.3f us/op\n", bp->GetName(),
				   us / double(TOTAL_AABB_MOVEMENTS));
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
			if (auto b = dynamic_cast<spp::BvhMedianSplitHeap *>(bp)) {
				std::multimap<double, int32_t, std::greater<double>> timestage;
				std::vector<char> bytesCacheTrashing;
				bytesCacheTrashing.resize(1024);
				spp::BvhMedianSplitHeap::RebuildProgress pr;
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
			printf("%s rebuild: %.3f us\n", bp->GetName(), us);
			fflush(stdout);
		}
		printf("\n");

		printf("\nAfter updated WITH rebuild:\n\n");

		for (int i = 1; i <= 3; ++i) {
			printf("\n     TestType: %s\n", testTypeNames[i]);
			Test(broadphases, TOTAL_AABB_TESTS, (TestType)i);
		}

		printf("\nAfter reconstruct and full rebuild:\n");
	}

	for (auto bp : broadphases) {
		auto beg = std::chrono::steady_clock::now();
		bp->Clear();
		auto end = std::chrono::steady_clock::now();
		auto diff = end - beg;
		int64_t ns =
			std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(diff)
				.count();
		double us = double(ns) / 1000.0;
		printf("%s clearing time: %.3f us\n", bp->GetName(), us);
		fflush(stdout);
	}

	printf("\n");

	for (auto bp : broadphases) {
		auto beg = std::chrono::steady_clock::now();
		bp->StartFastAdding();
		for (const auto &e : entities) {
			assert(e.id > 0);
			if (bp->Exists(e.id) == false) {
				bp->Add(e.id, e.aabb, e.mask);
			}
		}
		bp->StopFastAdding();
		auto end = std::chrono::steady_clock::now();
		auto diff = end - beg;
		int64_t ns =
			std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(diff)
				.count();
		double us = double(ns) / 1000.0;
		printf("%s adding time: %.3f us\n", bp->GetName(), us);
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
		printf("%s build: %.3f us\n", bp->GetName(), us);
		fflush(stdout);
	}
	printf("\n");

	if (enablePrepass) {
		for (int i = 1; i <= 3; ++i) {
			printf("\n     TestType: %s\n", testTypeNames[i]);
			Test(broadphases, TOTAL_AABB_TESTS, (TestType)i);
		}
	}

	printf("\nMore realistinc dynamic movement entagled with tests:\n");

	for (int i = 0; i < 10000; ++i) {
		ee.clear();
		vv.clear();
		ee.reserve(TOTAL_MOVES_AND_TESTS);
		vv.reserve(TOTAL_MOVES_AND_TESTS);
		for (size_t i = 0; i < TOTAL_MOVES_AND_TESTS; ++i) {
			ee.push_back(((mt() % MAX_MOVING_ENTITIES) % entities.size()) + 1);
			vv.push_back({distPos(mt), distPos(mt) / 4.0f, distPos(mt)});
		}

		Test(broadphases, TOTAL_MOVES_AND_TESTS, TEST_MIXED);

		printf("\n");
		for (auto bp : broadphases) {
			printf("%s memory: %lu B , %.6f MiB\n", bp->GetName(),
				   bp->GetMemoryUsage(),
				   bp->GetMemoryUsage() / (1024.0 * 1024.0));
			fflush(stdout);
		}
		printf("\n");
	}

	for (auto bp : broadphases) {
		bp->Clear();
		delete bp;
	}

	return 0;
}
