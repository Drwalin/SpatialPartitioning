#include <cstdio>

#include <algorithm>
#include <random>
#include <vector>
#include <chrono>

#include "../include/spatial_partitioning/BroadPhaseBase.hpp"
#include "../include/spatial_partitioning/BruteForce.hpp"
#include "../include/spatial_partitioning/BvhMedianSplitHeap.hpp"
#include "../include/spatial_partitioning/Dbvh.hpp"
#include "../include/spatial_partitioning/HashLooseOctree.hpp"
#include "../include/spatial_partitioning/LooseOctree.hpp"

const int32_t TOTAL_ENTITIES = 10000;
const size_t TOTAL_AABB_TESTS = 10000;
const size_t TOTAL_AABB_MOVEMENTS = 10000;
const size_t MAX_MOVING_ENTITIES = 1500;
const size_t BRUTE_FROCE_TESTS_COUNT_DIVISOR = 1;

std::mt19937_64 mt(12345);

enum TestType {
	TEST_AABB = 1,
	TEST_RAY_FIRST = 2,
	TEST_RAY = 3,
};

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
	glm::vec3 start, end, point;
	float n;
	spp::EntityType e;
};

std::vector<spp::Aabb> aabbsToTest;
SingleTestResult SingleTest(spp::BroadphaseBase *broadphase,
							const std::vector<spp::Aabb> &aabbsToTest,
							size_t testsCount,
							std::vector<uint64_t> &offsetOfPatch,
							TestType testType,
							std::vector<StartEndPoint> &hitPoints)
{
	SingleTestResult ret;
	switch (testType) {
	case TEST_AABB: {
		struct _Cb : public spp::IntersectionCallback {
			std::vector<spp::EntityType> entities;
		} cb;
		cb.mask = ~(uint32_t)0;
		typedef void (*CbT)(spp::IntersectionCallback *, spp::EntityType);
		cb.callback = (CbT) + [](_Cb *cb, spp::EntityType entity) {
			cb->entities.push_back(entity);
		};
		testsCount = std::min(testsCount, aabbsToTest.size());
		for (size_t i = 0; i < testsCount; ++i) {
			offsetOfPatch.push_back(cb.entities.size());
			cb.aabb = aabbsToTest[i];
			broadphase->IntersectAabb(cb);
		}

		std::swap(ret.entities, cb.entities);
		ret.nodesTestedCount = cb.nodesTestedCount;
		ret.testedCount = cb.testedCount;
		return ret;
	} break;
	case TEST_RAY: {
		struct _Cb : public spp::RayCallback {
			std::vector<spp::EntityType> entities;
		} cb;
		cb.mask = ~(uint32_t)0;
		typedef spp::RayPartialResult (*CbT)(spp::RayCallback *, spp::EntityType);
		cb.callback =
			(CbT) + [](_Cb *cb, spp::EntityType entity) -> spp::RayPartialResult {
			float n, f;
			spp::Aabb aabb = (*globalEntityData)[entity - 1].aabb;
			if (aabb.FastRayTest(cb->start, cb->dirNormalized, cb->invDir,
								 cb->length, n, f)) {
				cb->entities.push_back(entity);
				return {1.0f, true};
			}
			return {1.0f, false};
		};
		testsCount = std::min(testsCount, aabbsToTest.size());
		for (size_t i = 0; i < testsCount; ++i) {
			offsetOfPatch.push_back(cb.entities.size());
			cb.start = aabbsToTest[i].GetCenter();
			cb.end = aabbsToTest[(i + 1) % testsCount].GetCenter();
			broadphase->IntersectRay(cb);
		}

		std::swap(ret.entities, cb.entities);
		ret.nodesTestedCount = cb.nodesTestedCount;
		ret.testedCount = cb.testedCount;
		ret.hitCount = cb.hitCount;
		return ret;
	} break;
	case TEST_RAY_FIRST: {
		struct _Cb : public spp::RayCallbackFirstHit {
		} cb;
		cb.mask = ~(uint32_t)0;
		typedef spp::RayPartialResult (*CbT)(spp::RayCallback *, spp::EntityType);
		cb.callback =
			(CbT) + [](_Cb *cb, spp::EntityType entity) -> spp::RayPartialResult {
			float n, f;
			spp::Aabb aabb = (*globalEntityData)[entity - 1].aabb;
			if (aabb.FastRayTest(cb->start, cb->dirNormalized, cb->invDir,
								 cb->length, n, f)) {
				if (n < 0.0f) {
					n = 0.0f;
				}
				if (cb->hasHit == false) {
					cb->hitDistance = n;
					cb->hitPoint = cb->start + cb->dirNormalized * n;
					cb->hitEntity = entity;
					cb->hasHit = true;
					return {n * cb->invLength, true};
				} else if (n < cb->hitDistance) {
					cb->hitDistance = n;
					cb->hitPoint = cb->start + cb->dirNormalized * n;
					cb->hitEntity = entity;
					cb->hasHit = true;
					return {n * cb->invLength, true};
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
			broadphase->IntersectRay(cb);
			if (cb.hasHit) {
				ret.entities.push_back(cb.hitEntity);
				hitPoints.push_back(
					{cb.start, end, cb.hitPoint, cb.hitDistance, cb.hitEntity});
			} else {
				ret.entities.push_back(0);
				hitPoints.push_back({cb.start, end, {}, -1, 0});
			}
		}

		ret.nodesTestedCount = cb.nodesTestedCount;
		ret.testedCount = cb.testedCount;
		ret.hitCount = cb.hitCount;
		return ret;
	} break;
	}
	return {};
}

std::vector<spp::Aabb> globalAabbs;
std::vector<std::vector<spp::EntityType>>
Test(std::vector<spp::BroadphaseBase *> broadphases, size_t testsCount,
	 TestType testType)
{
	std::uniform_real_distribution<float> distPos(-1000, 1000);
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
	std::vector<uint64_t> offsetOfPatch;
	offsetOfPatch.reserve(testsCount);
	for (int i = 0; i < broadphases.size(); ++i) {
		offsetOfPatch.clear();
		hitPoints.push_back({});
		hitPoints.back().reserve(testsCount);
		auto &it = broadphases[i];
		size_t tC =
			i ? aabbs.size() : aabbs.size() / BRUTE_FROCE_TESTS_COUNT_DIVISOR;
		auto beg = std::chrono::steady_clock::now();
		auto vec = SingleTest(it, aabbs, tC, offsetOfPatch, testType,
							  hitPoints.back());
		auto end = std::chrono::steady_clock::now();
		for (size_t i = 0; i < offsetOfPatch.size(); ++i) {
			size_t of = offsetOfPatch[i];
			size_t en = vec.entities.size();
			if (i + 1 < offsetOfPatch.size()) {
				en = offsetOfPatch[i + 1];
			}
			if (testType != TEST_RAY_FIRST) {
				std::sort(vec.entities.begin() + of, vec.entities.begin() + en);
			}
		}
		ents.push_back(
			std::vector<spp::EntityType>(vec.entities.begin(), vec.entities.end()));
		auto diff = end - beg;
		int64_t ns =
			std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(diff)
				.count();
		double us = double(ns) / 1000.0;
		printf("%s intersection test [count: %lu]: %.3f us/op\n", it->GetName(),
			   tC, us / double(tC));
		printf("    nodesTested: %lu,   testedCount: %lu     [count = %lu]:\n ",
			   vec.nodesTestedCount, vec.testedCount, ents.back().size());
		fflush(stdout);
	}

	printf("\n");
	for (int i = 1; i < broadphases.size(); ++i) {
		size_t errs = glm::abs<int64_t>(ents[i].size() - ents[0].size());
		for (int j = 0; j < ents[i].size() && j < ents[0].size() &&
						j < hitPoints[0].size() && j < hitPoints[i].size();
			 ++j) {
			auto p0 = hitPoints[0][j];
			auto pi = hitPoints[i][j];
			if (p0.e != pi.e) {
				auto aabb0 =
					p0.e ? (*globalEntityData)[p0.e - 1].aabb : spp::Aabb{};
				auto aabbi =
					pi.e ? (*globalEntityData)[pi.e - 1].aabb : spp::Aabb{};

				auto com = aabb0 * aabbi;
				bool er = true;
				if (aabb0 && aabbi) {
					if (com && spp::Aabb{p0.point, p0.point}) {
						if (com && spp::Aabb{pi.point, pi.point}) {
							if (glm::length(pi.point - p0.point) < 0.001) {
								er = false;
							}
						}
					}
				}

				if (p0.e == 0 || pi.e == 0) {
					er = true;
				}

				if (er == true) {
					++errs;
					glm::vec3 a, b, c, d;
					a = aabb0.min;
					b = aabb0.max;
					c = aabbi.min;
					d = aabbi.max;
					printf("  %7i: %7lu == %7lu  ", j, p0.e, pi.e);
					printf("%7.2f %7.2f %7.2f .. %7.2f %7.2f %7.2f <-> %7.2f "
						   "%7.2f "
						   "%7.2f .. %7.2f %7.2f %7.2f",
						   a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z, d.x,
						   d.y, d.z);

					a = p0.point;
					b = pi.point;
					printf("     hit points: %7.2f %7.2f %7.2f <-> %7.2f %7.2f "
						   "%7.2f",
						   a.x, a.y, a.z, b.x, b.y, b.z);

					a = p0.start;
					b = pi.start;
					printf(
						"     start: %7.2f %7.2f %7.2f <-> %7.2f %7.2f %7.2f",
						a.x, a.y, a.z, b.x, b.y, b.z);

					a = p0.end;
					b = pi.end;
					printf("     end: %7.2f %7.2f %7.2f <-> %7.2f %7.2f %7.2f",
						   a.x, a.y, a.z, b.x, b.y, b.z);

					printf("     dist: %7.2f <-> %7.2f", p0.n, pi.n);

					printf("\n");
				}
			}
		}
		printf(" %s: errors count: %lu ... %s\n", broadphases[i]->GetName(),
			   errs, errs ? "ERRORS" : "OK");
	}

	return ents;
}

int main()
{
	std::vector<EntityData> entities;
	globalEntityData = &entities;
	entities.resize(TOTAL_ENTITIES);
	std::uniform_real_distribution<float> distPos(-300, 300);
	std::uniform_real_distribution<float> distSize(0.4, 10);
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
	for (int i = 2; i < 50; ++i) {
		auto &e = entities[i];
		e.aabb.min = {distPos(mt), distPos(mt) / 16.0f, distPos(mt)};
		e.aabb.max = {distSize(mt) * 6, distSize(mt) * 2, distSize(mt) * 6};
		e.aabb.max += e.aabb.min;
	}

	spp::BvhMedianSplitHeap bvh;
	spp::BruteForce bf;
	spp::Dbvh dbvh;
	spp::HashLooseOctree hlo(1.0, 14, 1.6);
	spp::LooseOctree lo(-glm::vec3{4096,4096,4096}, 1.0, 16, 1.6);

	std::vector<spp::BroadphaseBase *> broadphases = {
// 		&bf,
		&bvh,
		&dbvh,
// 		&hlo,
		&lo,
	};

	for (auto bp : broadphases) {
		auto beg = std::chrono::steady_clock::now();
		for (const auto &e : entities) {
			bp->Add(e.id, e.aabb, e.mask);
		}
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
	
	for (auto bp : broadphases) {
		printf("%s memory: %lu B , %.6f MiB\n", bp->GetName(), bp->GetMemoryUsage(), bp->GetMemoryUsage()/(1024.0*1024.0));
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
			std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(diff)
				.count();
		double us = double(ns) / 1000.0;
		printf("%s update data: %.3f us\n", bp->GetName(), us);
		fflush(stdout);
	}
	printf("\n");
	*/

	for (auto bp : broadphases) {
		auto beg = std::chrono::steady_clock::now();
		bp->Rebuild();
		auto end = std::chrono::steady_clock::now();
		auto diff = end - beg;
		int64_t ns =
			std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(diff)
				.count();
		double us = double(ns) / 1000.0;
		printf("%s rebuild: %.3f us\n", bp->GetName(), us);
		fflush(stdout);
	}
	printf("\n");

	printf("\nAfter rebuild:\n\n");

	for (int i = 1; i <= 3; ++i) {
		printf("\n     TestType: %s\n", i == 1	 ? "AABB"
										: i == 3 ? "RAY"
												 : "RAY_FIRST");
		Test(broadphases, TOTAL_AABB_TESTS, (TestType)i);
	}

	bvh.SetAabbUpdatePolicy(spp::BvhMedianSplitHeap::ON_UPDATE_EXTEND_AABB);

	std::vector<spp::EntityType> ee;
	std::vector<glm::vec3> vv;
	ee.reserve(TOTAL_AABB_MOVEMENTS);
	vv.reserve(TOTAL_AABB_MOVEMENTS);
	for (size_t i = 0; i < TOTAL_AABB_MOVEMENTS; ++i) {
		ee.push_back(((mt() % MAX_MOVING_ENTITIES ) % entities.size()) + 1);
		vv.push_back({distPos(mt), distPos(mt) / 4.0f, distPos(mt)});
	}

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
			std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(diff)
				.count();
		double us = double(ns) / 1000.0;
		printf("%s update data: %.3f us/op\n", bp->GetName(), us / double(TOTAL_AABB_MOVEMENTS));
		fflush(stdout);
	}
	printf("\n");

	printf("\nAfter updated without rebuild:\n\n");

	for (int i = 1; i <= 3; ++i) {
		printf("\n     TestType: %s\n", i == 1	 ? "AABB"
										: i == 2 ? "RAY"
												 : "RAY_FIRST");
		Test(broadphases, TOTAL_AABB_TESTS, (TestType)i);
	}
	
	printf("\nAfter reconstruct and full rebuild:\n");
	
	for (auto bp : broadphases) {
		bp->Clear();
	}
	
	for (auto bp : broadphases) {
		auto beg = std::chrono::steady_clock::now();
		for (const auto &e : entities) {
			bp->Add(e.id, e.aabb, e.mask);
		}
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
	
	for (int i = 1; i <= 3; ++i) {
		printf("\n     TestType: %s\n", i == 1	 ? "AABB"
										: i == 2 ? "RAY"
												 : "RAY_FIRST");
		Test(broadphases, TOTAL_AABB_TESTS, (TestType)i);
	}

	return 0;
}
