/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use
of this software. Permission is granted to anyone to use this software for any
purpose, including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim
that you wrote the original software. If you use this software in a product, an
acknowledgment in the product documentation would be appreciated but is not
required.
2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
/// btDbvt implementation by Nathanael Presson
///  2025 modified and rewritten by Marek Zalewski aka Drwalin

#pragma once

#include <immintrin.h>

#include "Aabb.hpp"

namespace spp
{

/* btDbvtNode				*/
struct btDbvtNode {
	Aabb volume;
	btDbvtNode *parent;
	inline bool isleaf() const { return (childs[1] == 0); }
	inline bool isinternal() const { return (!isleaf()); }
	union {
		btDbvtNode *childs[2];
		void *data;
		int dataAsInt;
	};
};

typedef std::vector<const btDbvtNode *> btNodeStack;

/// The btDbvt class implements a fast dynamic bounding volume tree based on
/// axis aligned bounding boxes (aabb tree). This btDbvt is used for soft body
/// collision detection and for the btDbvtBroadphase. It has a fast insert,
/// remove and update of nodes. Unlike the btQuantizedBvh, nodes can be
/// dynamically moved around, which allows for change in topology of the
/// underlying data structure.
struct btDbvt {
	/* Stack element	*/

	/* ICollide	*/
	struct ICollide {
		virtual ~ICollide() {}
		virtual void Process(const btDbvtNode *, const btDbvtNode *) {}
		virtual void Process(const btDbvtNode *) {}
		virtual void Process(const btDbvtNode *n, float) { Process(n); }
		virtual bool Descent(const btDbvtNode *) { return (true); }
		virtual bool AllLeaves(const btDbvtNode *) { return (true); }
	};
	/* IClone	*/
	struct IClone {
		virtual ~IClone() {}
		virtual void CloneLeaf(btDbvtNode *) {}
	};

	// Constants
	enum { SIMPLE_STACKSIZE = 64, DOUBLE_STACKSIZE = SIMPLE_STACKSIZE * 2 };

	// Fields
	btDbvtNode *m_root = nullptr;
	btDbvtNode *m_free = nullptr;
	int m_lkhd = -1;
	int m_leaves = 0;
	unsigned m_opath = 0;

	std::vector<const btDbvtNode *> stack;

	// Methods
	btDbvt();
	~btDbvt();
	void clear();
	bool empty() const { return (0 == m_root); }
	void optimizeBottomUp();
	void optimizeTopDown(int bu_treshold = 128);
	void optimizeIncremental(int passes);
	btDbvtNode *insert(const Aabb &box, void *data);
	void update(btDbvtNode *leaf, int lookahead = -1);
	void update(btDbvtNode *leaf, Aabb &volume);
	bool update(btDbvtNode *leaf, Aabb &volume, float margin);
	void remove(btDbvtNode *leaf);

	// T& policy must support ICollide policy/interface

	template <typename T> void collideTV(const Aabb &volume, T &policy);

	/// rayTestInternal is faster than rayTest, because it uses a persistent
	/// stack (to reduce dynamic memory allocations to a minimum) and it uses
	/// precomputed signs/rayInverseDirections rayTestInternal is used by
	/// btDbvtBroadphase to accelerate world ray casts
	template <typename T>
	void rayTestInternal(const glm::vec3 &start, const glm::vec3 &end,
						 const glm::vec3 &invDir, int signs[3],
						 float &lambda_max, T &policy);
	//
private:
	btDbvt(const btDbvt &) {}
};

//
inline void SignedExpand(Aabb &aabb, const glm::vec3 &e)
{
	if (e.x > 0)
		aabb.max.x = aabb.max.x + e.x;
	else
		aabb.min.x = aabb.min.x + e.x;
	if (e.y > 0)
		aabb.max.y = aabb.max.y + e.y;
	else
		aabb.min.y = aabb.min.y + e.y;
	if (e.z > 0)
		aabb.max.z = aabb.max.z + e.z;
	else
		aabb.min.z = aabb.min.z + e.z;
}

//
inline bool Contain(const Aabb &aabb, const Aabb &a)
{
	return ((aabb.min.x <= a.min.x) && (aabb.min.y <= a.min.y) &&
			(aabb.min.z <= a.min.z) && (aabb.max.x >= a.max.x) &&
			(aabb.max.y >= a.max.y) && (aabb.max.z >= a.max.z));
}

//////////////////////////////////////

//
inline float Proximity(const Aabb &a, const Aabb &b)
{
	const glm::vec3 d = (a.min + a.max) - (b.min + b.max);
	return (glm::abs(d.x) + glm::abs(d.y) + glm::abs(d.z));
}

//
inline int Select(const Aabb &o, const Aabb &a, const Aabb &b)
{
	return (Proximity(o, a) < Proximity(o, b) ? 0 : 1);
}

//
inline void Merge(const Aabb &a, const Aabb &b, Aabb &r) { r = a + b; }

//
inline bool NotEqual(const Aabb &a, const Aabb &b)
{
	return ((a.min.x != b.min.x) || (a.min.y != b.min.y) ||
			(a.min.z != b.min.z) || (a.max.x != b.max.x) ||
			(a.max.y != b.max.y) || (a.max.z != b.max.z));
}

//
// Inline's
//

//
template <typename T> inline void btDbvt::collideTV(const Aabb &vol, T &policy)
{
	if (m_root) {
		Aabb volume(vol);
		stack.resize(0);
		stack.reserve(SIMPLE_STACKSIZE);
		stack.push_back(m_root);
		do {
			const btDbvtNode *n = stack[stack.size() - 1];
			stack.pop_back();
			if (n->volume && volume) {
				if (n->isinternal()) {
					stack.push_back(n->childs[0]);
					stack.push_back(n->childs[1]);
				} else {
					policy.Process(n);
				}
			}
		} while (stack.size() > 0);
	}
}

template <typename T>
inline void btDbvt::rayTestInternal(const glm::vec3 &start,
									const glm::vec3 &end,
									const glm::vec3 &invDir, int signs[3],
									float &near, T &policy)
{
	if (m_root) {
		int depth = 1;
		int treshold = DOUBLE_STACKSIZE - 2;
		stack.resize(DOUBLE_STACKSIZE);
		stack[0] = m_root;
		Aabb aabb;
		do {
			const btDbvtNode *node = stack[--depth];
			aabb = node->volume;
			float _near;
			float far;
			unsigned int result1 = false;
			result1 =
				node->volume.FastRayTest2(start, invDir, signs, _near, far);
			if (result1 && _near < near) {
				if (node->isinternal()) {
					if (depth > treshold) {
						stack.resize(stack.size() * 2);
						treshold = stack.size() - 2;
					}
					stack[depth++] = node->childs[0];
					stack[depth++] = node->childs[1];
				} else {
					policy.Process(node);
				}
			}
		} while (depth);
	}
}
} // namespace spp
