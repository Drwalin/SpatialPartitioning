// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <cstring>

#include <vector>

#include "HashMap.hpp"

namespace spp
{
template <typename KeyUIntType, typename ValueType, bool enableDense = false,
		  ValueType NULL_VALUE = 0>
class DenseSparseIntMap
{
public:
	inline const static bool IS_INTEGRAL = std::is_integral_v<ValueType>;

	inline DenseSparseIntMap(KeyUIntType denseRange)
		: denseRange(denseRange + 1)
	{
		Clear();
	}
	inline ~DenseSparseIntMap() {}

	inline DenseSparseIntMap(const DenseSparseIntMap &) = default;
	inline DenseSparseIntMap(DenseSparseIntMap &) = default;
	inline DenseSparseIntMap(DenseSparseIntMap &&) = default;

	inline void Reserve(KeyUIntType capacity)
	{
		if constexpr (enableDense == false) {
			sparse.reserve(capacity);
		} else if (capacity > denseRange) {
			sparse.reserve(capacity - denseRange);
		}
	}

	inline void Clear()
	{
		size = 0;
		sparse.clear();
		if constexpr (enableDense) {
			dense.resize(denseRange);
			if constexpr (IS_INTEGRAL) {
				if constexpr (NULL_VALUE == 0) {
					memset(dense.data(), 0, denseRange * sizeof(ValueType));
					return;
				}
			}
			for (auto &v : dense) {
				v = NULL_VALUE;
			}
		}
	}

	inline void Insert(KeyUIntType key, ValueType value) { Set(key, value); }

	inline void Remove(KeyUIntType key)
	{
		if (enableDense && (key < denseRange)) {
			if (dense[key] != NULL_VALUE) {
				--size;
			}
			dense[key] = NULL_VALUE;
		} else {
			if (sparse.contains(key)) {
				--size;
			}
			sparse.erase(key);
		}
	}

	inline void Set(KeyUIntType key, ValueType value)
	{
		if (enableDense && (key < denseRange)) {
			if (!(dense[key] != NULL_VALUE) && value != NULL_VALUE) {
				++size;
			}
			dense[key] = value;
		} else {
			if (!(value != NULL_VALUE)) {
				Remove(key);
			} else {
				if (sparse.contains(key) == false) {
					++size;
				}
				sparse[key] = value;
			}
		}
	}

	inline ValueType Get(KeyUIntType key) const
	{
		if (enableDense && (key < denseRange)) {
			return dense[key];
		} else {
			auto it = sparse.find(key);
			if (it != sparse.end()) {
				return it->second;
			}
		}
		return NULL_VALUE;
	}

	inline bool Has(KeyUIntType key) const
	{
		if (enableDense && (key < denseRange)) {
			return dense[key] != NULL_VALUE;
		} else {
			auto it = sparse.find(key);
			if (it != sparse.end()) {
				return it->second != NULL_VALUE;
			}
		}
		return false;
	}

	inline ValueType *find(KeyUIntType key)
	{
		if (enableDense && (key < denseRange)) {
			if (dense[key] != NULL_VALUE) {
				return &(dense[key]);
			}
		} else {
			auto it = sparse.find(key);
			if (it != sparse.end()) {
				if (it->second != NULL_VALUE) {
					return &(it->second);
				}
			}
		}
		return nullptr;
	}

	inline const ValueType *find(KeyUIntType key) const
	{
		if (enableDense && (key < denseRange)) {
			if (dense[key] != NULL_VALUE) {
				return &(dense[key]);
			}
		} else {
			auto it = sparse.find(key);
			if (it != sparse.end()) {
				if (it->second != NULL_VALUE) {
					return &(it->second);
				}
			}
		}
		return nullptr;
	}

	inline ValueType operator[](KeyUIntType key) const { return Get(key); }

	__attribute__((noinline))
	void ShrinkToFit() { sparse.rehash(0); }

	inline KeyUIntType Size() const { return size; }

	inline size_t GetMemoryUsage() const
	{
		return sparse.GetMemoryUsage() + dense.capacity() * sizeof(ValueType);
	}

public:
	struct Iterator {
		Iterator(Iterator &&it) = default;
		Iterator(Iterator &it) = default;
		Iterator(const Iterator &it) = default;
		
		Iterator &operator=(Iterator &&it) = default;
		Iterator &operator=(Iterator &it) = default;
		Iterator &operator=(const Iterator &it) = default;
		
		Iterator(DenseSparseIntMap *map) : map(map)
		{
			if (map) {
				first = 0;
				Next();
			} else {
				first = -1;
			}
		}

		bool operator==(Iterator o) const
		{
			if (o.end && end) {
				return true;
			}
			return it2 == o.it2 && first == o.first;
		}

		bool operator!=(Iterator o) const { return !((*this) == o); }

		Iterator SetEnd()
		{
			end = true;
			first = map->denseRange;
			it2 = map->sparse.end();
			return *this;
		}

		bool end = false;
		DenseSparseIntMap *map;
		KeyUIntType first;
		ValueType second;
		std::unordered_map<KeyUIntType, ValueType>::iterator it2;

		void operator++(int) { Next(); }
		void operator++() { Next(); }

		std::pair<KeyUIntType, ValueType> operator*()
		{
			return {first, second};
		}

		void Next()
		{
			if (first < map->denseRange) {
				do {
					++first;
				} while (first < map->denseRange &&
						 !(map->dense[first] != NULL_VALUE));
				if (first < map->denseRange) {
					second = map->dense[first];
					return;
				}
				it2 = map->sparse.begin();
				if (it2 != map->sparse.end()) {
					first = it2->first;
					second = it2->second;
				} else {
					end = true;
				}
				return;
			}
			++it2;
			if (it2 != map->sparse.end()) {
				first = it2->first;
				second = it2->second;
			} else {
				end = true;
			}
			return;
		}
	};

	Iterator begin() { return Iterator(this); }

	Iterator end() { return Iterator(this).SetEnd(); }

private:
	HashMap<KeyUIntType, ValueType> sparse;
	std::vector<ValueType> dense;
	const KeyUIntType denseRange;
	KeyUIntType size = 0;
};
} // namespace spp
