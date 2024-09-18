// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <unordered_map>

#include "NodesArray.hpp"

namespace spp
{
/*
 * Offsets start from 1
 */
template <typename KeyType, typename OffsetType, typename ValueType>
class AssociativeArray final
{
public:
	inline AssociativeArray() { Clear(); }
	inline ~AssociativeArray() {}

	inline AssociativeArray(const AssociativeArray &) = default;
	inline AssociativeArray(AssociativeArray &) = default;
	inline AssociativeArray(AssociativeArray &&) = default;

	inline OffsetType Add(KeyType key, ValueType &&value)
	{
		if (offsets.find(key) != offsets.end()) {
			return -1;
		} else {
			OffsetType offset = data.Add(std::move(value));
			offsets[key] = offset;
			return offset;
		}
	}

	inline void RemoveByKey(KeyType key)
	{
		auto it = offsets.find(key);
		if (it == offsets.end())
			return;
		OffsetType offset = it->second;
		offsets.erase(it);
		data.Remove(offset);
	}

	inline void Clear()
	{
		data.Clear();
		offsets.clear();
	}

	inline void ShrinkToFit()
	{
		offsets.shrink_to_fit();
		data.ShrinkToFit();
	}

	inline void Reserve(OffsetType capacity)
	{
		offsets.reserve(capacity);
		data.reserve(capacity);
	}

	inline OffsetType Size() const { return offsets.size(); }

	inline OffsetType GetOffset(KeyType key) const
	{
		auto it = offsets.find(key);
		if (it == offsets.end())
			return -1;
		return it->second;
	}

	inline ValueType &operator[](OffsetType offset) { return data[offset]; }

	inline size_t GetMemoryUsage() const
	{
		return offsets.bucket_count() * sizeof(void *) +
			   offsets.size() * (sizeof(void *) * 2lu + sizeof(OffsetType) +
								 sizeof(KeyType)) +
			   data.GetMemoryUsage();
	}

private:
	std::unordered_map<KeyType, OffsetType> offsets;
	NodesArray<OffsetType, ValueType> data;
};
} // namespace spp
