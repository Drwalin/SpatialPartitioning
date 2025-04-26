// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "NodesArray.hpp"
#include "DenseSparseIntMap.hpp"

namespace spp
{
/*
 * Offsets start from 1
 */
template <typename KeyType, typename OffsetType, typename ValueType,
		  bool enableDense = false>
class AssociativeArray final
{
public:
	inline AssociativeArray(KeyType denseRange = 0) : offsets(denseRange)
	{
		Clear();
	}
	inline ~AssociativeArray() {}

	inline AssociativeArray(const AssociativeArray &) = default;
	inline AssociativeArray(AssociativeArray &) = default;
	inline AssociativeArray(AssociativeArray &&) = default;

	inline OffsetType Add(KeyType key, ValueType &&value)
	{
		if (offsets.Has(key)) {
			return 0;
		} else {
			OffsetType offset = data.Add(std::move(value));
			offsets.Set(key, offset);
			return offset;
		}
	}

	inline void RemoveByKey(KeyType key)
	{
		auto it = offsets.find(key);
		if (it == nullptr) {
			return;
		}
		OffsetType offset = *it;
		offsets.Remove(key);
		data.Remove(offset);
	}

	inline void Clear()
	{
		offsets.Clear();
		data.Clear();
	}

	inline void ShrinkToFit()
	{
		offsets.ShrinkToFit();
		data.ShrinkToFit();
	}

	inline void Reserve(OffsetType capacity)
	{
		offsets.Reserve(capacity);
		data.reserve(capacity);
	}

	inline OffsetType Size() const { return offsets.Size(); }

	inline OffsetType GetOffset(KeyType key) const
	{
		auto it = offsets.find(key);
		if (it)
			return *it;
		return 0;
	}

	inline void UpdateKeyOffset(KeyType key, OffsetType newOffset)
	{
		offsets[key] = newOffset;
	}

	inline ValueType &operator[](OffsetType offset) { return data[offset]; }
	inline const ValueType &operator[](OffsetType offset) const
	{
		return data[offset];
	}

	inline size_t GetMemoryUsage() const
	{
		return offsets.GetMemoryUsage() + data.GetMemoryUsage();
	}

	DenseSparseIntMap<KeyType, OffsetType, enableDense> &_Offsets()
	{
		return offsets;
	}
	NodesArray<OffsetType, ValueType> &_Data() { return data; }

private:
	DenseSparseIntMap<KeyType, OffsetType, enableDense> offsets;
	NodesArray<OffsetType, ValueType> data;
};
} // namespace spp
