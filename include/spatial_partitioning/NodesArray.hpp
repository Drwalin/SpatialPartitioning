// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <vector>
#include <queue>

namespace spp
{
template <class T, class S, class C>
S &GetContainer(std::priority_queue<T, S, C> &q)
{
	struct HackedQueue : private std::priority_queue<T, S, C> {
		static S &Container(std::priority_queue<T, S, C> &q)
		{
			return q.*&HackedQueue::c;
		}
	};
	return HackedQueue::Container(q);
}

template <class T, class S, class C>
const S &GetContainer(const std::priority_queue<T, S, C> &q)
{
	struct HackedQueue : private std::priority_queue<T, S, C> {
		static const S &Container(const std::priority_queue<T, S, C> &q)
		{
			return q.*&HackedQueue::c;
		}
	};
	return HackedQueue::Container(q);
}

/*
 * Offsets start from 1
 */
template <typename OffsetType, typename ValueType> class NodesArray final
{
public:
	inline NodesArray() { Clear(); }
	inline ~NodesArray() {}

	inline NodesArray(const NodesArray &) = default;
	inline NodesArray(NodesArray &) = default;
	inline NodesArray(NodesArray &&) = default;

	inline OffsetType Add(ValueType &&value)
	{
		if (freeOffsets.size() > 0) {
			OffsetType offset = freeOffsets.top();
			freeOffsets.pop();
			data[offset] = std::move(value);
			return offset;
		} else {
			OffsetType offset = data.size();
			data.push_back(value);
			return offset;
		}
	}

	inline void Remove(OffsetType offset)
	{
		if (offset == data.size() - 1) {
			data.resize(data.size() - 1);
		} else {
			data[offset] = {};
			freeOffsets.push(offset);
		}
	}

	inline void Clear()
	{
		GetContainer(freeOffsets).clear();
		data.resize(1);
	}

	inline void ShrinkToFit()
	{
		data.shrink_to_fit();
		GetContainer(freeOffsets).shrink_to_fit();
	}

	inline void Reserve(OffsetType capacity) { data.reserve(capacity); }

	inline OffsetType Size() const
	{
		return data.size() - freeOffsets.size() - 1;
	}

	inline ValueType &operator[](OffsetType offset) { return data[offset]; }
	inline const ValueType &operator[](OffsetType offset) const
	{
		return data[offset];
	}

	inline size_t GetMemoryUsage() const
	{
		return data.capacity() * sizeof(ValueType) +
			   GetContainer(freeOffsets).capacity() * sizeof(OffsetType);
	}

	std::vector<ValueType> &_Data() { return data; }
	std::priority_queue<OffsetType, std::vector<OffsetType>,
						std::greater<OffsetType>> &
	_FreeOffsets()
	{
		return freeOffsets;
	}

private:
	std::vector<ValueType> data;
	std::priority_queue<OffsetType, std::vector<OffsetType>,
						std::greater<OffsetType>>
		freeOffsets;
};
} // namespace spp
