// This file is part of SpatialPartitioning.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <type_traits>
#include <cstdlib>
#include <cstdint>
#include <utility>
#include <cstring>

namespace spp
{
template <typename T, typename TSize = uint32_t,
		  bool TRIVIALLY_COPYABLE = std::is_trivially_copyable_v<T>,
		  bool HAS_CONSTRUCTOR = true>
class Array
{
public:
	T *data = nullptr;
	TSize size = 0;
	TSize capacity = 0;

public:
	inline Array() {}

	inline ~Array()
	{
		if (data) {
			if constexpr (!TRIVIALLY_COPYABLE) {
				for (auto &it : *this) {
					it.~T();
				}
			}
			free(data);
			data = nullptr;
			size = 0;
			capacity = 0;
		}
	}

	inline Array(Array &&o) { this->operator=(std::move(o)); }
	inline Array(const Array &o)
	{
		size = capacity = o.size;
		data = (T *)malloc(sizeof(T) * capacity);
		CopyToUninited(data, o.data, size);
	}

	inline Array &operator=(Array &&o)
	{
		data = o.data;
		o.data = nullptr;
		size = o.size;
		o.size = 0;
		capacity = o.capacity;
		o.capacity = 0;
		return *this;
	}
	inline Array &operator=(const Array &o)
	{
		if (capacity < o.size) {
			this->~Array();
			new (this) Array(o);
		} else {
			if (size > o.size) {
				Destroy(data + o.size, size - o.size);
			}
			CopyToInited(data, o.data, o.size);
			size = o.size;
		}
		return *this;
	}

	inline T &operator[](TSize id) { return data[id]; }
	inline const T &operator[](TSize id) const { return data[id]; }

	inline T *begin() { return data; }
	inline T *end() { return data + size; }

	inline const T *begin() const { return data; }
	inline const T *end() const { return data + size; }

	inline void push_back(T &&o)
	{
		reserve(size + 1);
		data[size] = std::move(o);
		++size;
	}
	inline void push_back(const T &o)
	{
		reserve(size + 1);
		data[size] = o;
		++size;
	}

	inline void resize(TSize newSize)
	{
		if (newSize > size) {
			reserve(newSize);
			Construct(data + size, newSize - size);
		} else if (newSize < size) {
			Destroy(data + newSize, size - newSize);
		}
		size = newSize;
	}

	inline void reserve(TSize newCapacity)
	{
		if (newCapacity > capacity) {
			if (newCapacity > (capacity * 3) / 2) {
				capacity = newCapacity;
			} else {
				capacity = (capacity * 3) / 2;
			}
			if (data) {
				T *newData = (T *)malloc(sizeof(T) * capacity);
				MoveToUninited(newData, data, size);
				free(data);
				data = newData;
			} else {
				data = (T *)malloc(sizeof(T) * capacity);
			}
		}
	}

	inline void clear()
	{
		if (size) {
			Destroy(data, size);
			size = 0;
		}
	}

	inline void shrink_to_fit()
	{
		if (capacity != size) {
			capacity = size;
			T *newData = (T *)malloc(sizeof(T) * capacity);
			MoveToUninited(newData, data, size);
			free(data);
		}
	}

private:
	inline void Construct(T *ptr, TSize count)
	{
		if constexpr (HAS_CONSTRUCTOR) {
			for (TSize i = 0; i < count; ++i) {
				new (ptr + i) T();
			}
		}
	}

	inline void MoveToUninited(T *dst, T *src, TSize count)
	{
		if constexpr (!TRIVIALLY_COPYABLE) {
			for (TSize i = 0; i < count; ++i) {
				new (dst + i) T(std::move(src[i]));
			}
		} else {
			memcpy(dst, src, count * sizeof(T));
		}
	}

	inline void CopyToInited(T *dst, const T *src, TSize count)
	{
		if constexpr (!TRIVIALLY_COPYABLE) {
			for (TSize i = 0; i < count; ++i) {
				dst[i] = src[i];
			}
		} else {
			memcpy(dst, src, count * sizeof(T));
		}
	}

	inline void CopyToUninited(T *dst, T *src, TSize count)
	{
		if constexpr (!TRIVIALLY_COPYABLE) {
			for (TSize i = 0; i < count; ++i) {
				new (dst + i) T(src[i]);
			}
		} else {
			memcpy(dst, src, count * sizeof(T));
		}
	}

	inline void Destroy(T *ptr, TSize count)
	{
		if constexpr (!TRIVIALLY_COPYABLE) {
			for (TSize i = 0; i < count; ++i) {
				ptr[i].~T();
			}
		}
	}
};
} // namespace spp
