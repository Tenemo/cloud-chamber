/**
 * @file RingBuffer.h
 * @brief Simple fixed-size ring buffer template
 */

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <cstddef>
#include <utility>

template <typename T, size_t N> class RingBuffer {
  public:
    RingBuffer() : _head(0), _count(0) {}

    void push(const T &item) {
        _data[_head] = item;
        _head = (_head + 1) % N;
        if (_count < N)
            ++_count;
    }

    size_t size() const { return _count; }
    constexpr size_t capacity() const { return N; }

    /**
     * @brief Get element counted from newest (0 = most recent)
     * @return pointer or nullptr if out of range
     */
    const T *getFromNewest(size_t idx) const {
        if (idx >= _count)
            return nullptr;
        size_t pos = (_head + N - 1 - idx) % N;
        return &_data[pos];
    }

  private:
    T _data[N];
    size_t _head;
    size_t _count;
};

// ---------------------------------------------------------------------------
// RingBufferView: same API but uses external storage (e.g., PSRAM)
// ---------------------------------------------------------------------------
template <typename T> class RingBufferView {
  public:
    RingBufferView() : _data(nullptr), _cap(0), _head(0), _count(0) {}
    RingBufferView(T *storage, size_t cap)
        : _data(storage), _cap(cap), _head(0), _count(0) {}

    void reset(T *storage, size_t cap) {
        _data = storage;
        _cap = cap;
        _head = 0;
        _count = 0;
    }

    bool valid() const { return _data != nullptr && _cap > 0; }

    void push(const T &item) {
        if (!valid())
            return;
        _data[_head] = item;
        _head = (_head + 1) % _cap;
        if (_count < _cap)
            ++_count;
    }

    size_t size() const { return _count; }
    size_t capacity() const { return _cap; }

    const T *getFromNewest(size_t idx) const {
        if (!valid() || idx >= _count)
            return nullptr;
        size_t pos = (_head + _cap - 1 - idx) % _cap;
        return &_data[pos];
    }

  private:
    T *_data;
    size_t _cap;
    size_t _head;
    size_t _count;
};

#endif // RINGBUFFER_H
