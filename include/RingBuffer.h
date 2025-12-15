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

#endif // RINGBUFFER_H
