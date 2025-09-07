#ifndef BOUNDED_QUEUE_H
#define BOUNDED_QUEUE_H

#include <deque>
#include <mutex>
#include <cstddef>
#include <utility>
#include <stdexcept>

template <typename T>
class BoundedQueue {
public:
    explicit BoundedQueue(size_t capacity = 50)
        : maxSize(capacity) {}

    // Standard queue behavior: throws if full
    void push(const T& item) {
        std::lock_guard<std::mutex> lock(mutex);
        if (queue.size() >= maxSize) {
            throw std::overflow_error("BoundedQueue is full");
        }
        queue.push_back(item);
    }

    void push(T&& item) {
        std::lock_guard<std::mutex> lock(mutex);
        if (queue.size() >= maxSize) {
            throw std::overflow_error("BoundedQueue is full");
        }
        queue.push_back(std::move(item));
    }

    // Bounded push: drops oldest if full
    void push_drop_oldest(T&& item) {
        std::lock_guard<std::mutex> lock(mutex);
        if (queue.size() >= maxSize) {
            queue.pop_front(); // Drop oldest
        }
        queue.push_back(std::move(item));
    }

    // Standard pop method (removes front, no return)
    void pop() {
        std::lock_guard<std::mutex> lock(mutex);
        if (!queue.empty()) {
            queue.pop_front();
        }
    }

    // Access front element (throws if empty)
    T& front() {
        std::lock_guard<std::mutex> lock(mutex);
        if (queue.empty()) {
            throw std::underflow_error("Queue is empty");
        }
        return queue.front();
    }

    const T& front() const {
        std::lock_guard<std::mutex> lock(mutex);
        if (queue.empty()) {
            throw std::underflow_error("Queue is empty");
        }
        return queue.front();
    }

    // Try to pop and get the front element atomically
    bool try_pop(T& out) {
        std::lock_guard<std::mutex> lock(mutex);
        if (queue.empty()) return false;
        out = std::move(queue.front());
        queue.pop_front();
        return true;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex);
        return queue.empty();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex);
        return queue.size();
    }

private:
    size_t maxSize;
    mutable std::mutex mutex;
    std::deque<T> queue;
};

#endif // BOUNDED_QUEUE_H
