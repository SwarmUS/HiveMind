#ifndef __THREADSAFEQUEUE_H_
#define __THREADSAFEQUEUE_H_

#include <cpp-common/ICircularQueue.h>
#include <freertos-utils/LockGuard.h>

// TODO: move in another lib, it doesn't really belong here
/**
 *@brief A queue that is thread safe */
template <typename T>
class ThreadSafeQueue : public ICircularQueue<T> {
  public:
    ThreadSafeQueue(ICircularQueue<T>& queue) : m_queue(queue), m_mutex(10) {}

    ~ThreadSafeQueue() override = default;

    bool push(const T& item) override {
        LockGuard lock(m_mutex);
        return m_queue.push(item);
    }

    const std::optional<std::reference_wrapper<const T>> peek() const override {
        Mutex* mutableLock = const_cast<Mutex*>(&m_mutex);
        LockGuard lock(*mutableLock);
        return m_queue.peek();
    }

    void pop() override {
        LockGuard lock(m_mutex);
        return m_queue.pop();
    }

    void clear() override {
        LockGuard lock(m_mutex);
        return m_queue.clear();
    }

    bool isFull() const override {
        Mutex* mutableLock = const_cast<Mutex*>(&m_mutex);
        LockGuard lock(*mutableLock);
        return m_queue.isFull();
    }

    bool isEmpty() const override {
        Mutex* mutableLock = const_cast<Mutex*>(&m_mutex);
        LockGuard lock(*mutableLock);
        return m_queue.isEmpty();
    }

    uint16_t getLength() const override {
        Mutex* mutableLock = const_cast<Mutex*>(&m_mutex);
        LockGuard lock(*mutableLock);
        return m_queue.getLength();
    };

    uint16_t getFreeSize() const override {
        Mutex* mutableLock = const_cast<Mutex*>(&m_mutex);
        LockGuard lock(*mutableLock);
        return m_queue.getFreeSize();
    }

    std::optional<std::reference_wrapper<T>> getNextAllocation() override {
        LockGuard lock(m_mutex);
        return m_queue.getNextAllocation();
    }

    bool advance() override {
        LockGuard lock(m_mutex);
        return m_queue.advance();
    }

  private:
    ICircularQueue<T>& m_queue;
    Mutex m_mutex;
};

#endif // __THREADSAFEQUEUE_H_
