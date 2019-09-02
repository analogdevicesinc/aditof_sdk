#ifndef SAFEQUEUE_H
#define SAFEQUEUE_H

#include <condition_variable>
#include <mutex>
#include <queue>

template <class T>
class SafeQueue {
  public:
    SafeQueue() {}
    ~SafeQueue() {}

    void enqueue(T element) {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_queue.push(element);
        lock.unlock();
        m_cv.notify_one();
    }

    T dequeue() {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_cv.wait(lock, [&] { return !empty(); });
        T element = m_queue.front();
        m_queue.pop();
        return element;
    }

    bool empty() const { return m_queue.empty(); }

  private:
    std::queue<T> m_queue;
    std::mutex m_mutex;
    std::condition_variable m_cv;
};

#endif // SAFEQUEUE_H
