#ifndef SBARRIER_H
#define SBARRIER_H

#include<thread>
#include<mutex>
#include<condition_variable>
#include<atomic>
#include<exception>

#define OLD_BARRIER
#ifdef OLD_BARRIER

class barrier
{
public:
	barrier(unsigned int);
	bool wait();
private:
	std::mutex m_mutex;
	std::condition_variable m_cond;
	unsigned int m_threshold;
	unsigned int m_count;
	unsigned int m_generation;
};
inline barrier::barrier(unsigned int count)
		: m_threshold(count), m_count(count), m_generation(0)
{
	if (count == 0)
		throw std::invalid_argument("count cannot be zero.");
}
inline bool barrier::wait()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	unsigned int gen = m_generation;

	if (--m_count == 0)
	{
		m_generation++;
		m_count = m_threshold;
		m_cond.notify_all();
		return true;
	}

	while (gen == m_generation)
		m_cond.wait(lock);
	return false;
}

#else
//Use lock-free barrier (hardware atomic counters means its a hardware lock
class barrier
{
public:
	barrier(unsigned int);
	bool wait();
private:
	std::atomic<unsigned int> m_count;
	std::atomic<unsigned int> m_generation;
	unsigned int m_threshold;
};

inline barrier::barrier(unsigned int count)
		: m_threshold(count), m_count(count), m_generation(0)
{
	if (count == 0)
		throw std::invalid_argument("count cannot be zero.");
}
inline bool barrier::wait()
{
	unsigned int gen = m_generation;

	if (--m_count == 0)
	{
		m_generation++;
		m_count = m_threshold;
		return true;
	}

	while (gen == m_generation);
	return false;
}
#endif



#endif 
