#ifndef SBARRIER_H
#define SBARRIER_H

#include<thread>
#include<mutex>
#include<condition_variable>

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


#endif 