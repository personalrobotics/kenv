#ifndef TIMER_H_
#define TIMER_H_

#include <boost/date_time/posix_time/posix_time.hpp>

namespace kenv {

class Timer {
public:
    Timer()
        : begin_(boost::posix_time::microsec_clock::local_time())
    {
    }
    
    void Reset()
    {
        begin_ = boost::posix_time::microsec_clock::local_time();
    }

    double time_ellapsed() const
    {
        boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
        boost::posix_time::time_duration duration = now - begin_;
        return static_cast<double>(duration.total_nanoseconds()) / 1e9l;
    }

private:
    boost::posix_time::ptime begin_;
};

}

#endif
