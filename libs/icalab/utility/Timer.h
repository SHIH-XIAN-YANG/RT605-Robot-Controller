/*
 * Timer
 * Licensed under Apache
 *
 * Author: KaiWen <wenkai1987@gmail.com>
 * Date: Apr-16-2013
 *
 */
#ifndef TIMER_H
#define TIMER_H

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/unordered_map.hpp>

typedef boost::asio::deadline_timer* timer_ptr;

namespace bs = boost::system;

class timer;
class timer_node;

class tm_callback {
public:
    explicit tm_callback(boost::function<void(timer_node&)>& f) : m_f(f)
    {
    }

    void operator()(timer_node& node, const bs::error_code& e) {
        if (!e)
            m_f(node);
    }
private:
    boost::function<void(timer_node&)> m_f;
};

class timer_node {
    friend class timer;
public:
    timer_node() {}
    timer_node(timer_ptr p, int ms, boost::function<void(timer_node&)> f) :
        m_tptr(p), m_ms(ms), m_callback(f)
    {
    }

    void reset(unsigned int ms = 0, boost::function<void(timer_node&)> f = 0) {
        if (ms)
            m_tptr->expires_from_now(boost::posix_time::milliseconds(ms));
        else
            m_tptr->expires_from_now(boost::posix_time::milliseconds(m_ms));

        if (f)
            m_tptr->async_wait(boost::bind<void>(tm_callback(f), *this, _1));
        else
            m_tptr->async_wait(boost::bind<void>(tm_callback(m_callback), *this, _1));
    }
private:
    timer_ptr m_tptr;
    int m_ms;
    boost::function<void(timer_node&)> m_callback;
};