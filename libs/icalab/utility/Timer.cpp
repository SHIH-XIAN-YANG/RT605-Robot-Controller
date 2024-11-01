/*
 * Timer
 *
 * Licensed under Apache
 *
 * Author: KaiWen <wenkai1987@gmail.com>
 * Date: Apr-16-2013
 *
 */
#include "timer.h"
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/ptime.hpp>

namespace ba = boost::asio;

timer::timer(int thread_num) : m_next_ios(0), m_size(0) {
    for (int i = 0; i < thread_num; i++) {
        io_service_ptr p(new ba::io_service);
        work_ptr pw(new ba::io_service::work(*p));
        m_ios_list.push_back(p);
        m_works.push_back(pw);
    }

    pthread_spin_init(&m_lock, 0);
}

timer::~timer() {
    pthread_spin_destroy(&m_lock);
}

void timer::run() {
    for (size_t i = 0; i < m_ios_list.size(); i++)
        m_threads.create_thread(boost::bind(&ba::io_service::run, &*m_ios_list[i]))->detach();
}