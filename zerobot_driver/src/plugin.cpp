// Copyright 2018 Chengzhi Chen

#include <zerobot_driver/plugin.h>
#include <string>
#include <vector>

namespace zerobot_driver
{
std::array<Plugin::Signal, CALLBACK_COUNT> Plugin::callbacks_;
boost::mutex Plugin::buffer_mutex_;
boost::condition_variable Plugin::buffer_cv_;
boost::circular_buffer<Plugin::Buffer> Plugin::buffer_(BUFFER_COUNT);
};  // namespace zerobot_driver
