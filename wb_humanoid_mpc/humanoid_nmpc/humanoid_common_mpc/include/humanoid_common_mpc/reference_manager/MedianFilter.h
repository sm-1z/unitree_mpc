#pragma once

#include <humanoid_common_mpc/common/Types.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <deque>
#include <vector>

namespace ocs2::humanoid {

/**
 * 中值 + 一阶低通滤波组合器
 */

class MedianFilter final {
   public:
    /**
     * 构造函数
     * @param breakFrequency 低通滤波器截止频率 [Hz]
     * @param windowSize 中值滤波器窗口大小（必须为奇数）
     * @param y_init 初始状态
     */
    MedianFilter(scalar_t breakFrequency, size_t windowSize,
                 const vector_t& y_init)
        : breakDeltaT_(1 / (2 * M_PI * breakFrequency)),
          y_last_(y_init),
          windowSize_(windowSize),
          buffer_(y_init.size()) {
        assert(windowSize_ % 2 == 1);
        for (size_t i = 0; i < y_init.size(); ++i) {
            for (size_t j = 0; j < windowSize_; ++j) {
                buffer_[i].push_back(y_init(i));
            }
        }
        lastTimeFilterCalled_ = std::chrono::steady_clock::now();
    }

    /**
     * 对输入向量执行中值滤波
     */
    vector_t getFilteredVector(const vector_t& x) {
        assert(x.size() == y_last_.size());

        // 1. 中值滤波（消除突变值）
        vector_t medianFiltered(x.size());
        for (size_t i = 0; i < x.size(); ++i) {
            auto& queue = buffer_[i];
            queue.push_back(x(i));
            if (queue.size() > windowSize_) queue.pop_front();
            std::vector<scalar_t> temp(queue.begin(), queue.end());
            std::nth_element(temp.begin(), temp.begin() + temp.size() / 2,
                             temp.end());
            medianFiltered(i) = temp[temp.size() / 2];
        }

        // 2. 一阶低通滤波（平滑变化）
        scalar_t alpha = computeAlpha();
        y_last_ = alpha * medianFiltered + (1 - alpha) * y_last_;
        return y_last_;
    }

   private:
    scalar_t computeAlpha() {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> dt = now - lastTimeFilterCalled_;
        lastTimeFilterCalled_ = now;
        return dt.count() / (dt.count() + breakDeltaT_);
    }

    scalar_t breakDeltaT_;
    vector_t y_last_;
    size_t windowSize_;
    std::vector<std::deque<scalar_t>> buffer_;
    std::chrono::time_point<std::chrono::steady_clock> lastTimeFilterCalled_;
};


}  // namespace ocs2::humanoid
