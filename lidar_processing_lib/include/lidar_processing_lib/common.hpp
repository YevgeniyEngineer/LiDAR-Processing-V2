/*
 * Copyright (c) 2024 Yevgeniy Simonov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef LIDAR_PROCESSING_LIB__COMMON_HPP
#define LIDAR_PROCESSING_LIB__COMMON_HPP

// STL
#include <cmath>

namespace lidar_processing_lib
{
/// @brief Approximate arctan2 function with a maximum relative error ~3.6e-5
/// https://stackoverflow.com/questions/46210708/atan2-approximation-with-11bits-in-mantissa-on-x86with-sse2-and-armwith-vfpv4
inline constexpr float atan2Approx(const float y, const float x) noexcept
{
    const float ax = std::fabs(x);
    const float ay = std::fabs(y);
    const float mx = std::max(ay, ax);
    const float mn = std::min(ay, ax);
    const float a = mn / mx;
    /* Minimax polynomial approximation to atan(a) on [0,1] */
    const float s = a * a;
    const float c = s * a;
    const float q = s * s;
    float r = 0.024840285F * q + 0.18681418F;
    const float t = -0.094097948F * q - 0.33213072F;
    r = r * s + t;
    r = r * c + a;
    /* Map to full circle */
    if (ay > ax)
    {
        r = 1.57079637F - r;
    }
    if (x < 0)
    {
        r = 3.14159274F - r;
    }
    if (y < 0)
    {
        r = -r;
    }
    return r;
}

} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__COMMON_HPP
