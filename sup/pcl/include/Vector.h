/*
 This is utility to that provides converter to convert k4a::image to cv::Mat.

 cv::Mat mat = k4a::get_mat( image );

 Copyright (c) 2019 Tsukasa Sugiura <t.sugiura0204@gmail.com>
 Licensed under the MIT license.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
*/

#pragma once
#include <vector>
#include <limits>

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>


namespace k4a
{

    struct Vector
    {
        float X;
        float Y;
        float Z;

        Vector() // default constructor
        {
        }

        Vector(float x, float y, float z)
                : X(x)
                , Y(y)
                , Z(z)
        {
        }

        Vector(const k4a_float3_t& v)
                : X(v.xyz.x)
                , Y(v.xyz.y)
                , Z(v.xyz.z)
        {
        }

        float Dot(const Vector& other) const
        {
            return X * other.X + Y * other.Y + Z * other.Z;
        }

        float SquareLength() const
        {
            return X * X + Y * Y + Z * Z;
        }

        float Length() const
        {
            return std::sqrt(SquareLength());
        }

        Vector operator*(float c) const
        {
            return { X * c, Y * c, Z * c };
        }

        Vector operator/(float c) const
        {
            return *this * (1 / c);
        }

        Vector Normalized() const
        {
            return *this / Length();
        }

        float Angle(const Vector& other) const
        {
            return std::acos(Dot(other) / Length() / other.Length());
        }
    };
}

