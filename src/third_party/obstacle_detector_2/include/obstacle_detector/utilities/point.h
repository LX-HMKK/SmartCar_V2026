/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#pragma once

#include <cmath>
#include <iostream>

namespace obstacle_detector
{

class Point
{
public:
  Point(double x = 0.0, double y = 0.0, double range = 0.0, double z = 0.0) : x(x), y(y), range(range), z(z) {}
  Point(const Point& p) : x(p.x), y(p.y), range(p.range), z(p.z) {}
  static Point fromPoolarCoords(const double r, const double phi) { return Point(r * cos(phi), r * sin(phi)); }

  double getRange()      const { return range > 0.0 ? range : length(); }
  double length()        const { return sqrt(pow(x, 2.0) + pow(y, 2.0)); }
  double lengthSquared() const { return pow(x, 2.0) + pow(y, 2.0); }
  double angle()         const { return atan2(y, x); }
  double angleDeg()      const { return 180.0 * atan2(y, x) / M_PI; }
  double dot(const Point& p)   const { return x * p.x + y * p.y; }
  double cross(const Point& p) const { return x * p.y - y * p.x; }

  Point normalized() { return (length() > 0.0) ? *this / length() : *this; }
  Point reflected(const Point& normal) const { return *this - 2.0 * normal * (normal.dot(*this)); }
  Point perpendicular() const { return Point(-y, x, 0., z); }

  friend Point operator+ (const Point& p1, const Point& p2) { return Point(p1.x + p2.x, p1.y + p2.y, 0., p1.z); }
  friend Point operator- (const Point& p1, const Point& p2) { return Point(p1.x - p2.x, p1.y - p2.y, 0., p1.z); }
  friend Point operator* (const double f, const Point& p)  { return Point(f * p.x, f * p.y, 0., p.z); }
  friend Point operator* (const Point& p, const double f)  { return Point(f * p.x, f * p.y, 0., p.z); }
  friend Point operator/ (const Point& p, const double f)  { return (f != 0.0) ? Point(p.x / f, p.y / f, 0., p.z) : Point(); }

  Point operator- () { return Point(-x, -y, 0, z); }
  Point operator+ () { return Point( x,  y, 0, z); }

  Point& operator=  (const Point& p) { if (this != &p) { x = p.x; y = p.y; range = p.range; z = p.z;} return *this; }
  Point& operator+= (const Point& p) { x += p.x; y += p.y; return *this; }
  Point& operator-= (const Point& p) { x -= p.x; y -= p.y; return *this; }

  friend bool operator== (const Point& p1, const Point& p2) { return (p1.x == p2.x && p1.y == p2.y); }
  friend bool operator!= (const Point& p1, const Point& p2) { return !(p1 == p2); }
  friend bool operator<  (const Point& p1, const Point& p2) { return (p1.lengthSquared() < p2.lengthSquared()); }
  friend bool operator<= (const Point& p1, const Point& p2) { return (p1.lengthSquared() <= p2.lengthSquared()); }
  friend bool operator>  (const Point& p1, const Point& p2) { return (p1.lengthSquared() > p2.lengthSquared()); }
  friend bool operator>= (const Point& p1, const Point& p2) { return (p1.lengthSquared() >= p2.lengthSquared()); }
  friend bool operator!  (const Point& p1) { return (p1.x == 0.0 && p1.y == 0.0); }

  friend std::ostream& operator<<(std::ostream& out, const Point& p)
  { out << "(" << p.x << ", " << p.y << ", " << p.z << ") with range " << p.getRange() << " to origin"; return out; }

  double x;
  double y;
  double z;
  double range = 0.0; // Distance w.r.t. lidar scan origin
};

} // namespace obstacle_detector
