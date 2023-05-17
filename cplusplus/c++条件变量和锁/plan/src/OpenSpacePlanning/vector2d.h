/////////////////////////////////////////////////////////////////////////
// Copyright (C) 2021-2026, by Motovis All Rights Reserved.
//
// 本代码仅用于魔视智能与广州汽车集团股份有限公司汽车工程研究院合作的
// X3V项目（以下简称本项目），魔视智能对本代码及基于本代码开发产生的
// 所有内容拥有全部知识产权，任何人不得侵害或破坏，未经魔视智能授权许可
// 或其他法律认可的方式，任何企业或个人不得随意复制、分发、下载和使用，
// 以及用于非本项目的其他商业用途。
//
// 本代码仅供指定接收人（包括但不限于      ）在魔视智能授权范围内使用，
// 指定接收人必须征得魔视智能授权，才可在软件库中加入本代码。
//
// 本代码是受法律保护的保密信息，如您不是指定接收人，请立即将本代码删除，
// 法律禁止任何非法的披露、或以任何方式使用本代码。指定接收人应对本代码
// 保密信息负有保密义务，未经允许，不得超出本项目约定的披露、复制、传播
// 或允许第三方披露、复制、传播本代码部分或全部信息。
//
//
/////////////////////////////////////////////////////////////////////////
/**
* @file vector2d.h
* @copyright 版权所有 2020 魔视智能科技(上海)有限公司
* @brief 定义vector2d
* @version V2.0
* @author  xingyu zhang
* @date 2020年7月2日
* @note
*/

#ifndef VECTOR2D
#define VECTOR2D

#include <cmath>
#if PLATFORM==PC_SYSTEM
#else
#include <iostream>
#endif

namespace HybridAStarPart {

class Vector2D {
 public:
  inline Vector2D(const double x = 0, const double y = 0) { this->x = x; this->y = y; }

  void setX(const double& x) { this->x = x; }

  void setY(const double& y) { this->y = y; }

  void setXY(const double& x, const double& y) { this->x = x; this->y = y; }

  inline Vector2D operator * (const double k) const { return Vector2D(x * k, y * k); }

  inline Vector2D operator / (const double k) const { return Vector2D(x / k, y / k); }

  inline Vector2D operator + (const Vector2D& b) const { return Vector2D(x + b.x, y + b.y); }

  inline Vector2D operator - (const Vector2D& b) const { return Vector2D(x - b.x, y - b.y); }

  inline Vector2D operator - () const  {return Vector2D(-x, -y);}

  double length() const { return std::sqrt(std::pow(x, 2) + std::pow(y, 2)); }

  double sqlength() const { return x*x + y*y; }

  double dot(Vector2D b)
  {
      return x * b.x + y * b.y;
  }

  double cross(Vector2D b)
  {
      return x * b.y - y * b.x;
  }

  inline Vector2D ort(Vector2D b)
  {
    Vector2D a(this->x, this->y);
    Vector2D c;
    c = a - b * a.dot(b) / b.sqlength();
    return c;
  }

  inline double getX() const { return x;}

  inline double getY() const { return y;}

private:
  double x;
  double y;
};
inline Vector2D operator * (double k, const Vector2D& b) {
  return (b * k);
}
}
#endif // VECTOR2D
