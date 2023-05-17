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



#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#define __STD_C99

/* Note: 为了避免与系统定义的types冲突, 我们统一采用大写字母开头的组合, 不带后缀 _t 
 * 若带有volatile关键字的, 用小写v开头, 后续部分采用一致定义. 
 * 对volatile的书写不做强制要求, 写成vUInt32/vUint32/volatile Uint32/volatile UInt32均可
 * 
 * 对于无符号数, Uint32/vUint32  UInt32/vUInt32 均支持. 
 *
 * 特别注意Int8类型和Char类型的区分, c++ 不允许signed char 或 unsigned char 与 char 的相互强换.
 */

#ifdef __STD_C99__
#include <stdint.h>
/* using ISO C99 standard */

typedef volatile int8_t  vInt8;
typedef volatile uint8_t vUint8;
typedef volatile uint8_t vUInt8;
typedef volatile int8_t  VINT8;
typedef volatile uint8_t VUINT8;

typedef volatile int16_t  vInt16;
typedef volatile uint16_t vUint16;
typedef volatile uint16_t vUInt16;
typedef volatile int16_t  VINT16;
typedef volatile uint16_t VUINT16;

typedef volatile int32_t  vInt32;
typedef volatile uint32_t vUint32;
typedef volatile uint32_t vUInt32;
typedef volatile int32_t  VINT32;
typedef volatile uint32_t VUINT32;

typedef volatile int64_t  vInt64;
typedef volatile uint64_t vUint64;
typedef volatile uint64_t vUInt64;
typedef volatile int64_t  VINT64;
typedef volatile uint64_t VUINT64;


/*
 * C++规定 char / unsigned char / signed char 为3种不同类型
 * 字符串默认是(char *)类型, 无法强转为(signed char *)类型
 * 为了兼容这个特性, 特意封装了Char类型, 用于描述字符串和字符
 */	

typedef int8_t  Int8;
typedef uint8_t Uint8;
typedef uint8_t UInt8;
typedef int8_t  INT8;
typedef uint8_t UINT8;

typedef int16_t  Int16;
typedef uint16_t Uint16;
typedef uint16_t UInt16;
typedef int16_t  INT16;
typedef uint16_t UINT16;

typedef int32_t  Int32;
typedef uint32_t Uint32;
typedef uint32_t UInt32;
typedef int32_t  INT32;
typedef uint32_t UINT32;

typedef int64_t  Int64;
typedef uint64_t Uint64;
typedef uint64_t UInt64;
typedef int64_t  INT64;
typedef uint64_t UINT64;

#else /* 不支持C99标准, 即无法包含<stdint.h> */

typedef signed char   Int8;
typedef unsigned char Uint8;
typedef unsigned char UInt8;
typedef signed char   INT8;
typedef unsigned char UINT8;

typedef signed short   Int16;
typedef unsigned short Uint16;
typedef unsigned short UInt16;
typedef signed short   INT16;
typedef unsigned short UINT16;

typedef signed int   Int32;
typedef unsigned int Uint32;
typedef unsigned int UInt32;
typedef signed int   INT32;
typedef unsigned int UINT32;

typedef signed char			Int8_t;
typedef unsigned char		UInt8_t;
typedef signed short		Int16_t;
typedef unsigned short		UInt16_t;
typedef signed int			Int32_t;
typedef unsigned int		UInt32_t;
typedef long long			Int64_t;
typedef unsigned long long	UInt64_t;
typedef float				Float32_t;
typedef double				Float64_t;
typedef void*				Handle;
typedef float              float32_t;
typedef double             float64_t;


typedef signed long long   Int64;
typedef unsigned long long Uint64;
typedef unsigned long long UInt64;
typedef signed long long   INT64;
typedef unsigned long long UINT64;

typedef volatile signed char   vInt8;
typedef volatile unsigned char vUint8;
typedef volatile unsigned char vUInt8;
typedef volatile signed char   VINT8;
typedef volatile unsigned char VUINT8;

typedef volatile signed short   vInt16;
typedef volatile unsigned short vUint16;
typedef volatile unsigned short vUInt16;
typedef volatile signed short   VINT16;
typedef volatile unsigned short VUINT16;

typedef volatile signed int   vInt32;
typedef volatile unsigned int vUint32;
typedef volatile unsigned int vUInt32;
typedef volatile signed int   VINT32;
typedef volatile unsigned int VUINT32;

typedef volatile signed long long   vInt64;
typedef volatile unsigned long long vUint64;
typedef volatile unsigned long long vUInt64;
typedef volatile signed long long   VINT64;
typedef volatile unsigned long long VUINT64;

#endif

typedef char Char;
typedef void Void;
typedef float Float;
typedef double Double;

typedef char CHAR;
typedef void VOID;
typedef float FLOAT;
typedef double DOUBLE;

#endif //TYPEDEFS_H


