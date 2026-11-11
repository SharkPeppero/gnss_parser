#ifndef GNSS_COMM_CONSTANTS_OBSERVATION_HPP_
#define GNSS_COMM_CONSTANTS_OBSERVATION_HPP_

// 观测容量与信号编码；CODE_* 标识频段和调制方式，不是频率值。
// MAXOBS 为单历元观测数上限，可由外部预定义覆盖。
// 历元：一个采样时刻
// 一个历元可以包含多颗卫星、每颗卫星可能包含多个信号

namespace gnss_comm {

#ifndef MAXOBS
#define MAXOBS 64 /* max number of obs in an epoch */
#endif

// 观测信号的类型 星座的各种频段和调制方式
#define CODE_NONE 0 /* obs code: none or unknown */
#define CODE_L1C 1  /* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
#define CODE_L1P 2  /* obs code: L1P,G1P    (GPS,GLO) */
#define CODE_L1W 3  /* obs code: L1 Z-track (GPS) */
#define CODE_L1Y 4  /* obs code: L1Y        (GPS) */
#define CODE_L1M 5  /* obs code: L1M        (GPS) */
#define CODE_L1N 6  /* obs code: L1codeless (GPS) */
#define CODE_L1S 7  /* obs code: L1C(D)     (GPS,QZS) */
#define CODE_L1L 8  /* obs code: L1C(P)     (GPS,QZS) */
#define CODE_L1E 9  /* obs code: L1-SAIF    (QZS) */
#define CODE_L1A 10 /* obs code: E1A        (GAL) */
#define CODE_L1B 11 /* obs code: E1B        (GAL) */
#define CODE_L1X 12 /* obs code: E1B+C,L1C(D+P) (GAL,QZS) */
#define CODE_L1Z 13 /* obs code: E1A+B+C,L1SAIF (GAL,QZS) */
#define CODE_L2C 14 /* obs code: L2C/A,G2C/A (GPS,GLO) */
#define CODE_L2D 15 /* obs code: L2 L1C/A-(P2-P1) (GPS) */
#define CODE_L2S 16 /* obs code: L2C(M)     (GPS,QZS) */
#define CODE_L2L 17 /* obs code: L2C(L)     (GPS,QZS) */
#define CODE_L2X 18 /* obs code: L2C(M+L),B1I+Q (GPS,QZS,CMP) */
#define CODE_L2P 19 /* obs code: L2P,G2P    (GPS,GLO) */
#define CODE_L2W 20 /* obs code: L2 Z-track (GPS) */
#define CODE_L2Y 21 /* obs code: L2Y        (GPS) */
#define CODE_L2M 22 /* obs code: L2M        (GPS) */
#define CODE_L2N 23 /* obs code: L2codeless (GPS) */
#define CODE_L5I 24 /* obs code: L5/E5aI    (GPS,GAL,QZS,SBS) */
#define CODE_L5Q 25 /* obs code: L5/E5aQ    (GPS,GAL,QZS,SBS) */
#define CODE_L5X 26 /* obs code: L5/E5aI+Q  (GPS,GAL,QZS,SBS) */
#define CODE_L7I 27 /* obs code: E5bI,B2I   (GAL,CMP) */
#define CODE_L7Q 28 /* obs code: E5bQ,B2Q   (GAL,CMP) */
#define CODE_L7X 29 /* obs code: E5bI+Q,B2I+Q (GAL,CMP) */
#define CODE_L6A 30 /* obs code: E6A        (GAL) */
#define CODE_L6B 31 /* obs code: E6B        (GAL) */
#define CODE_L6C 32 /* obs code: E6C        (GAL) */
#define CODE_L6X 33 /* obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,CMP) */
#define CODE_L6Z 34 /* obs code: E6A+B+C    (GAL) */
#define CODE_L6S 35 /* obs code: LEXS       (QZS) */
#define CODE_L6L 36 /* obs code: LEXL       (QZS) */
#define CODE_L8I 37 /* obs code: E5(a+b)I   (GAL) */
#define CODE_L8Q 38 /* obs code: E5(a+b)Q   (GAL) */
#define CODE_L8X 39 /* obs code: E5(a+b)I+Q (GAL) */
#define CODE_L2I 40 /* obs code: B1I        (CMP, RINEX 3.02 "2I"; ublox uses CODE_L1I) */
#define CODE_L2Q 41 /* obs code: B1Q        (CMP) */
#define CODE_L6I 42 /* obs code: B3I        (CMP) */
#define CODE_L6Q 43 /* obs code: B3Q        (CMP) */
#define CODE_L3I 44 /* obs code: G3I        (GLO) */
#define CODE_L3Q 45 /* obs code: G3Q        (GLO) */
#define CODE_L3X 46 /* obs code: G3I+Q      (GLO) */
#define CODE_L1I 47 /* obs code: B1I        (BDS, used by ublox driver) */
#define CODE_L1Q 48 /* obs code: B1Q        (BDS) */
#define MAXCODE 48  /* max number of obs code */

} // namespace gnss_comm

#endif // GNSS_COMM_CONSTANTS_OBSERVATION_HPP_
