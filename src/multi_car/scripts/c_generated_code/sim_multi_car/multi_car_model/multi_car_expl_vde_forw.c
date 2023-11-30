/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) multi_car_expl_vde_forw_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s1[213] = {14, 14, 0, 14, 28, 42, 56, 70, 84, 98, 112, 126, 140, 154, 168, 182, 196, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s2[93] = {14, 6, 0, 14, 28, 42, 56, 70, 84, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s3[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s4[302] = {298, 1, 0, 298, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 270, 271, 272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287, 288, 289, 290, 291, 292, 293, 294, 295, 296, 297};

/* multi_car_expl_vde_forw:(i0[14],i1[14x14],i2[14x6],i3[6],i4[298])->(o0[14],o1[14x14],o2[14x6]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][3] : 0;
  a1=arg[0]? arg[0][2] : 0;
  a2=cos(a1);
  a3=(a0*a2);
  if (res[0]!=0) res[0][0]=a3;
  a3=sin(a1);
  a4=(a0*a3);
  if (res[0]!=0) res[0][1]=a4;
  a4=arg[0]? arg[0][4] : 0;
  if (res[0]!=0) res[0][2]=a4;
  a4=arg[3]? arg[3][0] : 0;
  if (res[0]!=0) res[0][3]=a4;
  a4=arg[3]? arg[3][1] : 0;
  if (res[0]!=0) res[0][4]=a4;
  a4=arg[0]? arg[0][6] : 0;
  if (res[0]!=0) res[0][5]=a4;
  a4=arg[3]? arg[3][2] : 0;
  if (res[0]!=0) res[0][6]=a4;
  a4=arg[0]? arg[0][10] : 0;
  a5=arg[0]? arg[0][9] : 0;
  a6=cos(a5);
  a7=(a4*a6);
  if (res[0]!=0) res[0][7]=a7;
  a7=sin(a5);
  a8=(a4*a7);
  if (res[0]!=0) res[0][8]=a8;
  a8=arg[0]? arg[0][11] : 0;
  if (res[0]!=0) res[0][9]=a8;
  a8=arg[3]? arg[3][3] : 0;
  if (res[0]!=0) res[0][10]=a8;
  a8=arg[3]? arg[3][4] : 0;
  if (res[0]!=0) res[0][11]=a8;
  a8=arg[0]? arg[0][13] : 0;
  if (res[0]!=0) res[0][12]=a8;
  a8=arg[3]? arg[3][5] : 0;
  if (res[0]!=0) res[0][13]=a8;
  a8=arg[1]? arg[1][3] : 0;
  a9=(a2*a8);
  a10=sin(a1);
  a11=arg[1]? arg[1][2] : 0;
  a12=(a10*a11);
  a12=(a0*a12);
  a9=(a9-a12);
  if (res[1]!=0) res[1][0]=a9;
  a8=(a3*a8);
  a9=cos(a1);
  a11=(a9*a11);
  a11=(a0*a11);
  a8=(a8+a11);
  if (res[1]!=0) res[1][1]=a8;
  a8=arg[1]? arg[1][4] : 0;
  if (res[1]!=0) res[1][2]=a8;
  a8=0.;
  if (res[1]!=0) res[1][3]=a8;
  if (res[1]!=0) res[1][4]=a8;
  a11=arg[1]? arg[1][6] : 0;
  if (res[1]!=0) res[1][5]=a11;
  if (res[1]!=0) res[1][6]=a8;
  a11=arg[1]? arg[1][10] : 0;
  a12=(a6*a11);
  a13=sin(a5);
  a14=arg[1]? arg[1][9] : 0;
  a15=(a13*a14);
  a15=(a4*a15);
  a12=(a12-a15);
  if (res[1]!=0) res[1][7]=a12;
  a11=(a7*a11);
  a12=cos(a5);
  a14=(a12*a14);
  a14=(a4*a14);
  a11=(a11+a14);
  if (res[1]!=0) res[1][8]=a11;
  a11=arg[1]? arg[1][11] : 0;
  if (res[1]!=0) res[1][9]=a11;
  if (res[1]!=0) res[1][10]=a8;
  if (res[1]!=0) res[1][11]=a8;
  a11=arg[1]? arg[1][13] : 0;
  if (res[1]!=0) res[1][12]=a11;
  if (res[1]!=0) res[1][13]=a8;
  a11=arg[1]? arg[1][17] : 0;
  a14=(a2*a11);
  a15=arg[1]? arg[1][16] : 0;
  a16=(a10*a15);
  a16=(a0*a16);
  a14=(a14-a16);
  if (res[1]!=0) res[1][14]=a14;
  a11=(a3*a11);
  a15=(a9*a15);
  a15=(a0*a15);
  a11=(a11+a15);
  if (res[1]!=0) res[1][15]=a11;
  a11=arg[1]? arg[1][18] : 0;
  if (res[1]!=0) res[1][16]=a11;
  if (res[1]!=0) res[1][17]=a8;
  if (res[1]!=0) res[1][18]=a8;
  a11=arg[1]? arg[1][20] : 0;
  if (res[1]!=0) res[1][19]=a11;
  if (res[1]!=0) res[1][20]=a8;
  a11=arg[1]? arg[1][24] : 0;
  a15=(a6*a11);
  a14=arg[1]? arg[1][23] : 0;
  a16=(a13*a14);
  a16=(a4*a16);
  a15=(a15-a16);
  if (res[1]!=0) res[1][21]=a15;
  a11=(a7*a11);
  a14=(a12*a14);
  a14=(a4*a14);
  a11=(a11+a14);
  if (res[1]!=0) res[1][22]=a11;
  a11=arg[1]? arg[1][25] : 0;
  if (res[1]!=0) res[1][23]=a11;
  if (res[1]!=0) res[1][24]=a8;
  if (res[1]!=0) res[1][25]=a8;
  a11=arg[1]? arg[1][27] : 0;
  if (res[1]!=0) res[1][26]=a11;
  if (res[1]!=0) res[1][27]=a8;
  a11=arg[1]? arg[1][31] : 0;
  a14=(a2*a11);
  a15=arg[1]? arg[1][30] : 0;
  a16=(a10*a15);
  a16=(a0*a16);
  a14=(a14-a16);
  if (res[1]!=0) res[1][28]=a14;
  a11=(a3*a11);
  a15=(a9*a15);
  a15=(a0*a15);
  a11=(a11+a15);
  if (res[1]!=0) res[1][29]=a11;
  a11=arg[1]? arg[1][32] : 0;
  if (res[1]!=0) res[1][30]=a11;
  if (res[1]!=0) res[1][31]=a8;
  if (res[1]!=0) res[1][32]=a8;
  a11=arg[1]? arg[1][34] : 0;
  if (res[1]!=0) res[1][33]=a11;
  if (res[1]!=0) res[1][34]=a8;
  a11=arg[1]? arg[1][38] : 0;
  a15=(a6*a11);
  a14=arg[1]? arg[1][37] : 0;
  a16=(a13*a14);
  a16=(a4*a16);
  a15=(a15-a16);
  if (res[1]!=0) res[1][35]=a15;
  a11=(a7*a11);
  a14=(a12*a14);
  a14=(a4*a14);
  a11=(a11+a14);
  if (res[1]!=0) res[1][36]=a11;
  a11=arg[1]? arg[1][39] : 0;
  if (res[1]!=0) res[1][37]=a11;
  if (res[1]!=0) res[1][38]=a8;
  if (res[1]!=0) res[1][39]=a8;
  a11=arg[1]? arg[1][41] : 0;
  if (res[1]!=0) res[1][40]=a11;
  if (res[1]!=0) res[1][41]=a8;
  a11=arg[1]? arg[1][45] : 0;
  a14=(a2*a11);
  a15=arg[1]? arg[1][44] : 0;
  a16=(a10*a15);
  a16=(a0*a16);
  a14=(a14-a16);
  if (res[1]!=0) res[1][42]=a14;
  a11=(a3*a11);
  a15=(a9*a15);
  a15=(a0*a15);
  a11=(a11+a15);
  if (res[1]!=0) res[1][43]=a11;
  a11=arg[1]? arg[1][46] : 0;
  if (res[1]!=0) res[1][44]=a11;
  if (res[1]!=0) res[1][45]=a8;
  if (res[1]!=0) res[1][46]=a8;
  a11=arg[1]? arg[1][48] : 0;
  if (res[1]!=0) res[1][47]=a11;
  if (res[1]!=0) res[1][48]=a8;
  a11=arg[1]? arg[1][52] : 0;
  a15=(a6*a11);
  a14=arg[1]? arg[1][51] : 0;
  a16=(a13*a14);
  a16=(a4*a16);
  a15=(a15-a16);
  if (res[1]!=0) res[1][49]=a15;
  a11=(a7*a11);
  a14=(a12*a14);
  a14=(a4*a14);
  a11=(a11+a14);
  if (res[1]!=0) res[1][50]=a11;
  a11=arg[1]? arg[1][53] : 0;
  if (res[1]!=0) res[1][51]=a11;
  if (res[1]!=0) res[1][52]=a8;
  if (res[1]!=0) res[1][53]=a8;
  a11=arg[1]? arg[1][55] : 0;
  if (res[1]!=0) res[1][54]=a11;
  if (res[1]!=0) res[1][55]=a8;
  a11=arg[1]? arg[1][59] : 0;
  a14=(a2*a11);
  a15=arg[1]? arg[1][58] : 0;
  a16=(a10*a15);
  a16=(a0*a16);
  a14=(a14-a16);
  if (res[1]!=0) res[1][56]=a14;
  a11=(a3*a11);
  a15=(a9*a15);
  a15=(a0*a15);
  a11=(a11+a15);
  if (res[1]!=0) res[1][57]=a11;
  a11=arg[1]? arg[1][60] : 0;
  if (res[1]!=0) res[1][58]=a11;
  if (res[1]!=0) res[1][59]=a8;
  if (res[1]!=0) res[1][60]=a8;
  a11=arg[1]? arg[1][62] : 0;
  if (res[1]!=0) res[1][61]=a11;
  if (res[1]!=0) res[1][62]=a8;
  a11=arg[1]? arg[1][66] : 0;
  a15=(a6*a11);
  a14=arg[1]? arg[1][65] : 0;
  a16=(a13*a14);
  a16=(a4*a16);
  a15=(a15-a16);
  if (res[1]!=0) res[1][63]=a15;
  a11=(a7*a11);
  a14=(a12*a14);
  a14=(a4*a14);
  a11=(a11+a14);
  if (res[1]!=0) res[1][64]=a11;
  a11=arg[1]? arg[1][67] : 0;
  if (res[1]!=0) res[1][65]=a11;
  if (res[1]!=0) res[1][66]=a8;
  if (res[1]!=0) res[1][67]=a8;
  a11=arg[1]? arg[1][69] : 0;
  if (res[1]!=0) res[1][68]=a11;
  if (res[1]!=0) res[1][69]=a8;
  a11=arg[1]? arg[1][73] : 0;
  a14=(a2*a11);
  a15=arg[1]? arg[1][72] : 0;
  a16=(a10*a15);
  a16=(a0*a16);
  a14=(a14-a16);
  if (res[1]!=0) res[1][70]=a14;
  a11=(a3*a11);
  a15=(a9*a15);
  a15=(a0*a15);
  a11=(a11+a15);
  if (res[1]!=0) res[1][71]=a11;
  a11=arg[1]? arg[1][74] : 0;
  if (res[1]!=0) res[1][72]=a11;
  if (res[1]!=0) res[1][73]=a8;
  if (res[1]!=0) res[1][74]=a8;
  a11=arg[1]? arg[1][76] : 0;
  if (res[1]!=0) res[1][75]=a11;
  if (res[1]!=0) res[1][76]=a8;
  a11=arg[1]? arg[1][80] : 0;
  a15=(a6*a11);
  a14=arg[1]? arg[1][79] : 0;
  a16=(a13*a14);
  a16=(a4*a16);
  a15=(a15-a16);
  if (res[1]!=0) res[1][77]=a15;
  a11=(a7*a11);
  a14=(a12*a14);
  a14=(a4*a14);
  a11=(a11+a14);
  if (res[1]!=0) res[1][78]=a11;
  a11=arg[1]? arg[1][81] : 0;
  if (res[1]!=0) res[1][79]=a11;
  if (res[1]!=0) res[1][80]=a8;
  if (res[1]!=0) res[1][81]=a8;
  a11=arg[1]? arg[1][83] : 0;
  if (res[1]!=0) res[1][82]=a11;
  if (res[1]!=0) res[1][83]=a8;
  a11=arg[1]? arg[1][87] : 0;
  a14=(a2*a11);
  a15=arg[1]? arg[1][86] : 0;
  a16=(a10*a15);
  a16=(a0*a16);
  a14=(a14-a16);
  if (res[1]!=0) res[1][84]=a14;
  a11=(a3*a11);
  a15=(a9*a15);
  a15=(a0*a15);
  a11=(a11+a15);
  if (res[1]!=0) res[1][85]=a11;
  a11=arg[1]? arg[1][88] : 0;
  if (res[1]!=0) res[1][86]=a11;
  if (res[1]!=0) res[1][87]=a8;
  if (res[1]!=0) res[1][88]=a8;
  a11=arg[1]? arg[1][90] : 0;
  if (res[1]!=0) res[1][89]=a11;
  if (res[1]!=0) res[1][90]=a8;
  a11=arg[1]? arg[1][94] : 0;
  a15=(a6*a11);
  a14=arg[1]? arg[1][93] : 0;
  a16=(a13*a14);
  a16=(a4*a16);
  a15=(a15-a16);
  if (res[1]!=0) res[1][91]=a15;
  a11=(a7*a11);
  a14=(a12*a14);
  a14=(a4*a14);
  a11=(a11+a14);
  if (res[1]!=0) res[1][92]=a11;
  a11=arg[1]? arg[1][95] : 0;
  if (res[1]!=0) res[1][93]=a11;
  if (res[1]!=0) res[1][94]=a8;
  if (res[1]!=0) res[1][95]=a8;
  a11=arg[1]? arg[1][97] : 0;
  if (res[1]!=0) res[1][96]=a11;
  if (res[1]!=0) res[1][97]=a8;
  a11=arg[1]? arg[1][101] : 0;
  a14=(a2*a11);
  a15=arg[1]? arg[1][100] : 0;
  a16=(a10*a15);
  a16=(a0*a16);
  a14=(a14-a16);
  if (res[1]!=0) res[1][98]=a14;
  a11=(a3*a11);
  a15=(a9*a15);
  a15=(a0*a15);
  a11=(a11+a15);
  if (res[1]!=0) res[1][99]=a11;
  a11=arg[1]? arg[1][102] : 0;
  if (res[1]!=0) res[1][100]=a11;
  if (res[1]!=0) res[1][101]=a8;
  if (res[1]!=0) res[1][102]=a8;
  a11=arg[1]? arg[1][104] : 0;
  if (res[1]!=0) res[1][103]=a11;
  if (res[1]!=0) res[1][104]=a8;
  a11=arg[1]? arg[1][108] : 0;
  a15=(a6*a11);
  a14=arg[1]? arg[1][107] : 0;
  a16=(a13*a14);
  a16=(a4*a16);
  a15=(a15-a16);
  if (res[1]!=0) res[1][105]=a15;
  a11=(a7*a11);
  a14=(a12*a14);
  a14=(a4*a14);
  a11=(a11+a14);
  if (res[1]!=0) res[1][106]=a11;
  a11=arg[1]? arg[1][109] : 0;
  if (res[1]!=0) res[1][107]=a11;
  if (res[1]!=0) res[1][108]=a8;
  if (res[1]!=0) res[1][109]=a8;
  a11=arg[1]? arg[1][111] : 0;
  if (res[1]!=0) res[1][110]=a11;
  if (res[1]!=0) res[1][111]=a8;
  a11=arg[1]? arg[1][115] : 0;
  a14=(a2*a11);
  a15=arg[1]? arg[1][114] : 0;
  a16=(a10*a15);
  a16=(a0*a16);
  a14=(a14-a16);
  if (res[1]!=0) res[1][112]=a14;
  a11=(a3*a11);
  a15=(a9*a15);
  a15=(a0*a15);
  a11=(a11+a15);
  if (res[1]!=0) res[1][113]=a11;
  a11=arg[1]? arg[1][116] : 0;
  if (res[1]!=0) res[1][114]=a11;
  if (res[1]!=0) res[1][115]=a8;
  if (res[1]!=0) res[1][116]=a8;
  a11=arg[1]? arg[1][118] : 0;
  if (res[1]!=0) res[1][117]=a11;
  if (res[1]!=0) res[1][118]=a8;
  a11=arg[1]? arg[1][122] : 0;
  a15=(a6*a11);
  a14=arg[1]? arg[1][121] : 0;
  a16=(a13*a14);
  a16=(a4*a16);
  a15=(a15-a16);
  if (res[1]!=0) res[1][119]=a15;
  a11=(a7*a11);
  a14=(a12*a14);
  a14=(a4*a14);
  a11=(a11+a14);
  if (res[1]!=0) res[1][120]=a11;
  a11=arg[1]? arg[1][123] : 0;
  if (res[1]!=0) res[1][121]=a11;
  if (res[1]!=0) res[1][122]=a8;
  if (res[1]!=0) res[1][123]=a8;
  a11=arg[1]? arg[1][125] : 0;
  if (res[1]!=0) res[1][124]=a11;
  if (res[1]!=0) res[1][125]=a8;
  a11=arg[1]? arg[1][129] : 0;
  a14=(a2*a11);
  a15=arg[1]? arg[1][128] : 0;
  a16=(a10*a15);
  a16=(a0*a16);
  a14=(a14-a16);
  if (res[1]!=0) res[1][126]=a14;
  a11=(a3*a11);
  a15=(a9*a15);
  a15=(a0*a15);
  a11=(a11+a15);
  if (res[1]!=0) res[1][127]=a11;
  a11=arg[1]? arg[1][130] : 0;
  if (res[1]!=0) res[1][128]=a11;
  if (res[1]!=0) res[1][129]=a8;
  if (res[1]!=0) res[1][130]=a8;
  a11=arg[1]? arg[1][132] : 0;
  if (res[1]!=0) res[1][131]=a11;
  if (res[1]!=0) res[1][132]=a8;
  a11=arg[1]? arg[1][136] : 0;
  a15=(a6*a11);
  a14=arg[1]? arg[1][135] : 0;
  a16=(a13*a14);
  a16=(a4*a16);
  a15=(a15-a16);
  if (res[1]!=0) res[1][133]=a15;
  a11=(a7*a11);
  a14=(a12*a14);
  a14=(a4*a14);
  a11=(a11+a14);
  if (res[1]!=0) res[1][134]=a11;
  a11=arg[1]? arg[1][137] : 0;
  if (res[1]!=0) res[1][135]=a11;
  if (res[1]!=0) res[1][136]=a8;
  if (res[1]!=0) res[1][137]=a8;
  a11=arg[1]? arg[1][139] : 0;
  if (res[1]!=0) res[1][138]=a11;
  if (res[1]!=0) res[1][139]=a8;
  a11=arg[1]? arg[1][143] : 0;
  a14=(a2*a11);
  a15=arg[1]? arg[1][142] : 0;
  a16=(a10*a15);
  a16=(a0*a16);
  a14=(a14-a16);
  if (res[1]!=0) res[1][140]=a14;
  a11=(a3*a11);
  a15=(a9*a15);
  a15=(a0*a15);
  a11=(a11+a15);
  if (res[1]!=0) res[1][141]=a11;
  a11=arg[1]? arg[1][144] : 0;
  if (res[1]!=0) res[1][142]=a11;
  if (res[1]!=0) res[1][143]=a8;
  if (res[1]!=0) res[1][144]=a8;
  a11=arg[1]? arg[1][146] : 0;
  if (res[1]!=0) res[1][145]=a11;
  if (res[1]!=0) res[1][146]=a8;
  a11=arg[1]? arg[1][150] : 0;
  a15=(a6*a11);
  a14=arg[1]? arg[1][149] : 0;
  a16=(a13*a14);
  a16=(a4*a16);
  a15=(a15-a16);
  if (res[1]!=0) res[1][147]=a15;
  a11=(a7*a11);
  a14=(a12*a14);
  a14=(a4*a14);
  a11=(a11+a14);
  if (res[1]!=0) res[1][148]=a11;
  a11=arg[1]? arg[1][151] : 0;
  if (res[1]!=0) res[1][149]=a11;
  if (res[1]!=0) res[1][150]=a8;
  if (res[1]!=0) res[1][151]=a8;
  a11=arg[1]? arg[1][153] : 0;
  if (res[1]!=0) res[1][152]=a11;
  if (res[1]!=0) res[1][153]=a8;
  a11=arg[1]? arg[1][157] : 0;
  a14=(a2*a11);
  a15=arg[1]? arg[1][156] : 0;
  a16=(a10*a15);
  a16=(a0*a16);
  a14=(a14-a16);
  if (res[1]!=0) res[1][154]=a14;
  a11=(a3*a11);
  a15=(a9*a15);
  a15=(a0*a15);
  a11=(a11+a15);
  if (res[1]!=0) res[1][155]=a11;
  a11=arg[1]? arg[1][158] : 0;
  if (res[1]!=0) res[1][156]=a11;
  if (res[1]!=0) res[1][157]=a8;
  if (res[1]!=0) res[1][158]=a8;
  a11=arg[1]? arg[1][160] : 0;
  if (res[1]!=0) res[1][159]=a11;
  if (res[1]!=0) res[1][160]=a8;
  a11=arg[1]? arg[1][164] : 0;
  a15=(a6*a11);
  a14=arg[1]? arg[1][163] : 0;
  a16=(a13*a14);
  a16=(a4*a16);
  a15=(a15-a16);
  if (res[1]!=0) res[1][161]=a15;
  a11=(a7*a11);
  a14=(a12*a14);
  a14=(a4*a14);
  a11=(a11+a14);
  if (res[1]!=0) res[1][162]=a11;
  a11=arg[1]? arg[1][165] : 0;
  if (res[1]!=0) res[1][163]=a11;
  if (res[1]!=0) res[1][164]=a8;
  if (res[1]!=0) res[1][165]=a8;
  a11=arg[1]? arg[1][167] : 0;
  if (res[1]!=0) res[1][166]=a11;
  if (res[1]!=0) res[1][167]=a8;
  a11=arg[1]? arg[1][171] : 0;
  a14=(a2*a11);
  a15=arg[1]? arg[1][170] : 0;
  a16=(a10*a15);
  a16=(a0*a16);
  a14=(a14-a16);
  if (res[1]!=0) res[1][168]=a14;
  a11=(a3*a11);
  a15=(a9*a15);
  a15=(a0*a15);
  a11=(a11+a15);
  if (res[1]!=0) res[1][169]=a11;
  a11=arg[1]? arg[1][172] : 0;
  if (res[1]!=0) res[1][170]=a11;
  if (res[1]!=0) res[1][171]=a8;
  if (res[1]!=0) res[1][172]=a8;
  a11=arg[1]? arg[1][174] : 0;
  if (res[1]!=0) res[1][173]=a11;
  if (res[1]!=0) res[1][174]=a8;
  a11=arg[1]? arg[1][178] : 0;
  a15=(a6*a11);
  a14=arg[1]? arg[1][177] : 0;
  a16=(a13*a14);
  a16=(a4*a16);
  a15=(a15-a16);
  if (res[1]!=0) res[1][175]=a15;
  a11=(a7*a11);
  a14=(a12*a14);
  a14=(a4*a14);
  a11=(a11+a14);
  if (res[1]!=0) res[1][176]=a11;
  a11=arg[1]? arg[1][179] : 0;
  if (res[1]!=0) res[1][177]=a11;
  if (res[1]!=0) res[1][178]=a8;
  if (res[1]!=0) res[1][179]=a8;
  a11=arg[1]? arg[1][181] : 0;
  if (res[1]!=0) res[1][180]=a11;
  if (res[1]!=0) res[1][181]=a8;
  a11=arg[1]? arg[1][185] : 0;
  a14=(a2*a11);
  a15=arg[1]? arg[1][184] : 0;
  a10=(a10*a15);
  a10=(a0*a10);
  a14=(a14-a10);
  if (res[1]!=0) res[1][182]=a14;
  a11=(a3*a11);
  a9=(a9*a15);
  a9=(a0*a9);
  a11=(a11+a9);
  if (res[1]!=0) res[1][183]=a11;
  a11=arg[1]? arg[1][186] : 0;
  if (res[1]!=0) res[1][184]=a11;
  if (res[1]!=0) res[1][185]=a8;
  if (res[1]!=0) res[1][186]=a8;
  a11=arg[1]? arg[1][188] : 0;
  if (res[1]!=0) res[1][187]=a11;
  if (res[1]!=0) res[1][188]=a8;
  a11=arg[1]? arg[1][192] : 0;
  a9=(a6*a11);
  a15=arg[1]? arg[1][191] : 0;
  a13=(a13*a15);
  a13=(a4*a13);
  a9=(a9-a13);
  if (res[1]!=0) res[1][189]=a9;
  a11=(a7*a11);
  a12=(a12*a15);
  a12=(a4*a12);
  a11=(a11+a12);
  if (res[1]!=0) res[1][190]=a11;
  a11=arg[1]? arg[1][193] : 0;
  if (res[1]!=0) res[1][191]=a11;
  if (res[1]!=0) res[1][192]=a8;
  if (res[1]!=0) res[1][193]=a8;
  a11=arg[1]? arg[1][195] : 0;
  if (res[1]!=0) res[1][194]=a11;
  if (res[1]!=0) res[1][195]=a8;
  a11=arg[2]? arg[2][3] : 0;
  a12=(a2*a11);
  a15=sin(a1);
  a9=arg[2]? arg[2][2] : 0;
  a13=(a15*a9);
  a13=(a0*a13);
  a12=(a12-a13);
  if (res[2]!=0) res[2][0]=a12;
  a11=(a3*a11);
  a1=cos(a1);
  a9=(a1*a9);
  a9=(a0*a9);
  a11=(a11+a9);
  if (res[2]!=0) res[2][1]=a11;
  a11=arg[2]? arg[2][4] : 0;
  if (res[2]!=0) res[2][2]=a11;
  a11=1.;
  if (res[2]!=0) res[2][3]=a11;
  if (res[2]!=0) res[2][4]=a8;
  a9=arg[2]? arg[2][6] : 0;
  if (res[2]!=0) res[2][5]=a9;
  if (res[2]!=0) res[2][6]=a8;
  a9=arg[2]? arg[2][10] : 0;
  a12=(a6*a9);
  a13=sin(a5);
  a14=arg[2]? arg[2][9] : 0;
  a10=(a13*a14);
  a10=(a4*a10);
  a12=(a12-a10);
  if (res[2]!=0) res[2][7]=a12;
  a9=(a7*a9);
  a5=cos(a5);
  a14=(a5*a14);
  a14=(a4*a14);
  a9=(a9+a14);
  if (res[2]!=0) res[2][8]=a9;
  a9=arg[2]? arg[2][11] : 0;
  if (res[2]!=0) res[2][9]=a9;
  if (res[2]!=0) res[2][10]=a8;
  if (res[2]!=0) res[2][11]=a8;
  a9=arg[2]? arg[2][13] : 0;
  if (res[2]!=0) res[2][12]=a9;
  if (res[2]!=0) res[2][13]=a8;
  a9=arg[2]? arg[2][17] : 0;
  a14=(a2*a9);
  a12=arg[2]? arg[2][16] : 0;
  a10=(a15*a12);
  a10=(a0*a10);
  a14=(a14-a10);
  if (res[2]!=0) res[2][14]=a14;
  a9=(a3*a9);
  a12=(a1*a12);
  a12=(a0*a12);
  a9=(a9+a12);
  if (res[2]!=0) res[2][15]=a9;
  a9=arg[2]? arg[2][18] : 0;
  if (res[2]!=0) res[2][16]=a9;
  if (res[2]!=0) res[2][17]=a8;
  if (res[2]!=0) res[2][18]=a11;
  a9=arg[2]? arg[2][20] : 0;
  if (res[2]!=0) res[2][19]=a9;
  if (res[2]!=0) res[2][20]=a8;
  a9=arg[2]? arg[2][24] : 0;
  a12=(a6*a9);
  a14=arg[2]? arg[2][23] : 0;
  a10=(a13*a14);
  a10=(a4*a10);
  a12=(a12-a10);
  if (res[2]!=0) res[2][21]=a12;
  a9=(a7*a9);
  a14=(a5*a14);
  a14=(a4*a14);
  a9=(a9+a14);
  if (res[2]!=0) res[2][22]=a9;
  a9=arg[2]? arg[2][25] : 0;
  if (res[2]!=0) res[2][23]=a9;
  if (res[2]!=0) res[2][24]=a8;
  if (res[2]!=0) res[2][25]=a8;
  a9=arg[2]? arg[2][27] : 0;
  if (res[2]!=0) res[2][26]=a9;
  if (res[2]!=0) res[2][27]=a8;
  a9=arg[2]? arg[2][31] : 0;
  a14=(a2*a9);
  a12=arg[2]? arg[2][30] : 0;
  a10=(a15*a12);
  a10=(a0*a10);
  a14=(a14-a10);
  if (res[2]!=0) res[2][28]=a14;
  a9=(a3*a9);
  a12=(a1*a12);
  a12=(a0*a12);
  a9=(a9+a12);
  if (res[2]!=0) res[2][29]=a9;
  a9=arg[2]? arg[2][32] : 0;
  if (res[2]!=0) res[2][30]=a9;
  if (res[2]!=0) res[2][31]=a8;
  if (res[2]!=0) res[2][32]=a8;
  a9=arg[2]? arg[2][34] : 0;
  if (res[2]!=0) res[2][33]=a9;
  if (res[2]!=0) res[2][34]=a11;
  a9=arg[2]? arg[2][38] : 0;
  a12=(a6*a9);
  a14=arg[2]? arg[2][37] : 0;
  a10=(a13*a14);
  a10=(a4*a10);
  a12=(a12-a10);
  if (res[2]!=0) res[2][35]=a12;
  a9=(a7*a9);
  a14=(a5*a14);
  a14=(a4*a14);
  a9=(a9+a14);
  if (res[2]!=0) res[2][36]=a9;
  a9=arg[2]? arg[2][39] : 0;
  if (res[2]!=0) res[2][37]=a9;
  if (res[2]!=0) res[2][38]=a8;
  if (res[2]!=0) res[2][39]=a8;
  a9=arg[2]? arg[2][41] : 0;
  if (res[2]!=0) res[2][40]=a9;
  if (res[2]!=0) res[2][41]=a8;
  a9=arg[2]? arg[2][45] : 0;
  a14=(a2*a9);
  a12=arg[2]? arg[2][44] : 0;
  a10=(a15*a12);
  a10=(a0*a10);
  a14=(a14-a10);
  if (res[2]!=0) res[2][42]=a14;
  a9=(a3*a9);
  a12=(a1*a12);
  a12=(a0*a12);
  a9=(a9+a12);
  if (res[2]!=0) res[2][43]=a9;
  a9=arg[2]? arg[2][46] : 0;
  if (res[2]!=0) res[2][44]=a9;
  if (res[2]!=0) res[2][45]=a8;
  if (res[2]!=0) res[2][46]=a8;
  a9=arg[2]? arg[2][48] : 0;
  if (res[2]!=0) res[2][47]=a9;
  if (res[2]!=0) res[2][48]=a8;
  a9=arg[2]? arg[2][52] : 0;
  a12=(a6*a9);
  a14=arg[2]? arg[2][51] : 0;
  a10=(a13*a14);
  a10=(a4*a10);
  a12=(a12-a10);
  if (res[2]!=0) res[2][49]=a12;
  a9=(a7*a9);
  a14=(a5*a14);
  a14=(a4*a14);
  a9=(a9+a14);
  if (res[2]!=0) res[2][50]=a9;
  a9=arg[2]? arg[2][53] : 0;
  if (res[2]!=0) res[2][51]=a9;
  if (res[2]!=0) res[2][52]=a11;
  if (res[2]!=0) res[2][53]=a8;
  a9=arg[2]? arg[2][55] : 0;
  if (res[2]!=0) res[2][54]=a9;
  if (res[2]!=0) res[2][55]=a8;
  a9=arg[2]? arg[2][59] : 0;
  a14=(a2*a9);
  a12=arg[2]? arg[2][58] : 0;
  a10=(a15*a12);
  a10=(a0*a10);
  a14=(a14-a10);
  if (res[2]!=0) res[2][56]=a14;
  a9=(a3*a9);
  a12=(a1*a12);
  a12=(a0*a12);
  a9=(a9+a12);
  if (res[2]!=0) res[2][57]=a9;
  a9=arg[2]? arg[2][60] : 0;
  if (res[2]!=0) res[2][58]=a9;
  if (res[2]!=0) res[2][59]=a8;
  if (res[2]!=0) res[2][60]=a8;
  a9=arg[2]? arg[2][62] : 0;
  if (res[2]!=0) res[2][61]=a9;
  if (res[2]!=0) res[2][62]=a8;
  a9=arg[2]? arg[2][66] : 0;
  a12=(a6*a9);
  a14=arg[2]? arg[2][65] : 0;
  a10=(a13*a14);
  a10=(a4*a10);
  a12=(a12-a10);
  if (res[2]!=0) res[2][63]=a12;
  a9=(a7*a9);
  a14=(a5*a14);
  a14=(a4*a14);
  a9=(a9+a14);
  if (res[2]!=0) res[2][64]=a9;
  a9=arg[2]? arg[2][67] : 0;
  if (res[2]!=0) res[2][65]=a9;
  if (res[2]!=0) res[2][66]=a8;
  if (res[2]!=0) res[2][67]=a11;
  a9=arg[2]? arg[2][69] : 0;
  if (res[2]!=0) res[2][68]=a9;
  if (res[2]!=0) res[2][69]=a8;
  a9=arg[2]? arg[2][73] : 0;
  a2=(a2*a9);
  a14=arg[2]? arg[2][72] : 0;
  a15=(a15*a14);
  a15=(a0*a15);
  a2=(a2-a15);
  if (res[2]!=0) res[2][70]=a2;
  a3=(a3*a9);
  a1=(a1*a14);
  a0=(a0*a1);
  a3=(a3+a0);
  if (res[2]!=0) res[2][71]=a3;
  a3=arg[2]? arg[2][74] : 0;
  if (res[2]!=0) res[2][72]=a3;
  if (res[2]!=0) res[2][73]=a8;
  if (res[2]!=0) res[2][74]=a8;
  a3=arg[2]? arg[2][76] : 0;
  if (res[2]!=0) res[2][75]=a3;
  if (res[2]!=0) res[2][76]=a8;
  a3=arg[2]? arg[2][80] : 0;
  a6=(a6*a3);
  a0=arg[2]? arg[2][79] : 0;
  a13=(a13*a0);
  a13=(a4*a13);
  a6=(a6-a13);
  if (res[2]!=0) res[2][77]=a6;
  a7=(a7*a3);
  a5=(a5*a0);
  a4=(a4*a5);
  a7=(a7+a4);
  if (res[2]!=0) res[2][78]=a7;
  a7=arg[2]? arg[2][81] : 0;
  if (res[2]!=0) res[2][79]=a7;
  if (res[2]!=0) res[2][80]=a8;
  if (res[2]!=0) res[2][81]=a8;
  a8=arg[2]? arg[2][83] : 0;
  if (res[2]!=0) res[2][82]=a8;
  if (res[2]!=0) res[2][83]=a11;
  return 0;
}

CASADI_SYMBOL_EXPORT int multi_car_expl_vde_forw(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int multi_car_expl_vde_forw_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int multi_car_expl_vde_forw_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void multi_car_expl_vde_forw_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int multi_car_expl_vde_forw_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void multi_car_expl_vde_forw_release(int mem) {
}

CASADI_SYMBOL_EXPORT void multi_car_expl_vde_forw_incref(void) {
}

CASADI_SYMBOL_EXPORT void multi_car_expl_vde_forw_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int multi_car_expl_vde_forw_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int multi_car_expl_vde_forw_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real multi_car_expl_vde_forw_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* multi_car_expl_vde_forw_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* multi_car_expl_vde_forw_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* multi_car_expl_vde_forw_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    case 4: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* multi_car_expl_vde_forw_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int multi_car_expl_vde_forw_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
