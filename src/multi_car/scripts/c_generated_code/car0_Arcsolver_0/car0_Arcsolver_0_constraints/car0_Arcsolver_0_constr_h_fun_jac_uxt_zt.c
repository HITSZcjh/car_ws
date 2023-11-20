/* This file was automatically generated by CasADi 3.6.3.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) car0_Arcsolver_0_constr_h_fun_jac_uxt_zt_ ## ID
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
#define casadi_sq CASADI_PREFIX(sq)

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

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[64] = {60, 1, 0, 60, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59};
static const casadi_int casadi_s1[3] = {0, 0, 0};
static const casadi_int casadi_s2[52] = {48, 1, 0, 48, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47};
static const casadi_int casadi_s3[222] = {60, 48, 0, 1, 2, 3, 7, 11, 15, 16, 17, 18, 22, 26, 30, 31, 32, 33, 37, 41, 45, 46, 47, 48, 52, 56, 60, 61, 62, 63, 67, 71, 75, 79, 83, 87, 91, 95, 99, 103, 107, 111, 115, 119, 123, 132, 141, 150, 159, 168, 171, 0, 4, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 16, 20, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 28, 32, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 40, 44, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 52, 56, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 1, 2, 3, 13, 5, 6, 7, 17, 9, 10, 11, 21, 13, 14, 15, 25, 17, 18, 19, 29, 21, 22, 23, 33, 25, 26, 27, 37, 29, 30, 31, 41, 33, 34, 35, 45, 37, 38, 39, 49, 41, 42, 43, 53, 45, 46, 47, 57, 1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15, 17, 18, 19, 21, 22, 23, 25, 26, 27, 29, 30, 31, 33, 34, 35, 37, 38, 39, 41, 42, 43, 45, 46, 47, 49, 50, 51, 53, 54, 55, 57, 58, 59, 1, 5, 9};
static const casadi_int casadi_s4[3] = {48, 0, 0};

/* car0_Arcsolver_0_constr_h_fun_jac_uxt_zt:(i0[60],i1[],i2[],i3[])->(o0[48],o1[60x48,171nz],o2[48x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a6, a7, a8, a9;
  a0=arg[0]? arg[0][0] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a1=arg[0]? arg[0][4] : 0;
  if (res[0]!=0) res[0][1]=a1;
  a2=arg[0]? arg[0][8] : 0;
  if (res[0]!=0) res[0][2]=a2;
  a3=6.2746131716582509e-01;
  a4=arg[0]? arg[0][1] : 0;
  a5=(a3*a4);
  a0=(a0+a5);
  a5=3.9370770453947218e-01;
  a6=arg[0]? arg[0][2] : 0;
  a7=(a5*a6);
  a0=(a0+a7);
  a7=2.4703635486867070e-01;
  a8=arg[0]? arg[0][3] : 0;
  a9=(a7*a8);
  a0=(a0+a9);
  if (res[0]!=0) res[0][3]=a0;
  a0=arg[0]? arg[0][5] : 0;
  a9=(a3*a0);
  a1=(a1+a9);
  a9=arg[0]? arg[0][6] : 0;
  a10=(a5*a9);
  a1=(a1+a10);
  a10=arg[0]? arg[0][7] : 0;
  a11=(a7*a10);
  a1=(a1+a11);
  if (res[0]!=0) res[0][4]=a1;
  a1=arg[0]? arg[0][9] : 0;
  a11=(a3*a1);
  a2=(a2+a11);
  a11=arg[0]? arg[0][10] : 0;
  a12=(a5*a11);
  a2=(a2+a12);
  a12=arg[0]? arg[0][11] : 0;
  a13=(a7*a12);
  a2=(a2+a13);
  if (res[0]!=0) res[0][5]=a2;
  a2=arg[0]? arg[0][12] : 0;
  if (res[0]!=0) res[0][6]=a2;
  a13=arg[0]? arg[0][16] : 0;
  if (res[0]!=0) res[0][7]=a13;
  a14=arg[0]? arg[0][20] : 0;
  if (res[0]!=0) res[0][8]=a14;
  a15=arg[0]? arg[0][13] : 0;
  a16=(a3*a15);
  a2=(a2+a16);
  a16=arg[0]? arg[0][14] : 0;
  a17=(a5*a16);
  a2=(a2+a17);
  a17=arg[0]? arg[0][15] : 0;
  a18=(a7*a17);
  a2=(a2+a18);
  if (res[0]!=0) res[0][9]=a2;
  a2=arg[0]? arg[0][17] : 0;
  a18=(a3*a2);
  a13=(a13+a18);
  a18=arg[0]? arg[0][18] : 0;
  a19=(a5*a18);
  a13=(a13+a19);
  a19=arg[0]? arg[0][19] : 0;
  a20=(a7*a19);
  a13=(a13+a20);
  if (res[0]!=0) res[0][10]=a13;
  a13=arg[0]? arg[0][21] : 0;
  a20=(a3*a13);
  a14=(a14+a20);
  a20=arg[0]? arg[0][22] : 0;
  a21=(a5*a20);
  a14=(a14+a21);
  a21=arg[0]? arg[0][23] : 0;
  a22=(a7*a21);
  a14=(a14+a22);
  if (res[0]!=0) res[0][11]=a14;
  a14=arg[0]? arg[0][24] : 0;
  if (res[0]!=0) res[0][12]=a14;
  a22=arg[0]? arg[0][28] : 0;
  if (res[0]!=0) res[0][13]=a22;
  a23=arg[0]? arg[0][32] : 0;
  if (res[0]!=0) res[0][14]=a23;
  a24=arg[0]? arg[0][25] : 0;
  a25=(a3*a24);
  a14=(a14+a25);
  a25=arg[0]? arg[0][26] : 0;
  a26=(a5*a25);
  a14=(a14+a26);
  a26=arg[0]? arg[0][27] : 0;
  a27=(a7*a26);
  a14=(a14+a27);
  if (res[0]!=0) res[0][15]=a14;
  a14=arg[0]? arg[0][29] : 0;
  a27=(a3*a14);
  a22=(a22+a27);
  a27=arg[0]? arg[0][30] : 0;
  a28=(a5*a27);
  a22=(a22+a28);
  a28=arg[0]? arg[0][31] : 0;
  a29=(a7*a28);
  a22=(a22+a29);
  if (res[0]!=0) res[0][16]=a22;
  a22=arg[0]? arg[0][33] : 0;
  a29=(a3*a22);
  a23=(a23+a29);
  a29=arg[0]? arg[0][34] : 0;
  a30=(a5*a29);
  a23=(a23+a30);
  a30=arg[0]? arg[0][35] : 0;
  a31=(a7*a30);
  a23=(a23+a31);
  if (res[0]!=0) res[0][17]=a23;
  a23=arg[0]? arg[0][36] : 0;
  if (res[0]!=0) res[0][18]=a23;
  a31=arg[0]? arg[0][40] : 0;
  if (res[0]!=0) res[0][19]=a31;
  a32=arg[0]? arg[0][44] : 0;
  if (res[0]!=0) res[0][20]=a32;
  a33=arg[0]? arg[0][37] : 0;
  a34=(a3*a33);
  a23=(a23+a34);
  a34=arg[0]? arg[0][38] : 0;
  a35=(a5*a34);
  a23=(a23+a35);
  a35=arg[0]? arg[0][39] : 0;
  a36=(a7*a35);
  a23=(a23+a36);
  if (res[0]!=0) res[0][21]=a23;
  a23=arg[0]? arg[0][41] : 0;
  a36=(a3*a23);
  a31=(a31+a36);
  a36=arg[0]? arg[0][42] : 0;
  a37=(a5*a36);
  a31=(a31+a37);
  a37=arg[0]? arg[0][43] : 0;
  a38=(a7*a37);
  a31=(a31+a38);
  if (res[0]!=0) res[0][22]=a31;
  a31=arg[0]? arg[0][45] : 0;
  a38=(a3*a31);
  a32=(a32+a38);
  a38=arg[0]? arg[0][46] : 0;
  a39=(a5*a38);
  a32=(a32+a39);
  a39=arg[0]? arg[0][47] : 0;
  a40=(a7*a39);
  a32=(a32+a40);
  if (res[0]!=0) res[0][23]=a32;
  a32=arg[0]? arg[0][48] : 0;
  if (res[0]!=0) res[0][24]=a32;
  a40=arg[0]? arg[0][52] : 0;
  if (res[0]!=0) res[0][25]=a40;
  a41=arg[0]? arg[0][56] : 0;
  if (res[0]!=0) res[0][26]=a41;
  a42=arg[0]? arg[0][49] : 0;
  a43=(a3*a42);
  a32=(a32+a43);
  a43=arg[0]? arg[0][50] : 0;
  a44=(a5*a43);
  a32=(a32+a44);
  a44=arg[0]? arg[0][51] : 0;
  a45=(a7*a44);
  a32=(a32+a45);
  if (res[0]!=0) res[0][27]=a32;
  a32=arg[0]? arg[0][53] : 0;
  a45=(a3*a32);
  a40=(a40+a45);
  a45=arg[0]? arg[0][54] : 0;
  a46=(a5*a45);
  a40=(a40+a46);
  a46=arg[0]? arg[0][55] : 0;
  a47=(a7*a46);
  a40=(a40+a47);
  if (res[0]!=0) res[0][28]=a40;
  a40=arg[0]? arg[0][57] : 0;
  a47=(a3*a40);
  a41=(a41+a47);
  a47=arg[0]? arg[0][58] : 0;
  a48=(a5*a47);
  a41=(a41+a48);
  a48=arg[0]? arg[0][59] : 0;
  a49=(a7*a48);
  a41=(a41+a49);
  if (res[0]!=0) res[0][29]=a41;
  a41=1.2549226343316502e+00;
  a49=(a41*a6);
  a49=(a4+a49);
  a50=1.1811231136184166e+00;
  a51=(a50*a8);
  a49=(a49+a51);
  a49=(a49-a15);
  if (res[0]!=0) res[0][30]=a49;
  a49=(a41*a9);
  a49=(a0+a49);
  a51=(a50*a10);
  a49=(a49+a51);
  a49=(a49-a2);
  if (res[0]!=0) res[0][31]=a49;
  a49=(a41*a11);
  a49=(a1+a49);
  a51=(a50*a12);
  a49=(a49+a51);
  a49=(a49-a13);
  if (res[0]!=0) res[0][32]=a49;
  a49=(a41*a16);
  a49=(a15+a49);
  a51=(a50*a17);
  a49=(a49+a51);
  a49=(a49-a24);
  if (res[0]!=0) res[0][33]=a49;
  a49=(a41*a18);
  a49=(a2+a49);
  a51=(a50*a19);
  a49=(a49+a51);
  a49=(a49-a14);
  if (res[0]!=0) res[0][34]=a49;
  a49=(a41*a20);
  a49=(a13+a49);
  a51=(a50*a21);
  a49=(a49+a51);
  a49=(a49-a22);
  if (res[0]!=0) res[0][35]=a49;
  a49=(a41*a25);
  a49=(a24+a49);
  a51=(a50*a26);
  a49=(a49+a51);
  a49=(a49-a33);
  if (res[0]!=0) res[0][36]=a49;
  a49=(a41*a27);
  a49=(a14+a49);
  a51=(a50*a28);
  a49=(a49+a51);
  a49=(a49-a23);
  if (res[0]!=0) res[0][37]=a49;
  a49=(a41*a29);
  a49=(a22+a49);
  a51=(a50*a30);
  a49=(a49+a51);
  a49=(a49-a31);
  if (res[0]!=0) res[0][38]=a49;
  a49=(a41*a34);
  a49=(a33+a49);
  a51=(a50*a35);
  a49=(a49+a51);
  a49=(a49-a42);
  if (res[0]!=0) res[0][39]=a49;
  a49=(a41*a36);
  a49=(a23+a49);
  a51=(a50*a37);
  a49=(a49+a51);
  a49=(a49-a32);
  if (res[0]!=0) res[0][40]=a49;
  a49=(a41*a38);
  a49=(a31+a49);
  a51=(a50*a39);
  a49=(a49+a51);
  a49=(a49-a40);
  if (res[0]!=0) res[0][41]=a49;
  a49=2.;
  a6=(a49*a6);
  a6=(a3*a6);
  a6=(a4+a6);
  a51=3.;
  a8=(a51*a8);
  a8=(a5*a8);
  a6=(a6+a8);
  a8=casadi_sq(a6);
  a9=(a49*a9);
  a9=(a3*a9);
  a9=(a0+a9);
  a10=(a51*a10);
  a10=(a5*a10);
  a9=(a9+a10);
  a10=casadi_sq(a9);
  a8=(a8+a10);
  a11=(a49*a11);
  a11=(a3*a11);
  a11=(a1+a11);
  a12=(a51*a12);
  a12=(a5*a12);
  a11=(a11+a12);
  a12=casadi_sq(a11);
  a8=(a8+a12);
  if (res[0]!=0) res[0][42]=a8;
  a16=(a49*a16);
  a16=(a3*a16);
  a15=(a15+a16);
  a17=(a51*a17);
  a17=(a5*a17);
  a15=(a15+a17);
  a17=casadi_sq(a15);
  a18=(a49*a18);
  a18=(a3*a18);
  a2=(a2+a18);
  a19=(a51*a19);
  a19=(a5*a19);
  a2=(a2+a19);
  a19=casadi_sq(a2);
  a17=(a17+a19);
  a20=(a49*a20);
  a20=(a3*a20);
  a13=(a13+a20);
  a21=(a51*a21);
  a21=(a5*a21);
  a13=(a13+a21);
  a21=casadi_sq(a13);
  a17=(a17+a21);
  if (res[0]!=0) res[0][43]=a17;
  a25=(a49*a25);
  a25=(a3*a25);
  a24=(a24+a25);
  a26=(a51*a26);
  a26=(a5*a26);
  a24=(a24+a26);
  a26=casadi_sq(a24);
  a27=(a49*a27);
  a27=(a3*a27);
  a14=(a14+a27);
  a28=(a51*a28);
  a28=(a5*a28);
  a14=(a14+a28);
  a28=casadi_sq(a14);
  a26=(a26+a28);
  a29=(a49*a29);
  a29=(a3*a29);
  a22=(a22+a29);
  a30=(a51*a30);
  a30=(a5*a30);
  a22=(a22+a30);
  a30=casadi_sq(a22);
  a26=(a26+a30);
  if (res[0]!=0) res[0][44]=a26;
  a34=(a49*a34);
  a34=(a3*a34);
  a33=(a33+a34);
  a35=(a51*a35);
  a35=(a5*a35);
  a33=(a33+a35);
  a35=casadi_sq(a33);
  a36=(a49*a36);
  a36=(a3*a36);
  a23=(a23+a36);
  a37=(a51*a37);
  a37=(a5*a37);
  a23=(a23+a37);
  a37=casadi_sq(a23);
  a35=(a35+a37);
  a38=(a49*a38);
  a38=(a3*a38);
  a31=(a31+a38);
  a39=(a51*a39);
  a39=(a5*a39);
  a31=(a31+a39);
  a39=casadi_sq(a31);
  a35=(a35+a39);
  if (res[0]!=0) res[0][45]=a35;
  a43=(a49*a43);
  a43=(a3*a43);
  a42=(a42+a43);
  a44=(a51*a44);
  a44=(a5*a44);
  a42=(a42+a44);
  a44=casadi_sq(a42);
  a45=(a49*a45);
  a45=(a3*a45);
  a32=(a32+a45);
  a46=(a51*a46);
  a46=(a5*a46);
  a32=(a32+a46);
  a46=casadi_sq(a32);
  a44=(a44+a46);
  a47=(a49*a47);
  a47=(a3*a47);
  a40=(a40+a47);
  a48=(a51*a48);
  a48=(a5*a48);
  a40=(a40+a48);
  a48=casadi_sq(a40);
  a44=(a44+a48);
  if (res[0]!=0) res[0][46]=a44;
  a44=casadi_sq(a4);
  a48=casadi_sq(a0);
  a44=(a44+a48);
  a48=casadi_sq(a1);
  a44=(a44+a48);
  if (res[0]!=0) res[0][47]=a44;
  a44=1.;
  if (res[1]!=0) res[1][0]=a44;
  if (res[1]!=0) res[1][1]=a44;
  if (res[1]!=0) res[1][2]=a44;
  if (res[1]!=0) res[1][3]=a44;
  if (res[1]!=0) res[1][4]=a3;
  if (res[1]!=0) res[1][5]=a5;
  if (res[1]!=0) res[1][6]=a7;
  if (res[1]!=0) res[1][7]=a44;
  if (res[1]!=0) res[1][8]=a3;
  if (res[1]!=0) res[1][9]=a5;
  if (res[1]!=0) res[1][10]=a7;
  if (res[1]!=0) res[1][11]=a44;
  if (res[1]!=0) res[1][12]=a3;
  if (res[1]!=0) res[1][13]=a5;
  if (res[1]!=0) res[1][14]=a7;
  if (res[1]!=0) res[1][15]=a44;
  if (res[1]!=0) res[1][16]=a44;
  if (res[1]!=0) res[1][17]=a44;
  if (res[1]!=0) res[1][18]=a44;
  if (res[1]!=0) res[1][19]=a3;
  if (res[1]!=0) res[1][20]=a5;
  if (res[1]!=0) res[1][21]=a7;
  if (res[1]!=0) res[1][22]=a44;
  if (res[1]!=0) res[1][23]=a3;
  if (res[1]!=0) res[1][24]=a5;
  if (res[1]!=0) res[1][25]=a7;
  if (res[1]!=0) res[1][26]=a44;
  if (res[1]!=0) res[1][27]=a3;
  if (res[1]!=0) res[1][28]=a5;
  if (res[1]!=0) res[1][29]=a7;
  if (res[1]!=0) res[1][30]=a44;
  if (res[1]!=0) res[1][31]=a44;
  if (res[1]!=0) res[1][32]=a44;
  if (res[1]!=0) res[1][33]=a44;
  if (res[1]!=0) res[1][34]=a3;
  if (res[1]!=0) res[1][35]=a5;
  if (res[1]!=0) res[1][36]=a7;
  if (res[1]!=0) res[1][37]=a44;
  if (res[1]!=0) res[1][38]=a3;
  if (res[1]!=0) res[1][39]=a5;
  if (res[1]!=0) res[1][40]=a7;
  if (res[1]!=0) res[1][41]=a44;
  if (res[1]!=0) res[1][42]=a3;
  if (res[1]!=0) res[1][43]=a5;
  if (res[1]!=0) res[1][44]=a7;
  if (res[1]!=0) res[1][45]=a44;
  if (res[1]!=0) res[1][46]=a44;
  if (res[1]!=0) res[1][47]=a44;
  if (res[1]!=0) res[1][48]=a44;
  if (res[1]!=0) res[1][49]=a3;
  if (res[1]!=0) res[1][50]=a5;
  if (res[1]!=0) res[1][51]=a7;
  if (res[1]!=0) res[1][52]=a44;
  if (res[1]!=0) res[1][53]=a3;
  if (res[1]!=0) res[1][54]=a5;
  if (res[1]!=0) res[1][55]=a7;
  if (res[1]!=0) res[1][56]=a44;
  if (res[1]!=0) res[1][57]=a3;
  if (res[1]!=0) res[1][58]=a5;
  if (res[1]!=0) res[1][59]=a7;
  if (res[1]!=0) res[1][60]=a44;
  if (res[1]!=0) res[1][61]=a44;
  if (res[1]!=0) res[1][62]=a44;
  if (res[1]!=0) res[1][63]=a44;
  if (res[1]!=0) res[1][64]=a3;
  if (res[1]!=0) res[1][65]=a5;
  if (res[1]!=0) res[1][66]=a7;
  if (res[1]!=0) res[1][67]=a44;
  if (res[1]!=0) res[1][68]=a3;
  if (res[1]!=0) res[1][69]=a5;
  if (res[1]!=0) res[1][70]=a7;
  if (res[1]!=0) res[1][71]=a44;
  if (res[1]!=0) res[1][72]=a3;
  if (res[1]!=0) res[1][73]=a5;
  if (res[1]!=0) res[1][74]=a7;
  if (res[1]!=0) res[1][75]=a44;
  if (res[1]!=0) res[1][76]=a41;
  if (res[1]!=0) res[1][77]=a50;
  a7=-1.;
  if (res[1]!=0) res[1][78]=a7;
  if (res[1]!=0) res[1][79]=a44;
  if (res[1]!=0) res[1][80]=a41;
  if (res[1]!=0) res[1][81]=a50;
  if (res[1]!=0) res[1][82]=a7;
  if (res[1]!=0) res[1][83]=a44;
  if (res[1]!=0) res[1][84]=a41;
  if (res[1]!=0) res[1][85]=a50;
  if (res[1]!=0) res[1][86]=a7;
  if (res[1]!=0) res[1][87]=a44;
  if (res[1]!=0) res[1][88]=a41;
  if (res[1]!=0) res[1][89]=a50;
  if (res[1]!=0) res[1][90]=a7;
  if (res[1]!=0) res[1][91]=a44;
  if (res[1]!=0) res[1][92]=a41;
  if (res[1]!=0) res[1][93]=a50;
  if (res[1]!=0) res[1][94]=a7;
  if (res[1]!=0) res[1][95]=a44;
  if (res[1]!=0) res[1][96]=a41;
  if (res[1]!=0) res[1][97]=a50;
  if (res[1]!=0) res[1][98]=a7;
  if (res[1]!=0) res[1][99]=a44;
  if (res[1]!=0) res[1][100]=a41;
  if (res[1]!=0) res[1][101]=a50;
  if (res[1]!=0) res[1][102]=a7;
  if (res[1]!=0) res[1][103]=a44;
  if (res[1]!=0) res[1][104]=a41;
  if (res[1]!=0) res[1][105]=a50;
  if (res[1]!=0) res[1][106]=a7;
  if (res[1]!=0) res[1][107]=a44;
  if (res[1]!=0) res[1][108]=a41;
  if (res[1]!=0) res[1][109]=a50;
  if (res[1]!=0) res[1][110]=a7;
  if (res[1]!=0) res[1][111]=a44;
  if (res[1]!=0) res[1][112]=a41;
  if (res[1]!=0) res[1][113]=a50;
  if (res[1]!=0) res[1][114]=a7;
  if (res[1]!=0) res[1][115]=a44;
  if (res[1]!=0) res[1][116]=a41;
  if (res[1]!=0) res[1][117]=a50;
  if (res[1]!=0) res[1][118]=a7;
  if (res[1]!=0) res[1][119]=a44;
  if (res[1]!=0) res[1][120]=a41;
  if (res[1]!=0) res[1][121]=a50;
  if (res[1]!=0) res[1][122]=a7;
  a6=(a6+a6);
  if (res[1]!=0) res[1][123]=a6;
  a7=(a3*a6);
  a7=(a49*a7);
  if (res[1]!=0) res[1][124]=a7;
  a6=(a5*a6);
  a6=(a51*a6);
  if (res[1]!=0) res[1][125]=a6;
  a9=(a9+a9);
  if (res[1]!=0) res[1][126]=a9;
  a6=(a3*a9);
  a6=(a49*a6);
  if (res[1]!=0) res[1][127]=a6;
  a9=(a5*a9);
  a9=(a51*a9);
  if (res[1]!=0) res[1][128]=a9;
  a11=(a11+a11);
  if (res[1]!=0) res[1][129]=a11;
  a9=(a3*a11);
  a9=(a49*a9);
  if (res[1]!=0) res[1][130]=a9;
  a11=(a5*a11);
  a11=(a51*a11);
  if (res[1]!=0) res[1][131]=a11;
  a15=(a15+a15);
  if (res[1]!=0) res[1][132]=a15;
  a11=(a3*a15);
  a11=(a49*a11);
  if (res[1]!=0) res[1][133]=a11;
  a15=(a5*a15);
  a15=(a51*a15);
  if (res[1]!=0) res[1][134]=a15;
  a2=(a2+a2);
  if (res[1]!=0) res[1][135]=a2;
  a15=(a3*a2);
  a15=(a49*a15);
  if (res[1]!=0) res[1][136]=a15;
  a2=(a5*a2);
  a2=(a51*a2);
  if (res[1]!=0) res[1][137]=a2;
  a13=(a13+a13);
  if (res[1]!=0) res[1][138]=a13;
  a2=(a3*a13);
  a2=(a49*a2);
  if (res[1]!=0) res[1][139]=a2;
  a13=(a5*a13);
  a13=(a51*a13);
  if (res[1]!=0) res[1][140]=a13;
  a24=(a24+a24);
  if (res[1]!=0) res[1][141]=a24;
  a13=(a3*a24);
  a13=(a49*a13);
  if (res[1]!=0) res[1][142]=a13;
  a24=(a5*a24);
  a24=(a51*a24);
  if (res[1]!=0) res[1][143]=a24;
  a14=(a14+a14);
  if (res[1]!=0) res[1][144]=a14;
  a24=(a3*a14);
  a24=(a49*a24);
  if (res[1]!=0) res[1][145]=a24;
  a14=(a5*a14);
  a14=(a51*a14);
  if (res[1]!=0) res[1][146]=a14;
  a22=(a22+a22);
  if (res[1]!=0) res[1][147]=a22;
  a14=(a3*a22);
  a14=(a49*a14);
  if (res[1]!=0) res[1][148]=a14;
  a22=(a5*a22);
  a22=(a51*a22);
  if (res[1]!=0) res[1][149]=a22;
  a33=(a33+a33);
  if (res[1]!=0) res[1][150]=a33;
  a22=(a3*a33);
  a22=(a49*a22);
  if (res[1]!=0) res[1][151]=a22;
  a33=(a5*a33);
  a33=(a51*a33);
  if (res[1]!=0) res[1][152]=a33;
  a23=(a23+a23);
  if (res[1]!=0) res[1][153]=a23;
  a33=(a3*a23);
  a33=(a49*a33);
  if (res[1]!=0) res[1][154]=a33;
  a23=(a5*a23);
  a23=(a51*a23);
  if (res[1]!=0) res[1][155]=a23;
  a31=(a31+a31);
  if (res[1]!=0) res[1][156]=a31;
  a23=(a3*a31);
  a23=(a49*a23);
  if (res[1]!=0) res[1][157]=a23;
  a31=(a5*a31);
  a31=(a51*a31);
  if (res[1]!=0) res[1][158]=a31;
  a42=(a42+a42);
  if (res[1]!=0) res[1][159]=a42;
  a31=(a3*a42);
  a31=(a49*a31);
  if (res[1]!=0) res[1][160]=a31;
  a42=(a5*a42);
  a42=(a51*a42);
  if (res[1]!=0) res[1][161]=a42;
  a32=(a32+a32);
  if (res[1]!=0) res[1][162]=a32;
  a42=(a3*a32);
  a42=(a49*a42);
  if (res[1]!=0) res[1][163]=a42;
  a32=(a5*a32);
  a32=(a51*a32);
  if (res[1]!=0) res[1][164]=a32;
  a40=(a40+a40);
  if (res[1]!=0) res[1][165]=a40;
  a3=(a3*a40);
  a49=(a49*a3);
  if (res[1]!=0) res[1][166]=a49;
  a5=(a5*a40);
  a51=(a51*a5);
  if (res[1]!=0) res[1][167]=a51;
  a4=(a4+a4);
  if (res[1]!=0) res[1][168]=a4;
  a0=(a0+a0);
  if (res[1]!=0) res[1][169]=a0;
  a1=(a1+a1);
  if (res[1]!=0) res[1][170]=a1;
  return 0;
}

CASADI_SYMBOL_EXPORT int car0_Arcsolver_0_constr_h_fun_jac_uxt_zt(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int car0_Arcsolver_0_constr_h_fun_jac_uxt_zt_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int car0_Arcsolver_0_constr_h_fun_jac_uxt_zt_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void car0_Arcsolver_0_constr_h_fun_jac_uxt_zt_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int car0_Arcsolver_0_constr_h_fun_jac_uxt_zt_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void car0_Arcsolver_0_constr_h_fun_jac_uxt_zt_release(int mem) {
}

CASADI_SYMBOL_EXPORT void car0_Arcsolver_0_constr_h_fun_jac_uxt_zt_incref(void) {
}

CASADI_SYMBOL_EXPORT void car0_Arcsolver_0_constr_h_fun_jac_uxt_zt_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int car0_Arcsolver_0_constr_h_fun_jac_uxt_zt_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int car0_Arcsolver_0_constr_h_fun_jac_uxt_zt_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real car0_Arcsolver_0_constr_h_fun_jac_uxt_zt_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car0_Arcsolver_0_constr_h_fun_jac_uxt_zt_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car0_Arcsolver_0_constr_h_fun_jac_uxt_zt_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* car0_Arcsolver_0_constr_h_fun_jac_uxt_zt_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    case 3: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* car0_Arcsolver_0_constr_h_fun_jac_uxt_zt_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    case 1: return casadi_s3;
    case 2: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int car0_Arcsolver_0_constr_h_fun_jac_uxt_zt_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
