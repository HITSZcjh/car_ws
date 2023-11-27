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
  #define CASADI_PREFIX(ID) car1_Arcsolver_0_constr_h_fun_ ## ID
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

/* car1_Arcsolver_0_constr_h_fun:(i0[60],i1[],i2[],i3[])->(o0[48]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a6, a7, a8, a9;
  a0=arg[0]? arg[0][0] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a1=arg[0]? arg[0][4] : 0;
  if (res[0]!=0) res[0][1]=a1;
  a2=arg[0]? arg[0][8] : 0;
  if (res[0]!=0) res[0][2]=a2;
  a3=6.7573196850265771e-01;
  a4=arg[0]? arg[0][1] : 0;
  a5=(a3*a4);
  a0=(a0+a5);
  a5=4.5661369325647677e-01;
  a6=arg[0]? arg[0][2] : 0;
  a7=(a5*a6);
  a0=(a0+a7);
  a7=3.0854846978946776e-01;
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
  a7=(a7*a48);
  a41=(a41+a7);
  if (res[0]!=0) res[0][29]=a41;
  a41=1.3514639370053154e+00;
  a7=(a41*a6);
  a7=(a4+a7);
  a49=1.3698410797694303e+00;
  a50=(a49*a8);
  a7=(a7+a50);
  a7=(a7-a15);
  if (res[0]!=0) res[0][30]=a7;
  a7=(a41*a9);
  a7=(a0+a7);
  a50=(a49*a10);
  a7=(a7+a50);
  a7=(a7-a2);
  if (res[0]!=0) res[0][31]=a7;
  a7=(a41*a11);
  a7=(a1+a7);
  a50=(a49*a12);
  a7=(a7+a50);
  a7=(a7-a13);
  if (res[0]!=0) res[0][32]=a7;
  a7=(a41*a16);
  a7=(a15+a7);
  a50=(a49*a17);
  a7=(a7+a50);
  a7=(a7-a24);
  if (res[0]!=0) res[0][33]=a7;
  a7=(a41*a18);
  a7=(a2+a7);
  a50=(a49*a19);
  a7=(a7+a50);
  a7=(a7-a14);
  if (res[0]!=0) res[0][34]=a7;
  a7=(a41*a20);
  a7=(a13+a7);
  a50=(a49*a21);
  a7=(a7+a50);
  a7=(a7-a22);
  if (res[0]!=0) res[0][35]=a7;
  a7=(a41*a25);
  a7=(a24+a7);
  a50=(a49*a26);
  a7=(a7+a50);
  a7=(a7-a33);
  if (res[0]!=0) res[0][36]=a7;
  a7=(a41*a27);
  a7=(a14+a7);
  a50=(a49*a28);
  a7=(a7+a50);
  a7=(a7-a23);
  if (res[0]!=0) res[0][37]=a7;
  a7=(a41*a29);
  a7=(a22+a7);
  a50=(a49*a30);
  a7=(a7+a50);
  a7=(a7-a31);
  if (res[0]!=0) res[0][38]=a7;
  a7=(a41*a34);
  a7=(a33+a7);
  a50=(a49*a35);
  a7=(a7+a50);
  a7=(a7-a42);
  if (res[0]!=0) res[0][39]=a7;
  a7=(a41*a36);
  a7=(a23+a7);
  a50=(a49*a37);
  a7=(a7+a50);
  a7=(a7-a32);
  if (res[0]!=0) res[0][40]=a7;
  a41=(a41*a38);
  a41=(a31+a41);
  a49=(a49*a39);
  a41=(a41+a49);
  a41=(a41-a40);
  if (res[0]!=0) res[0][41]=a41;
  a41=2.;
  a6=(a41*a6);
  a6=(a3*a6);
  a6=(a4+a6);
  a49=3.;
  a8=(a49*a8);
  a8=(a5*a8);
  a6=(a6+a8);
  a6=casadi_sq(a6);
  a9=(a41*a9);
  a9=(a3*a9);
  a9=(a0+a9);
  a10=(a49*a10);
  a10=(a5*a10);
  a9=(a9+a10);
  a9=casadi_sq(a9);
  a6=(a6+a9);
  a11=(a41*a11);
  a11=(a3*a11);
  a11=(a1+a11);
  a12=(a49*a12);
  a12=(a5*a12);
  a11=(a11+a12);
  a11=casadi_sq(a11);
  a6=(a6+a11);
  if (res[0]!=0) res[0][42]=a6;
  a16=(a41*a16);
  a16=(a3*a16);
  a15=(a15+a16);
  a17=(a49*a17);
  a17=(a5*a17);
  a15=(a15+a17);
  a15=casadi_sq(a15);
  a18=(a41*a18);
  a18=(a3*a18);
  a2=(a2+a18);
  a19=(a49*a19);
  a19=(a5*a19);
  a2=(a2+a19);
  a2=casadi_sq(a2);
  a15=(a15+a2);
  a20=(a41*a20);
  a20=(a3*a20);
  a13=(a13+a20);
  a21=(a49*a21);
  a21=(a5*a21);
  a13=(a13+a21);
  a13=casadi_sq(a13);
  a15=(a15+a13);
  if (res[0]!=0) res[0][43]=a15;
  a25=(a41*a25);
  a25=(a3*a25);
  a24=(a24+a25);
  a26=(a49*a26);
  a26=(a5*a26);
  a24=(a24+a26);
  a24=casadi_sq(a24);
  a27=(a41*a27);
  a27=(a3*a27);
  a14=(a14+a27);
  a28=(a49*a28);
  a28=(a5*a28);
  a14=(a14+a28);
  a14=casadi_sq(a14);
  a24=(a24+a14);
  a29=(a41*a29);
  a29=(a3*a29);
  a22=(a22+a29);
  a30=(a49*a30);
  a30=(a5*a30);
  a22=(a22+a30);
  a22=casadi_sq(a22);
  a24=(a24+a22);
  if (res[0]!=0) res[0][44]=a24;
  a34=(a41*a34);
  a34=(a3*a34);
  a33=(a33+a34);
  a35=(a49*a35);
  a35=(a5*a35);
  a33=(a33+a35);
  a33=casadi_sq(a33);
  a36=(a41*a36);
  a36=(a3*a36);
  a23=(a23+a36);
  a37=(a49*a37);
  a37=(a5*a37);
  a23=(a23+a37);
  a23=casadi_sq(a23);
  a33=(a33+a23);
  a38=(a41*a38);
  a38=(a3*a38);
  a31=(a31+a38);
  a39=(a49*a39);
  a39=(a5*a39);
  a31=(a31+a39);
  a31=casadi_sq(a31);
  a33=(a33+a31);
  if (res[0]!=0) res[0][45]=a33;
  a43=(a41*a43);
  a43=(a3*a43);
  a42=(a42+a43);
  a44=(a49*a44);
  a44=(a5*a44);
  a42=(a42+a44);
  a42=casadi_sq(a42);
  a45=(a41*a45);
  a45=(a3*a45);
  a32=(a32+a45);
  a46=(a49*a46);
  a46=(a5*a46);
  a32=(a32+a46);
  a32=casadi_sq(a32);
  a42=(a42+a32);
  a41=(a41*a47);
  a3=(a3*a41);
  a40=(a40+a3);
  a49=(a49*a48);
  a5=(a5*a49);
  a40=(a40+a5);
  a40=casadi_sq(a40);
  a42=(a42+a40);
  if (res[0]!=0) res[0][46]=a42;
  a4=casadi_sq(a4);
  a0=casadi_sq(a0);
  a4=(a4+a0);
  a1=casadi_sq(a1);
  a4=(a4+a1);
  if (res[0]!=0) res[0][47]=a4;
  return 0;
}

CASADI_SYMBOL_EXPORT int car1_Arcsolver_0_constr_h_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int car1_Arcsolver_0_constr_h_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int car1_Arcsolver_0_constr_h_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void car1_Arcsolver_0_constr_h_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int car1_Arcsolver_0_constr_h_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void car1_Arcsolver_0_constr_h_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void car1_Arcsolver_0_constr_h_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void car1_Arcsolver_0_constr_h_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int car1_Arcsolver_0_constr_h_fun_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int car1_Arcsolver_0_constr_h_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real car1_Arcsolver_0_constr_h_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car1_Arcsolver_0_constr_h_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car1_Arcsolver_0_constr_h_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* car1_Arcsolver_0_constr_h_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    case 3: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* car1_Arcsolver_0_constr_h_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int car1_Arcsolver_0_constr_h_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
