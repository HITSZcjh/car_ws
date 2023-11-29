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
  #define CASADI_PREFIX(ID) car0_Arcsolver_1_cost_ext_cost_fun_jac_ ## ID
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

static const casadi_int casadi_s0[64] = {60, 1, 0, 60, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59};
static const casadi_int casadi_s1[3] = {0, 0, 0};
static const casadi_int casadi_s2[5] = {1, 1, 0, 1, 0};

/* car0_Arcsolver_1_cost_ext_cost_fun_jac:(i0[60],i1[],i2[],i3[])->(o0,o1[60]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a7, a8, a9;
  a0=1.2812591744790700e+00;
  a1=arg[0]? arg[0][2] : 0;
  a2=(a0*a1);
  a3=6.1560940207004555e-01;
  a4=arg[0]? arg[0][3] : 0;
  a5=(a3*a4);
  a2=(a2+a5);
  a5=(a2*a1);
  a6=(a3*a1);
  a7=3.9437759714891024e-01;
  a8=(a7*a4);
  a6=(a6+a8);
  a8=(a6*a4);
  a5=(a5+a8);
  a8=arg[0]? arg[0][6] : 0;
  a9=(a0*a8);
  a10=arg[0]? arg[0][7] : 0;
  a11=(a3*a10);
  a9=(a9+a11);
  a11=(a9*a8);
  a5=(a5+a11);
  a11=(a3*a8);
  a12=(a7*a10);
  a11=(a11+a12);
  a12=(a11*a10);
  a5=(a5+a12);
  a12=arg[0]? arg[0][10] : 0;
  a13=(a0*a12);
  a14=arg[0]? arg[0][11] : 0;
  a15=(a3*a14);
  a13=(a13+a15);
  a15=(a13*a12);
  a5=(a5+a15);
  a15=(a3*a12);
  a16=(a7*a14);
  a15=(a15+a16);
  a16=(a15*a14);
  a5=(a5+a16);
  a16=arg[0]? arg[0][14] : 0;
  a17=(a0*a16);
  a18=arg[0]? arg[0][15] : 0;
  a19=(a3*a18);
  a17=(a17+a19);
  a19=(a17*a16);
  a5=(a5+a19);
  a19=(a3*a16);
  a20=(a7*a18);
  a19=(a19+a20);
  a20=(a19*a18);
  a5=(a5+a20);
  a20=arg[0]? arg[0][18] : 0;
  a21=(a0*a20);
  a22=arg[0]? arg[0][19] : 0;
  a23=(a3*a22);
  a21=(a21+a23);
  a23=(a21*a20);
  a5=(a5+a23);
  a23=(a3*a20);
  a24=(a7*a22);
  a23=(a23+a24);
  a24=(a23*a22);
  a5=(a5+a24);
  a24=arg[0]? arg[0][22] : 0;
  a25=(a0*a24);
  a26=arg[0]? arg[0][23] : 0;
  a27=(a3*a26);
  a25=(a25+a27);
  a27=(a25*a24);
  a5=(a5+a27);
  a27=(a3*a24);
  a28=(a7*a26);
  a27=(a27+a28);
  a28=(a27*a26);
  a5=(a5+a28);
  a28=arg[0]? arg[0][26] : 0;
  a29=(a0*a28);
  a30=arg[0]? arg[0][27] : 0;
  a31=(a3*a30);
  a29=(a29+a31);
  a31=(a29*a28);
  a5=(a5+a31);
  a31=(a3*a28);
  a32=(a7*a30);
  a31=(a31+a32);
  a32=(a31*a30);
  a5=(a5+a32);
  a32=arg[0]? arg[0][30] : 0;
  a33=(a0*a32);
  a34=arg[0]? arg[0][31] : 0;
  a35=(a3*a34);
  a33=(a33+a35);
  a35=(a33*a32);
  a5=(a5+a35);
  a35=(a3*a32);
  a36=(a7*a34);
  a35=(a35+a36);
  a36=(a35*a34);
  a5=(a5+a36);
  a36=arg[0]? arg[0][34] : 0;
  a37=(a0*a36);
  a38=arg[0]? arg[0][35] : 0;
  a39=(a3*a38);
  a37=(a37+a39);
  a39=(a37*a36);
  a5=(a5+a39);
  a39=(a3*a36);
  a40=(a7*a38);
  a39=(a39+a40);
  a40=(a39*a38);
  a5=(a5+a40);
  a40=arg[0]? arg[0][38] : 0;
  a41=(a0*a40);
  a42=arg[0]? arg[0][39] : 0;
  a43=(a3*a42);
  a41=(a41+a43);
  a43=(a41*a40);
  a5=(a5+a43);
  a43=(a3*a40);
  a44=(a7*a42);
  a43=(a43+a44);
  a44=(a43*a42);
  a5=(a5+a44);
  a44=arg[0]? arg[0][42] : 0;
  a45=(a0*a44);
  a46=arg[0]? arg[0][43] : 0;
  a47=(a3*a46);
  a45=(a45+a47);
  a47=(a45*a44);
  a5=(a5+a47);
  a47=(a3*a44);
  a48=(a7*a46);
  a47=(a47+a48);
  a48=(a47*a46);
  a5=(a5+a48);
  a48=arg[0]? arg[0][46] : 0;
  a49=(a0*a48);
  a50=arg[0]? arg[0][47] : 0;
  a51=(a3*a50);
  a49=(a49+a51);
  a51=(a49*a48);
  a5=(a5+a51);
  a51=(a3*a48);
  a52=(a7*a50);
  a51=(a51+a52);
  a52=(a51*a50);
  a5=(a5+a52);
  a52=arg[0]? arg[0][50] : 0;
  a53=(a0*a52);
  a54=arg[0]? arg[0][51] : 0;
  a55=(a3*a54);
  a53=(a53+a55);
  a55=(a53*a52);
  a5=(a5+a55);
  a55=(a3*a52);
  a56=(a7*a54);
  a55=(a55+a56);
  a56=(a55*a54);
  a5=(a5+a56);
  a56=arg[0]? arg[0][54] : 0;
  a57=(a0*a56);
  a58=arg[0]? arg[0][55] : 0;
  a59=(a3*a58);
  a57=(a57+a59);
  a59=(a57*a56);
  a5=(a5+a59);
  a59=(a3*a56);
  a60=(a7*a58);
  a59=(a59+a60);
  a60=(a59*a58);
  a5=(a5+a60);
  a60=arg[0]? arg[0][58] : 0;
  a61=(a0*a60);
  a62=arg[0]? arg[0][59] : 0;
  a63=(a3*a62);
  a61=(a61+a63);
  a63=(a61*a60);
  a5=(a5+a63);
  a63=(a3*a60);
  a64=(a7*a62);
  a63=(a63+a64);
  a64=(a63*a62);
  a5=(a5+a64);
  if (res[0]!=0) res[0][0]=a5;
  a5=0.;
  if (res[1]!=0) res[1][0]=a5;
  if (res[1]!=0) res[1][1]=a5;
  a64=(a3*a4);
  a64=(a64+a2);
  a2=(a0*a1);
  a64=(a64+a2);
  if (res[1]!=0) res[1][2]=a64;
  a4=(a7*a4);
  a6=(a6+a4);
  a1=(a3*a1);
  a6=(a6+a1);
  if (res[1]!=0) res[1][3]=a6;
  if (res[1]!=0) res[1][4]=a5;
  if (res[1]!=0) res[1][5]=a5;
  a6=(a3*a10);
  a6=(a6+a9);
  a9=(a0*a8);
  a6=(a6+a9);
  if (res[1]!=0) res[1][6]=a6;
  a10=(a7*a10);
  a11=(a11+a10);
  a8=(a3*a8);
  a11=(a11+a8);
  if (res[1]!=0) res[1][7]=a11;
  if (res[1]!=0) res[1][8]=a5;
  if (res[1]!=0) res[1][9]=a5;
  a11=(a3*a14);
  a11=(a11+a13);
  a13=(a0*a12);
  a11=(a11+a13);
  if (res[1]!=0) res[1][10]=a11;
  a14=(a7*a14);
  a15=(a15+a14);
  a12=(a3*a12);
  a15=(a15+a12);
  if (res[1]!=0) res[1][11]=a15;
  if (res[1]!=0) res[1][12]=a5;
  if (res[1]!=0) res[1][13]=a5;
  a15=(a3*a18);
  a15=(a15+a17);
  a17=(a0*a16);
  a15=(a15+a17);
  if (res[1]!=0) res[1][14]=a15;
  a18=(a7*a18);
  a19=(a19+a18);
  a16=(a3*a16);
  a19=(a19+a16);
  if (res[1]!=0) res[1][15]=a19;
  if (res[1]!=0) res[1][16]=a5;
  if (res[1]!=0) res[1][17]=a5;
  a19=(a3*a22);
  a19=(a19+a21);
  a21=(a0*a20);
  a19=(a19+a21);
  if (res[1]!=0) res[1][18]=a19;
  a22=(a7*a22);
  a23=(a23+a22);
  a20=(a3*a20);
  a23=(a23+a20);
  if (res[1]!=0) res[1][19]=a23;
  if (res[1]!=0) res[1][20]=a5;
  if (res[1]!=0) res[1][21]=a5;
  a23=(a3*a26);
  a23=(a23+a25);
  a25=(a0*a24);
  a23=(a23+a25);
  if (res[1]!=0) res[1][22]=a23;
  a26=(a7*a26);
  a27=(a27+a26);
  a24=(a3*a24);
  a27=(a27+a24);
  if (res[1]!=0) res[1][23]=a27;
  if (res[1]!=0) res[1][24]=a5;
  if (res[1]!=0) res[1][25]=a5;
  a27=(a3*a30);
  a27=(a27+a29);
  a29=(a0*a28);
  a27=(a27+a29);
  if (res[1]!=0) res[1][26]=a27;
  a30=(a7*a30);
  a31=(a31+a30);
  a28=(a3*a28);
  a31=(a31+a28);
  if (res[1]!=0) res[1][27]=a31;
  if (res[1]!=0) res[1][28]=a5;
  if (res[1]!=0) res[1][29]=a5;
  a31=(a3*a34);
  a31=(a31+a33);
  a33=(a0*a32);
  a31=(a31+a33);
  if (res[1]!=0) res[1][30]=a31;
  a34=(a7*a34);
  a35=(a35+a34);
  a32=(a3*a32);
  a35=(a35+a32);
  if (res[1]!=0) res[1][31]=a35;
  if (res[1]!=0) res[1][32]=a5;
  if (res[1]!=0) res[1][33]=a5;
  a35=(a3*a38);
  a35=(a35+a37);
  a37=(a0*a36);
  a35=(a35+a37);
  if (res[1]!=0) res[1][34]=a35;
  a38=(a7*a38);
  a39=(a39+a38);
  a36=(a3*a36);
  a39=(a39+a36);
  if (res[1]!=0) res[1][35]=a39;
  if (res[1]!=0) res[1][36]=a5;
  if (res[1]!=0) res[1][37]=a5;
  a39=(a3*a42);
  a39=(a39+a41);
  a41=(a0*a40);
  a39=(a39+a41);
  if (res[1]!=0) res[1][38]=a39;
  a42=(a7*a42);
  a43=(a43+a42);
  a40=(a3*a40);
  a43=(a43+a40);
  if (res[1]!=0) res[1][39]=a43;
  if (res[1]!=0) res[1][40]=a5;
  if (res[1]!=0) res[1][41]=a5;
  a43=(a3*a46);
  a43=(a43+a45);
  a45=(a0*a44);
  a43=(a43+a45);
  if (res[1]!=0) res[1][42]=a43;
  a46=(a7*a46);
  a47=(a47+a46);
  a44=(a3*a44);
  a47=(a47+a44);
  if (res[1]!=0) res[1][43]=a47;
  if (res[1]!=0) res[1][44]=a5;
  if (res[1]!=0) res[1][45]=a5;
  a47=(a3*a50);
  a47=(a47+a49);
  a49=(a0*a48);
  a47=(a47+a49);
  if (res[1]!=0) res[1][46]=a47;
  a50=(a7*a50);
  a51=(a51+a50);
  a48=(a3*a48);
  a51=(a51+a48);
  if (res[1]!=0) res[1][47]=a51;
  if (res[1]!=0) res[1][48]=a5;
  if (res[1]!=0) res[1][49]=a5;
  a51=(a3*a54);
  a51=(a51+a53);
  a53=(a0*a52);
  a51=(a51+a53);
  if (res[1]!=0) res[1][50]=a51;
  a54=(a7*a54);
  a55=(a55+a54);
  a52=(a3*a52);
  a55=(a55+a52);
  if (res[1]!=0) res[1][51]=a55;
  if (res[1]!=0) res[1][52]=a5;
  if (res[1]!=0) res[1][53]=a5;
  a55=(a3*a58);
  a55=(a55+a57);
  a57=(a0*a56);
  a55=(a55+a57);
  if (res[1]!=0) res[1][54]=a55;
  a58=(a7*a58);
  a59=(a59+a58);
  a56=(a3*a56);
  a59=(a59+a56);
  if (res[1]!=0) res[1][55]=a59;
  if (res[1]!=0) res[1][56]=a5;
  if (res[1]!=0) res[1][57]=a5;
  a5=(a3*a62);
  a5=(a5+a61);
  a0=(a0*a60);
  a5=(a5+a0);
  if (res[1]!=0) res[1][58]=a5;
  a7=(a7*a62);
  a63=(a63+a7);
  a3=(a3*a60);
  a63=(a63+a3);
  if (res[1]!=0) res[1][59]=a63;
  return 0;
}

CASADI_SYMBOL_EXPORT int car0_Arcsolver_1_cost_ext_cost_fun_jac(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int car0_Arcsolver_1_cost_ext_cost_fun_jac_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int car0_Arcsolver_1_cost_ext_cost_fun_jac_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void car0_Arcsolver_1_cost_ext_cost_fun_jac_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int car0_Arcsolver_1_cost_ext_cost_fun_jac_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void car0_Arcsolver_1_cost_ext_cost_fun_jac_release(int mem) {
}

CASADI_SYMBOL_EXPORT void car0_Arcsolver_1_cost_ext_cost_fun_jac_incref(void) {
}

CASADI_SYMBOL_EXPORT void car0_Arcsolver_1_cost_ext_cost_fun_jac_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int car0_Arcsolver_1_cost_ext_cost_fun_jac_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int car0_Arcsolver_1_cost_ext_cost_fun_jac_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real car0_Arcsolver_1_cost_ext_cost_fun_jac_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car0_Arcsolver_1_cost_ext_cost_fun_jac_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car0_Arcsolver_1_cost_ext_cost_fun_jac_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* car0_Arcsolver_1_cost_ext_cost_fun_jac_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    case 3: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* car0_Arcsolver_1_cost_ext_cost_fun_jac_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    case 1: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int car0_Arcsolver_1_cost_ext_cost_fun_jac_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
