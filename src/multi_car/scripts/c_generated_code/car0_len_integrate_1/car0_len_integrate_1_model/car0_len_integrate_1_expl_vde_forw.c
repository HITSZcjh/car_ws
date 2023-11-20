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
  #define CASADI_PREFIX(ID) car0_len_integrate_1_expl_vde_forw_ ## ID
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

static const casadi_int casadi_s0[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s1[33] = {5, 5, 0, 5, 10, 15, 20, 25, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4};
static const casadi_int casadi_s2[3] = {5, 0, 0};
static const casadi_int casadi_s3[3] = {0, 0, 0};

/* car0_len_integrate_1_expl_vde_forw:(i0[5],i1[5x5],i2[5x0],i3[],i4[])->(o0[5],o1[5x5],o2[5x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a4, a5, a6, a7, a8, a9;
  a0=1.;
  if (res[0]!=0) res[0][0]=a0;
  a0=7.4999999961205677e-01;
  a1=2.;
  a2=-3.7499999980601434e-01;
  a3=arg[0]? arg[0][0] : 0;
  a4=(a2*a3);
  a4=(a1*a4);
  a0=(a0+a4);
  a4=3.;
  a5=-7.1512208326494833e-15;
  a6=casadi_sq(a3);
  a6=(a5*a6);
  a6=(a4*a6);
  a0=(a0+a6);
  a6=4.;
  a7=-1.0337972228114721e-15;
  a8=casadi_sq(a3);
  a9=(a3*a8);
  a9=(a7*a9);
  a9=(a6*a9);
  a0=(a0+a9);
  a9=5.;
  a10=5.5129103862741875e-16;
  a11=casadi_sq(a3);
  a12=casadi_sq(a11);
  a12=(a10*a12);
  a12=(a9*a12);
  a0=(a0+a12);
  if (res[0]!=0) res[0][1]=a0;
  a12=-7.5000000000000000e-01;
  a13=-3.7499999999999994e-01;
  a14=(a13*a3);
  a14=(a1*a14);
  a12=(a12+a14);
  a14=5.7720803631475752e-17;
  a15=casadi_sq(a3);
  a15=(a14*a15);
  a15=(a4*a15);
  a12=(a12+a15);
  a15=3.5363967611591004e-18;
  a16=casadi_sq(a3);
  a17=(a3*a16);
  a17=(a15*a17);
  a17=(a6*a17);
  a12=(a12+a17);
  a17=-3.1029974265141000e-18;
  a18=casadi_sq(a3);
  a19=casadi_sq(a18);
  a19=(a17*a19);
  a19=(a9*a19);
  a12=(a12+a19);
  if (res[0]!=0) res[0][2]=a12;
  a19=0.;
  if (res[0]!=0) res[0][3]=a19;
  a20=casadi_sq(a0);
  a21=casadi_sq(a12);
  a20=(a20+a21);
  a20=sqrt(a20);
  if (res[0]!=0) res[0][4]=a20;
  if (res[1]!=0) res[1][0]=a19;
  a21=arg[1]? arg[1][0] : 0;
  a22=(a2*a21);
  a22=(a1*a22);
  a23=(a3+a3);
  a24=(a23*a21);
  a24=(a5*a24);
  a24=(a4*a24);
  a22=(a22+a24);
  a24=(a8*a21);
  a25=(a3+a3);
  a26=(a25*a21);
  a26=(a3*a26);
  a24=(a24+a26);
  a24=(a7*a24);
  a24=(a6*a24);
  a22=(a22+a24);
  a11=(a11+a11);
  a24=(a3+a3);
  a26=(a24*a21);
  a26=(a11*a26);
  a26=(a10*a26);
  a26=(a9*a26);
  a22=(a22+a26);
  if (res[1]!=0) res[1][1]=a22;
  a26=(a13*a21);
  a26=(a1*a26);
  a27=(a3+a3);
  a28=(a27*a21);
  a28=(a14*a28);
  a28=(a4*a28);
  a26=(a26+a28);
  a28=(a16*a21);
  a29=(a3+a3);
  a30=(a29*a21);
  a30=(a3*a30);
  a28=(a28+a30);
  a28=(a15*a28);
  a28=(a6*a28);
  a26=(a26+a28);
  a18=(a18+a18);
  a28=(a3+a3);
  a21=(a28*a21);
  a21=(a18*a21);
  a21=(a17*a21);
  a21=(a9*a21);
  a26=(a26+a21);
  if (res[1]!=0) res[1][2]=a26;
  if (res[1]!=0) res[1][3]=a19;
  a0=(a0+a0);
  a22=(a0*a22);
  a12=(a12+a12);
  a26=(a12*a26);
  a22=(a22+a26);
  a20=(a20+a20);
  a22=(a22/a20);
  if (res[1]!=0) res[1][4]=a22;
  if (res[1]!=0) res[1][5]=a19;
  a22=arg[1]? arg[1][5] : 0;
  a26=(a2*a22);
  a26=(a1*a26);
  a21=(a23*a22);
  a21=(a5*a21);
  a21=(a4*a21);
  a26=(a26+a21);
  a21=(a8*a22);
  a30=(a25*a22);
  a30=(a3*a30);
  a21=(a21+a30);
  a21=(a7*a21);
  a21=(a6*a21);
  a26=(a26+a21);
  a21=(a24*a22);
  a21=(a11*a21);
  a21=(a10*a21);
  a21=(a9*a21);
  a26=(a26+a21);
  if (res[1]!=0) res[1][6]=a26;
  a21=(a13*a22);
  a21=(a1*a21);
  a30=(a27*a22);
  a30=(a14*a30);
  a30=(a4*a30);
  a21=(a21+a30);
  a30=(a16*a22);
  a31=(a29*a22);
  a31=(a3*a31);
  a30=(a30+a31);
  a30=(a15*a30);
  a30=(a6*a30);
  a21=(a21+a30);
  a22=(a28*a22);
  a22=(a18*a22);
  a22=(a17*a22);
  a22=(a9*a22);
  a21=(a21+a22);
  if (res[1]!=0) res[1][7]=a21;
  if (res[1]!=0) res[1][8]=a19;
  a26=(a0*a26);
  a21=(a12*a21);
  a26=(a26+a21);
  a26=(a26/a20);
  if (res[1]!=0) res[1][9]=a26;
  if (res[1]!=0) res[1][10]=a19;
  a26=arg[1]? arg[1][10] : 0;
  a21=(a2*a26);
  a21=(a1*a21);
  a22=(a23*a26);
  a22=(a5*a22);
  a22=(a4*a22);
  a21=(a21+a22);
  a22=(a8*a26);
  a30=(a25*a26);
  a30=(a3*a30);
  a22=(a22+a30);
  a22=(a7*a22);
  a22=(a6*a22);
  a21=(a21+a22);
  a22=(a24*a26);
  a22=(a11*a22);
  a22=(a10*a22);
  a22=(a9*a22);
  a21=(a21+a22);
  if (res[1]!=0) res[1][11]=a21;
  a22=(a13*a26);
  a22=(a1*a22);
  a30=(a27*a26);
  a30=(a14*a30);
  a30=(a4*a30);
  a22=(a22+a30);
  a30=(a16*a26);
  a31=(a29*a26);
  a31=(a3*a31);
  a30=(a30+a31);
  a30=(a15*a30);
  a30=(a6*a30);
  a22=(a22+a30);
  a26=(a28*a26);
  a26=(a18*a26);
  a26=(a17*a26);
  a26=(a9*a26);
  a22=(a22+a26);
  if (res[1]!=0) res[1][12]=a22;
  if (res[1]!=0) res[1][13]=a19;
  a21=(a0*a21);
  a22=(a12*a22);
  a21=(a21+a22);
  a21=(a21/a20);
  if (res[1]!=0) res[1][14]=a21;
  if (res[1]!=0) res[1][15]=a19;
  a21=arg[1]? arg[1][15] : 0;
  a22=(a2*a21);
  a22=(a1*a22);
  a26=(a23*a21);
  a26=(a5*a26);
  a26=(a4*a26);
  a22=(a22+a26);
  a26=(a8*a21);
  a30=(a25*a21);
  a30=(a3*a30);
  a26=(a26+a30);
  a26=(a7*a26);
  a26=(a6*a26);
  a22=(a22+a26);
  a26=(a24*a21);
  a26=(a11*a26);
  a26=(a10*a26);
  a26=(a9*a26);
  a22=(a22+a26);
  if (res[1]!=0) res[1][16]=a22;
  a26=(a13*a21);
  a26=(a1*a26);
  a30=(a27*a21);
  a30=(a14*a30);
  a30=(a4*a30);
  a26=(a26+a30);
  a30=(a16*a21);
  a31=(a29*a21);
  a31=(a3*a31);
  a30=(a30+a31);
  a30=(a15*a30);
  a30=(a6*a30);
  a26=(a26+a30);
  a21=(a28*a21);
  a21=(a18*a21);
  a21=(a17*a21);
  a21=(a9*a21);
  a26=(a26+a21);
  if (res[1]!=0) res[1][17]=a26;
  if (res[1]!=0) res[1][18]=a19;
  a22=(a0*a22);
  a26=(a12*a26);
  a22=(a22+a26);
  a22=(a22/a20);
  if (res[1]!=0) res[1][19]=a22;
  if (res[1]!=0) res[1][20]=a19;
  a22=arg[1]? arg[1][20] : 0;
  a2=(a2*a22);
  a2=(a1*a2);
  a23=(a23*a22);
  a5=(a5*a23);
  a5=(a4*a5);
  a2=(a2+a5);
  a8=(a8*a22);
  a25=(a25*a22);
  a25=(a3*a25);
  a8=(a8+a25);
  a7=(a7*a8);
  a7=(a6*a7);
  a2=(a2+a7);
  a24=(a24*a22);
  a11=(a11*a24);
  a10=(a10*a11);
  a10=(a9*a10);
  a2=(a2+a10);
  if (res[1]!=0) res[1][21]=a2;
  a13=(a13*a22);
  a1=(a1*a13);
  a27=(a27*a22);
  a14=(a14*a27);
  a4=(a4*a14);
  a1=(a1+a4);
  a16=(a16*a22);
  a29=(a29*a22);
  a3=(a3*a29);
  a16=(a16+a3);
  a15=(a15*a16);
  a6=(a6*a15);
  a1=(a1+a6);
  a28=(a28*a22);
  a18=(a18*a28);
  a17=(a17*a18);
  a9=(a9*a17);
  a1=(a1+a9);
  if (res[1]!=0) res[1][22]=a1;
  if (res[1]!=0) res[1][23]=a19;
  a0=(a0*a2);
  a12=(a12*a1);
  a0=(a0+a12);
  a0=(a0/a20);
  if (res[1]!=0) res[1][24]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int car0_len_integrate_1_expl_vde_forw(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int car0_len_integrate_1_expl_vde_forw_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int car0_len_integrate_1_expl_vde_forw_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void car0_len_integrate_1_expl_vde_forw_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int car0_len_integrate_1_expl_vde_forw_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void car0_len_integrate_1_expl_vde_forw_release(int mem) {
}

CASADI_SYMBOL_EXPORT void car0_len_integrate_1_expl_vde_forw_incref(void) {
}

CASADI_SYMBOL_EXPORT void car0_len_integrate_1_expl_vde_forw_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int car0_len_integrate_1_expl_vde_forw_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int car0_len_integrate_1_expl_vde_forw_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real car0_len_integrate_1_expl_vde_forw_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car0_len_integrate_1_expl_vde_forw_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car0_len_integrate_1_expl_vde_forw_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* car0_len_integrate_1_expl_vde_forw_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    case 4: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* car0_len_integrate_1_expl_vde_forw_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int car0_len_integrate_1_expl_vde_forw_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
