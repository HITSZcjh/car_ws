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
  #define CASADI_PREFIX(ID) car0_len_integrate_0_expl_vde_forw_ ## ID
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
#define casadi_sign CASADI_PREFIX(sign)
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

casadi_real casadi_sign(casadi_real x) { return x<0 ? -1 : x>0 ? 1 : x;}

static const casadi_int casadi_s0[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s1[33] = {5, 5, 0, 5, 10, 15, 20, 25, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4};
static const casadi_int casadi_s2[3] = {5, 0, 0};
static const casadi_int casadi_s3[3] = {0, 0, 0};

/* car0_len_integrate_0_expl_vde_forw:(i0[5],i1[5x5],i2[5x0],i3[],i4[])->(o0[5],o1[5x5],o2[5x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=1.;
  if (res[0]!=0) res[0][0]=a0;
  a0=1.5000000000000004e+00;
  a1=2.;
  a2=-8.1017633138959106e-16;
  a3=arg[0]? arg[0][0] : 0;
  a4=(a2*a3);
  a4=(a1*a4);
  a0=(a0+a4);
  a4=3.;
  a5=3.5193474490154475e-16;
  a6=casadi_sq(a3);
  a6=(a5*a6);
  a6=(a4*a6);
  a0=(a0+a6);
  a6=4.;
  a7=-6.4297343411790249e-17;
  a8=casadi_sq(a3);
  a9=(a3*a8);
  a9=(a7*a9);
  a9=(a6*a9);
  a0=(a0+a9);
  a9=5.;
  a10=4.3710529608751765e-18;
  a11=casadi_sq(a3);
  a12=casadi_sq(a11);
  a12=(a10*a12);
  a12=(a9*a12);
  a0=(a0+a12);
  if (res[0]!=0) res[0][1]=a0;
  a12=0.;
  if (res[0]!=0) res[0][2]=a12;
  if (res[0]!=0) res[0][3]=a12;
  a13=fabs(a0);
  if (res[0]!=0) res[0][4]=a13;
  if (res[1]!=0) res[1][0]=a12;
  a13=arg[1]? arg[1][0] : 0;
  a14=(a2*a13);
  a14=(a1*a14);
  a15=(a3+a3);
  a16=(a15*a13);
  a16=(a5*a16);
  a16=(a4*a16);
  a14=(a14+a16);
  a16=(a8*a13);
  a17=(a3+a3);
  a18=(a17*a13);
  a18=(a3*a18);
  a16=(a16+a18);
  a16=(a7*a16);
  a16=(a6*a16);
  a14=(a14+a16);
  a11=(a11+a11);
  a16=(a3+a3);
  a13=(a16*a13);
  a13=(a11*a13);
  a13=(a10*a13);
  a13=(a9*a13);
  a14=(a14+a13);
  if (res[1]!=0) res[1][1]=a14;
  if (res[1]!=0) res[1][2]=a12;
  if (res[1]!=0) res[1][3]=a12;
  a0=casadi_sign(a0);
  a14=(a0*a14);
  if (res[1]!=0) res[1][4]=a14;
  if (res[1]!=0) res[1][5]=a12;
  a14=arg[1]? arg[1][5] : 0;
  a13=(a2*a14);
  a13=(a1*a13);
  a18=(a15*a14);
  a18=(a5*a18);
  a18=(a4*a18);
  a13=(a13+a18);
  a18=(a8*a14);
  a19=(a17*a14);
  a19=(a3*a19);
  a18=(a18+a19);
  a18=(a7*a18);
  a18=(a6*a18);
  a13=(a13+a18);
  a14=(a16*a14);
  a14=(a11*a14);
  a14=(a10*a14);
  a14=(a9*a14);
  a13=(a13+a14);
  if (res[1]!=0) res[1][6]=a13;
  if (res[1]!=0) res[1][7]=a12;
  if (res[1]!=0) res[1][8]=a12;
  a13=(a0*a13);
  if (res[1]!=0) res[1][9]=a13;
  if (res[1]!=0) res[1][10]=a12;
  a13=arg[1]? arg[1][10] : 0;
  a14=(a2*a13);
  a14=(a1*a14);
  a18=(a15*a13);
  a18=(a5*a18);
  a18=(a4*a18);
  a14=(a14+a18);
  a18=(a8*a13);
  a19=(a17*a13);
  a19=(a3*a19);
  a18=(a18+a19);
  a18=(a7*a18);
  a18=(a6*a18);
  a14=(a14+a18);
  a13=(a16*a13);
  a13=(a11*a13);
  a13=(a10*a13);
  a13=(a9*a13);
  a14=(a14+a13);
  if (res[1]!=0) res[1][11]=a14;
  if (res[1]!=0) res[1][12]=a12;
  if (res[1]!=0) res[1][13]=a12;
  a14=(a0*a14);
  if (res[1]!=0) res[1][14]=a14;
  if (res[1]!=0) res[1][15]=a12;
  a14=arg[1]? arg[1][15] : 0;
  a13=(a2*a14);
  a13=(a1*a13);
  a18=(a15*a14);
  a18=(a5*a18);
  a18=(a4*a18);
  a13=(a13+a18);
  a18=(a8*a14);
  a19=(a17*a14);
  a19=(a3*a19);
  a18=(a18+a19);
  a18=(a7*a18);
  a18=(a6*a18);
  a13=(a13+a18);
  a14=(a16*a14);
  a14=(a11*a14);
  a14=(a10*a14);
  a14=(a9*a14);
  a13=(a13+a14);
  if (res[1]!=0) res[1][16]=a13;
  if (res[1]!=0) res[1][17]=a12;
  if (res[1]!=0) res[1][18]=a12;
  a13=(a0*a13);
  if (res[1]!=0) res[1][19]=a13;
  if (res[1]!=0) res[1][20]=a12;
  a13=arg[1]? arg[1][20] : 0;
  a2=(a2*a13);
  a1=(a1*a2);
  a15=(a15*a13);
  a5=(a5*a15);
  a4=(a4*a5);
  a1=(a1+a4);
  a8=(a8*a13);
  a17=(a17*a13);
  a3=(a3*a17);
  a8=(a8+a3);
  a7=(a7*a8);
  a6=(a6*a7);
  a1=(a1+a6);
  a16=(a16*a13);
  a11=(a11*a16);
  a10=(a10*a11);
  a9=(a9*a10);
  a1=(a1+a9);
  if (res[1]!=0) res[1][21]=a1;
  if (res[1]!=0) res[1][22]=a12;
  if (res[1]!=0) res[1][23]=a12;
  a0=(a0*a1);
  if (res[1]!=0) res[1][24]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int car0_len_integrate_0_expl_vde_forw(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int car0_len_integrate_0_expl_vde_forw_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int car0_len_integrate_0_expl_vde_forw_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void car0_len_integrate_0_expl_vde_forw_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int car0_len_integrate_0_expl_vde_forw_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void car0_len_integrate_0_expl_vde_forw_release(int mem) {
}

CASADI_SYMBOL_EXPORT void car0_len_integrate_0_expl_vde_forw_incref(void) {
}

CASADI_SYMBOL_EXPORT void car0_len_integrate_0_expl_vde_forw_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int car0_len_integrate_0_expl_vde_forw_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int car0_len_integrate_0_expl_vde_forw_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real car0_len_integrate_0_expl_vde_forw_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car0_len_integrate_0_expl_vde_forw_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car0_len_integrate_0_expl_vde_forw_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* car0_len_integrate_0_expl_vde_forw_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    case 4: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* car0_len_integrate_0_expl_vde_forw_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int car0_len_integrate_0_expl_vde_forw_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
