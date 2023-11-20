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
  #define CASADI_PREFIX(ID) car0_len_integrate_1_expl_vde_adj_ ## ID
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
static const casadi_int casadi_s1[3] = {0, 0, 0};

/* car0_len_integrate_1_expl_vde_adj:(i0[5],i1[5],i2[],i3[])->(o0[5]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][0] : 0;
  a1=(a0+a0);
  a2=casadi_sq(a0);
  a3=(a2+a2);
  a4=-3.1029974265141000e-18;
  a5=5.;
  a6=-7.5000000000000000e-01;
  a7=2.;
  a8=-3.7499999999999994e-01;
  a9=(a8*a0);
  a9=(a7*a9);
  a6=(a6+a9);
  a9=3.;
  a10=5.7720803631475752e-17;
  a11=casadi_sq(a0);
  a11=(a10*a11);
  a11=(a9*a11);
  a6=(a6+a11);
  a11=4.;
  a12=3.5363967611591004e-18;
  a13=casadi_sq(a0);
  a14=(a0*a13);
  a14=(a12*a14);
  a14=(a11*a14);
  a6=(a6+a14);
  a2=casadi_sq(a2);
  a2=(a4*a2);
  a2=(a5*a2);
  a6=(a6+a2);
  a2=(a6+a6);
  a14=arg[1]? arg[1][4] : 0;
  a15=7.4999999961205677e-01;
  a16=-3.7499999980601434e-01;
  a17=(a16*a0);
  a17=(a7*a17);
  a15=(a15+a17);
  a17=-7.1512208326494833e-15;
  a18=casadi_sq(a0);
  a18=(a17*a18);
  a18=(a9*a18);
  a15=(a15+a18);
  a18=-1.0337972228114721e-15;
  a19=casadi_sq(a0);
  a20=(a0*a19);
  a20=(a18*a20);
  a20=(a11*a20);
  a15=(a15+a20);
  a20=5.5129103862741875e-16;
  a21=casadi_sq(a0);
  a22=casadi_sq(a21);
  a22=(a20*a22);
  a22=(a5*a22);
  a15=(a15+a22);
  a22=casadi_sq(a15);
  a6=casadi_sq(a6);
  a22=(a22+a6);
  a22=sqrt(a22);
  a22=(a22+a22);
  a14=(a14/a22);
  a2=(a2*a14);
  a22=arg[1]? arg[1][2] : 0;
  a2=(a2+a22);
  a22=(a5*a2);
  a4=(a4*a22);
  a3=(a3*a4);
  a1=(a1*a3);
  a3=(a11*a2);
  a12=(a12*a3);
  a13=(a13*a12);
  a1=(a1+a13);
  a13=(a0+a0);
  a12=(a0*a12);
  a13=(a13*a12);
  a1=(a1+a13);
  a13=(a0+a0);
  a12=(a9*a2);
  a10=(a10*a12);
  a13=(a13*a10);
  a1=(a1+a13);
  a2=(a7*a2);
  a8=(a8*a2);
  a1=(a1+a8);
  a8=(a0+a0);
  a21=(a21+a21);
  a15=(a15+a15);
  a15=(a15*a14);
  a14=arg[1]? arg[1][1] : 0;
  a15=(a15+a14);
  a5=(a5*a15);
  a20=(a20*a5);
  a21=(a21*a20);
  a8=(a8*a21);
  a1=(a1+a8);
  a11=(a11*a15);
  a18=(a18*a11);
  a19=(a19*a18);
  a1=(a1+a19);
  a19=(a0+a0);
  a18=(a0*a18);
  a19=(a19*a18);
  a1=(a1+a19);
  a0=(a0+a0);
  a9=(a9*a15);
  a17=(a17*a9);
  a0=(a0*a17);
  a1=(a1+a0);
  a7=(a7*a15);
  a16=(a16*a7);
  a1=(a1+a16);
  if (res[0]!=0) res[0][0]=a1;
  a1=0.;
  if (res[0]!=0) res[0][1]=a1;
  if (res[0]!=0) res[0][2]=a1;
  if (res[0]!=0) res[0][3]=a1;
  if (res[0]!=0) res[0][4]=a1;
  return 0;
}

CASADI_SYMBOL_EXPORT int car0_len_integrate_1_expl_vde_adj(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int car0_len_integrate_1_expl_vde_adj_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int car0_len_integrate_1_expl_vde_adj_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void car0_len_integrate_1_expl_vde_adj_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int car0_len_integrate_1_expl_vde_adj_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void car0_len_integrate_1_expl_vde_adj_release(int mem) {
}

CASADI_SYMBOL_EXPORT void car0_len_integrate_1_expl_vde_adj_incref(void) {
}

CASADI_SYMBOL_EXPORT void car0_len_integrate_1_expl_vde_adj_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int car0_len_integrate_1_expl_vde_adj_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int car0_len_integrate_1_expl_vde_adj_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real car0_len_integrate_1_expl_vde_adj_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car0_len_integrate_1_expl_vde_adj_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car0_len_integrate_1_expl_vde_adj_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* car0_len_integrate_1_expl_vde_adj_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* car0_len_integrate_1_expl_vde_adj_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int car0_len_integrate_1_expl_vde_adj_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
