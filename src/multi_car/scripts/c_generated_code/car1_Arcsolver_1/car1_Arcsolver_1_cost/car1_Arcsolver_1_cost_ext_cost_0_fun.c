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
  #define CASADI_PREFIX(ID) car1_Arcsolver_1_cost_ext_cost_0_fun_ ## ID
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

/* car1_Arcsolver_1_cost_ext_cost_0_fun:(i0[60],i1[],i2[],i3[])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3, a4, a5, a6, a7;
  a0=2.3999999999999995e+00;
  a1=arg[0]? arg[0][2] : 0;
  a2=(a0*a1);
  a3=2.1599999999999988e+00;
  a4=arg[0]? arg[0][3] : 0;
  a5=(a3*a4);
  a2=(a2+a5);
  a2=(a2*a1);
  a1=(a3*a1);
  a5=2.5919999999999983e+00;
  a6=(a5*a4);
  a1=(a1+a6);
  a1=(a1*a4);
  a2=(a2+a1);
  a1=arg[0]? arg[0][6] : 0;
  a4=(a0*a1);
  a6=arg[0]? arg[0][7] : 0;
  a7=(a3*a6);
  a4=(a4+a7);
  a4=(a4*a1);
  a2=(a2+a4);
  a1=(a3*a1);
  a4=(a5*a6);
  a1=(a1+a4);
  a1=(a1*a6);
  a2=(a2+a1);
  a1=arg[0]? arg[0][10] : 0;
  a6=(a0*a1);
  a4=arg[0]? arg[0][11] : 0;
  a7=(a3*a4);
  a6=(a6+a7);
  a6=(a6*a1);
  a2=(a2+a6);
  a1=(a3*a1);
  a6=(a5*a4);
  a1=(a1+a6);
  a1=(a1*a4);
  a2=(a2+a1);
  a1=arg[0]? arg[0][14] : 0;
  a4=(a0*a1);
  a6=arg[0]? arg[0][15] : 0;
  a7=(a3*a6);
  a4=(a4+a7);
  a4=(a4*a1);
  a2=(a2+a4);
  a1=(a3*a1);
  a4=(a5*a6);
  a1=(a1+a4);
  a1=(a1*a6);
  a2=(a2+a1);
  a1=arg[0]? arg[0][18] : 0;
  a6=(a0*a1);
  a4=arg[0]? arg[0][19] : 0;
  a7=(a3*a4);
  a6=(a6+a7);
  a6=(a6*a1);
  a2=(a2+a6);
  a1=(a3*a1);
  a6=(a5*a4);
  a1=(a1+a6);
  a1=(a1*a4);
  a2=(a2+a1);
  a1=arg[0]? arg[0][22] : 0;
  a4=(a0*a1);
  a6=arg[0]? arg[0][23] : 0;
  a7=(a3*a6);
  a4=(a4+a7);
  a4=(a4*a1);
  a2=(a2+a4);
  a1=(a3*a1);
  a4=(a5*a6);
  a1=(a1+a4);
  a1=(a1*a6);
  a2=(a2+a1);
  a1=arg[0]? arg[0][26] : 0;
  a6=(a0*a1);
  a4=arg[0]? arg[0][27] : 0;
  a7=(a3*a4);
  a6=(a6+a7);
  a6=(a6*a1);
  a2=(a2+a6);
  a1=(a3*a1);
  a6=(a5*a4);
  a1=(a1+a6);
  a1=(a1*a4);
  a2=(a2+a1);
  a1=arg[0]? arg[0][30] : 0;
  a4=(a0*a1);
  a6=arg[0]? arg[0][31] : 0;
  a7=(a3*a6);
  a4=(a4+a7);
  a4=(a4*a1);
  a2=(a2+a4);
  a1=(a3*a1);
  a4=(a5*a6);
  a1=(a1+a4);
  a1=(a1*a6);
  a2=(a2+a1);
  a1=arg[0]? arg[0][34] : 0;
  a6=(a0*a1);
  a4=arg[0]? arg[0][35] : 0;
  a7=(a3*a4);
  a6=(a6+a7);
  a6=(a6*a1);
  a2=(a2+a6);
  a1=(a3*a1);
  a6=(a5*a4);
  a1=(a1+a6);
  a1=(a1*a4);
  a2=(a2+a1);
  a1=arg[0]? arg[0][38] : 0;
  a4=(a0*a1);
  a6=arg[0]? arg[0][39] : 0;
  a7=(a3*a6);
  a4=(a4+a7);
  a4=(a4*a1);
  a2=(a2+a4);
  a1=(a3*a1);
  a4=(a5*a6);
  a1=(a1+a4);
  a1=(a1*a6);
  a2=(a2+a1);
  a1=arg[0]? arg[0][42] : 0;
  a6=(a0*a1);
  a4=arg[0]? arg[0][43] : 0;
  a7=(a3*a4);
  a6=(a6+a7);
  a6=(a6*a1);
  a2=(a2+a6);
  a1=(a3*a1);
  a6=(a5*a4);
  a1=(a1+a6);
  a1=(a1*a4);
  a2=(a2+a1);
  a1=arg[0]? arg[0][46] : 0;
  a4=(a0*a1);
  a6=arg[0]? arg[0][47] : 0;
  a7=(a3*a6);
  a4=(a4+a7);
  a4=(a4*a1);
  a2=(a2+a4);
  a1=(a3*a1);
  a4=(a5*a6);
  a1=(a1+a4);
  a1=(a1*a6);
  a2=(a2+a1);
  a1=arg[0]? arg[0][50] : 0;
  a6=(a0*a1);
  a4=arg[0]? arg[0][51] : 0;
  a7=(a3*a4);
  a6=(a6+a7);
  a6=(a6*a1);
  a2=(a2+a6);
  a1=(a3*a1);
  a6=(a5*a4);
  a1=(a1+a6);
  a1=(a1*a4);
  a2=(a2+a1);
  a1=arg[0]? arg[0][54] : 0;
  a4=(a0*a1);
  a6=arg[0]? arg[0][55] : 0;
  a7=(a3*a6);
  a4=(a4+a7);
  a4=(a4*a1);
  a2=(a2+a4);
  a1=(a3*a1);
  a4=(a5*a6);
  a1=(a1+a4);
  a1=(a1*a6);
  a2=(a2+a1);
  a1=arg[0]? arg[0][58] : 0;
  a0=(a0*a1);
  a6=arg[0]? arg[0][59] : 0;
  a4=(a3*a6);
  a0=(a0+a4);
  a0=(a0*a1);
  a2=(a2+a0);
  a3=(a3*a1);
  a5=(a5*a6);
  a3=(a3+a5);
  a3=(a3*a6);
  a2=(a2+a3);
  if (res[0]!=0) res[0][0]=a2;
  return 0;
}

CASADI_SYMBOL_EXPORT int car1_Arcsolver_1_cost_ext_cost_0_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int car1_Arcsolver_1_cost_ext_cost_0_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int car1_Arcsolver_1_cost_ext_cost_0_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void car1_Arcsolver_1_cost_ext_cost_0_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int car1_Arcsolver_1_cost_ext_cost_0_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void car1_Arcsolver_1_cost_ext_cost_0_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void car1_Arcsolver_1_cost_ext_cost_0_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void car1_Arcsolver_1_cost_ext_cost_0_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int car1_Arcsolver_1_cost_ext_cost_0_fun_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int car1_Arcsolver_1_cost_ext_cost_0_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real car1_Arcsolver_1_cost_ext_cost_0_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car1_Arcsolver_1_cost_ext_cost_0_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car1_Arcsolver_1_cost_ext_cost_0_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* car1_Arcsolver_1_cost_ext_cost_0_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    case 3: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* car1_Arcsolver_1_cost_ext_cost_0_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int car1_Arcsolver_1_cost_ext_cost_0_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
