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
  #define CASADI_PREFIX(ID) car1_Arcsolver_1_dyn_disc_phi_fun_ ## ID
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

/* car1_Arcsolver_1_dyn_disc_phi_fun:(i0[60],i1[],i2[])->(o0[60]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0;
  a0=arg[0]? arg[0][0] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[0]? arg[0][1] : 0;
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[0]? arg[0][2] : 0;
  if (res[0]!=0) res[0][2]=a0;
  a0=arg[0]? arg[0][3] : 0;
  if (res[0]!=0) res[0][3]=a0;
  a0=arg[0]? arg[0][4] : 0;
  if (res[0]!=0) res[0][4]=a0;
  a0=arg[0]? arg[0][5] : 0;
  if (res[0]!=0) res[0][5]=a0;
  a0=arg[0]? arg[0][6] : 0;
  if (res[0]!=0) res[0][6]=a0;
  a0=arg[0]? arg[0][7] : 0;
  if (res[0]!=0) res[0][7]=a0;
  a0=arg[0]? arg[0][8] : 0;
  if (res[0]!=0) res[0][8]=a0;
  a0=arg[0]? arg[0][9] : 0;
  if (res[0]!=0) res[0][9]=a0;
  a0=arg[0]? arg[0][10] : 0;
  if (res[0]!=0) res[0][10]=a0;
  a0=arg[0]? arg[0][11] : 0;
  if (res[0]!=0) res[0][11]=a0;
  a0=arg[0]? arg[0][12] : 0;
  if (res[0]!=0) res[0][12]=a0;
  a0=arg[0]? arg[0][13] : 0;
  if (res[0]!=0) res[0][13]=a0;
  a0=arg[0]? arg[0][14] : 0;
  if (res[0]!=0) res[0][14]=a0;
  a0=arg[0]? arg[0][15] : 0;
  if (res[0]!=0) res[0][15]=a0;
  a0=arg[0]? arg[0][16] : 0;
  if (res[0]!=0) res[0][16]=a0;
  a0=arg[0]? arg[0][17] : 0;
  if (res[0]!=0) res[0][17]=a0;
  a0=arg[0]? arg[0][18] : 0;
  if (res[0]!=0) res[0][18]=a0;
  a0=arg[0]? arg[0][19] : 0;
  if (res[0]!=0) res[0][19]=a0;
  a0=arg[0]? arg[0][20] : 0;
  if (res[0]!=0) res[0][20]=a0;
  a0=arg[0]? arg[0][21] : 0;
  if (res[0]!=0) res[0][21]=a0;
  a0=arg[0]? arg[0][22] : 0;
  if (res[0]!=0) res[0][22]=a0;
  a0=arg[0]? arg[0][23] : 0;
  if (res[0]!=0) res[0][23]=a0;
  a0=arg[0]? arg[0][24] : 0;
  if (res[0]!=0) res[0][24]=a0;
  a0=arg[0]? arg[0][25] : 0;
  if (res[0]!=0) res[0][25]=a0;
  a0=arg[0]? arg[0][26] : 0;
  if (res[0]!=0) res[0][26]=a0;
  a0=arg[0]? arg[0][27] : 0;
  if (res[0]!=0) res[0][27]=a0;
  a0=arg[0]? arg[0][28] : 0;
  if (res[0]!=0) res[0][28]=a0;
  a0=arg[0]? arg[0][29] : 0;
  if (res[0]!=0) res[0][29]=a0;
  a0=arg[0]? arg[0][30] : 0;
  if (res[0]!=0) res[0][30]=a0;
  a0=arg[0]? arg[0][31] : 0;
  if (res[0]!=0) res[0][31]=a0;
  a0=arg[0]? arg[0][32] : 0;
  if (res[0]!=0) res[0][32]=a0;
  a0=arg[0]? arg[0][33] : 0;
  if (res[0]!=0) res[0][33]=a0;
  a0=arg[0]? arg[0][34] : 0;
  if (res[0]!=0) res[0][34]=a0;
  a0=arg[0]? arg[0][35] : 0;
  if (res[0]!=0) res[0][35]=a0;
  a0=arg[0]? arg[0][36] : 0;
  if (res[0]!=0) res[0][36]=a0;
  a0=arg[0]? arg[0][37] : 0;
  if (res[0]!=0) res[0][37]=a0;
  a0=arg[0]? arg[0][38] : 0;
  if (res[0]!=0) res[0][38]=a0;
  a0=arg[0]? arg[0][39] : 0;
  if (res[0]!=0) res[0][39]=a0;
  a0=arg[0]? arg[0][40] : 0;
  if (res[0]!=0) res[0][40]=a0;
  a0=arg[0]? arg[0][41] : 0;
  if (res[0]!=0) res[0][41]=a0;
  a0=arg[0]? arg[0][42] : 0;
  if (res[0]!=0) res[0][42]=a0;
  a0=arg[0]? arg[0][43] : 0;
  if (res[0]!=0) res[0][43]=a0;
  a0=arg[0]? arg[0][44] : 0;
  if (res[0]!=0) res[0][44]=a0;
  a0=arg[0]? arg[0][45] : 0;
  if (res[0]!=0) res[0][45]=a0;
  a0=arg[0]? arg[0][46] : 0;
  if (res[0]!=0) res[0][46]=a0;
  a0=arg[0]? arg[0][47] : 0;
  if (res[0]!=0) res[0][47]=a0;
  a0=arg[0]? arg[0][48] : 0;
  if (res[0]!=0) res[0][48]=a0;
  a0=arg[0]? arg[0][49] : 0;
  if (res[0]!=0) res[0][49]=a0;
  a0=arg[0]? arg[0][50] : 0;
  if (res[0]!=0) res[0][50]=a0;
  a0=arg[0]? arg[0][51] : 0;
  if (res[0]!=0) res[0][51]=a0;
  a0=arg[0]? arg[0][52] : 0;
  if (res[0]!=0) res[0][52]=a0;
  a0=arg[0]? arg[0][53] : 0;
  if (res[0]!=0) res[0][53]=a0;
  a0=arg[0]? arg[0][54] : 0;
  if (res[0]!=0) res[0][54]=a0;
  a0=arg[0]? arg[0][55] : 0;
  if (res[0]!=0) res[0][55]=a0;
  a0=arg[0]? arg[0][56] : 0;
  if (res[0]!=0) res[0][56]=a0;
  a0=arg[0]? arg[0][57] : 0;
  if (res[0]!=0) res[0][57]=a0;
  a0=arg[0]? arg[0][58] : 0;
  if (res[0]!=0) res[0][58]=a0;
  a0=arg[0]? arg[0][59] : 0;
  if (res[0]!=0) res[0][59]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int car1_Arcsolver_1_dyn_disc_phi_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int car1_Arcsolver_1_dyn_disc_phi_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int car1_Arcsolver_1_dyn_disc_phi_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void car1_Arcsolver_1_dyn_disc_phi_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int car1_Arcsolver_1_dyn_disc_phi_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void car1_Arcsolver_1_dyn_disc_phi_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void car1_Arcsolver_1_dyn_disc_phi_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void car1_Arcsolver_1_dyn_disc_phi_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int car1_Arcsolver_1_dyn_disc_phi_fun_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int car1_Arcsolver_1_dyn_disc_phi_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real car1_Arcsolver_1_dyn_disc_phi_fun_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car1_Arcsolver_1_dyn_disc_phi_fun_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car1_Arcsolver_1_dyn_disc_phi_fun_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* car1_Arcsolver_1_dyn_disc_phi_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* car1_Arcsolver_1_dyn_disc_phi_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int car1_Arcsolver_1_dyn_disc_phi_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
