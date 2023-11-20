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
  #define CASADI_PREFIX(ID) circle_car_expl_vde_forw_ ## ID
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

static const casadi_int casadi_s0[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s1[33] = {5, 5, 0, 5, 10, 15, 20, 25, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4};
static const casadi_int casadi_s2[15] = {5, 2, 0, 5, 10, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4};
static const casadi_int casadi_s3[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s4[3] = {0, 0, 0};

/* circle_car_expl_vde_forw:(i0[5],i1[5x5],i2[5x2],i3[2],i4[])->(o0[5],o1[5x5],o2[5x2]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a2, a3, a4, a5, a6, a7, a8, a9;
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
  a5=2.5000000000000000e+00;
  a6=arg[3]? arg[3][0] : 0;
  a6=(a6-a0);
  a6=(a5*a6);
  if (res[0]!=0) res[0][3]=a6;
  a6=arg[3]? arg[3][1] : 0;
  a6=(a6-a4);
  a6=(a5*a6);
  if (res[0]!=0) res[0][4]=a6;
  a6=arg[1]? arg[1][3] : 0;
  a4=(a2*a6);
  a7=sin(a1);
  a8=arg[1]? arg[1][2] : 0;
  a9=(a7*a8);
  a9=(a0*a9);
  a4=(a4-a9);
  if (res[1]!=0) res[1][0]=a4;
  a4=(a3*a6);
  a9=cos(a1);
  a8=(a9*a8);
  a8=(a0*a8);
  a4=(a4+a8);
  if (res[1]!=0) res[1][1]=a4;
  a4=arg[1]? arg[1][4] : 0;
  if (res[1]!=0) res[1][2]=a4;
  a6=(a5*a6);
  a6=(-a6);
  if (res[1]!=0) res[1][3]=a6;
  a4=(a5*a4);
  a4=(-a4);
  if (res[1]!=0) res[1][4]=a4;
  a4=arg[1]? arg[1][8] : 0;
  a6=(a2*a4);
  a8=arg[1]? arg[1][7] : 0;
  a10=(a7*a8);
  a10=(a0*a10);
  a6=(a6-a10);
  if (res[1]!=0) res[1][5]=a6;
  a6=(a3*a4);
  a8=(a9*a8);
  a8=(a0*a8);
  a6=(a6+a8);
  if (res[1]!=0) res[1][6]=a6;
  a6=arg[1]? arg[1][9] : 0;
  if (res[1]!=0) res[1][7]=a6;
  a4=(a5*a4);
  a4=(-a4);
  if (res[1]!=0) res[1][8]=a4;
  a6=(a5*a6);
  a6=(-a6);
  if (res[1]!=0) res[1][9]=a6;
  a6=arg[1]? arg[1][13] : 0;
  a4=(a2*a6);
  a8=arg[1]? arg[1][12] : 0;
  a10=(a7*a8);
  a10=(a0*a10);
  a4=(a4-a10);
  if (res[1]!=0) res[1][10]=a4;
  a4=(a3*a6);
  a8=(a9*a8);
  a8=(a0*a8);
  a4=(a4+a8);
  if (res[1]!=0) res[1][11]=a4;
  a4=arg[1]? arg[1][14] : 0;
  if (res[1]!=0) res[1][12]=a4;
  a6=(a5*a6);
  a6=(-a6);
  if (res[1]!=0) res[1][13]=a6;
  a4=(a5*a4);
  a4=(-a4);
  if (res[1]!=0) res[1][14]=a4;
  a4=arg[1]? arg[1][18] : 0;
  a6=(a2*a4);
  a8=arg[1]? arg[1][17] : 0;
  a10=(a7*a8);
  a10=(a0*a10);
  a6=(a6-a10);
  if (res[1]!=0) res[1][15]=a6;
  a6=(a3*a4);
  a8=(a9*a8);
  a8=(a0*a8);
  a6=(a6+a8);
  if (res[1]!=0) res[1][16]=a6;
  a6=arg[1]? arg[1][19] : 0;
  if (res[1]!=0) res[1][17]=a6;
  a4=(a5*a4);
  a4=(-a4);
  if (res[1]!=0) res[1][18]=a4;
  a6=(a5*a6);
  a6=(-a6);
  if (res[1]!=0) res[1][19]=a6;
  a6=arg[1]? arg[1][23] : 0;
  a4=(a2*a6);
  a8=arg[1]? arg[1][22] : 0;
  a7=(a7*a8);
  a7=(a0*a7);
  a4=(a4-a7);
  if (res[1]!=0) res[1][20]=a4;
  a4=(a3*a6);
  a9=(a9*a8);
  a9=(a0*a9);
  a4=(a4+a9);
  if (res[1]!=0) res[1][21]=a4;
  a4=arg[1]? arg[1][24] : 0;
  if (res[1]!=0) res[1][22]=a4;
  a6=(a5*a6);
  a6=(-a6);
  if (res[1]!=0) res[1][23]=a6;
  a4=(a5*a4);
  a4=(-a4);
  if (res[1]!=0) res[1][24]=a4;
  a4=arg[2]? arg[2][3] : 0;
  a6=(a2*a4);
  a9=sin(a1);
  a8=arg[2]? arg[2][2] : 0;
  a7=(a9*a8);
  a7=(a0*a7);
  a6=(a6-a7);
  if (res[2]!=0) res[2][0]=a6;
  a6=(a3*a4);
  a1=cos(a1);
  a8=(a1*a8);
  a8=(a0*a8);
  a6=(a6+a8);
  if (res[2]!=0) res[2][1]=a6;
  a6=arg[2]? arg[2][4] : 0;
  if (res[2]!=0) res[2][2]=a6;
  a4=(a5*a4);
  a4=(a5-a4);
  if (res[2]!=0) res[2][3]=a4;
  a6=(a5*a6);
  a6=(-a6);
  if (res[2]!=0) res[2][4]=a6;
  a6=arg[2]? arg[2][8] : 0;
  a2=(a2*a6);
  a4=arg[2]? arg[2][7] : 0;
  a9=(a9*a4);
  a9=(a0*a9);
  a2=(a2-a9);
  if (res[2]!=0) res[2][5]=a2;
  a3=(a3*a6);
  a1=(a1*a4);
  a0=(a0*a1);
  a3=(a3+a0);
  if (res[2]!=0) res[2][6]=a3;
  a3=arg[2]? arg[2][9] : 0;
  if (res[2]!=0) res[2][7]=a3;
  a6=(a5*a6);
  a6=(-a6);
  if (res[2]!=0) res[2][8]=a6;
  a3=(a5*a3);
  a5=(a5-a3);
  if (res[2]!=0) res[2][9]=a5;
  return 0;
}

CASADI_SYMBOL_EXPORT int circle_car_expl_vde_forw(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int circle_car_expl_vde_forw_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int circle_car_expl_vde_forw_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void circle_car_expl_vde_forw_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int circle_car_expl_vde_forw_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void circle_car_expl_vde_forw_release(int mem) {
}

CASADI_SYMBOL_EXPORT void circle_car_expl_vde_forw_incref(void) {
}

CASADI_SYMBOL_EXPORT void circle_car_expl_vde_forw_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int circle_car_expl_vde_forw_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int circle_car_expl_vde_forw_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real circle_car_expl_vde_forw_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* circle_car_expl_vde_forw_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* circle_car_expl_vde_forw_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* circle_car_expl_vde_forw_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    case 4: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* circle_car_expl_vde_forw_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int circle_car_expl_vde_forw_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
