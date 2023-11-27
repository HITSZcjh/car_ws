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
  #define CASADI_PREFIX(ID) trax_car0_dyn_disc_phi_fun_jac_ ## ID
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

static const casadi_int casadi_s0[16] = {12, 1, 0, 12, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const casadi_int casadi_s1[3] = {0, 0, 0};
static const casadi_int casadi_s2[27] = {12, 12, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

/* trax_car0_dyn_disc_phi_fun_jac:(i0[12],i1[],i2[])->(o0[12],o1[12x12,12nz]) */
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
  a0=1.;
  if (res[1]!=0) res[1][0]=a0;
  if (res[1]!=0) res[1][1]=a0;
  if (res[1]!=0) res[1][2]=a0;
  if (res[1]!=0) res[1][3]=a0;
  if (res[1]!=0) res[1][4]=a0;
  if (res[1]!=0) res[1][5]=a0;
  if (res[1]!=0) res[1][6]=a0;
  if (res[1]!=0) res[1][7]=a0;
  if (res[1]!=0) res[1][8]=a0;
  if (res[1]!=0) res[1][9]=a0;
  if (res[1]!=0) res[1][10]=a0;
  if (res[1]!=0) res[1][11]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int trax_car0_dyn_disc_phi_fun_jac(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int trax_car0_dyn_disc_phi_fun_jac_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int trax_car0_dyn_disc_phi_fun_jac_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void trax_car0_dyn_disc_phi_fun_jac_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int trax_car0_dyn_disc_phi_fun_jac_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void trax_car0_dyn_disc_phi_fun_jac_release(int mem) {
}

CASADI_SYMBOL_EXPORT void trax_car0_dyn_disc_phi_fun_jac_incref(void) {
}

CASADI_SYMBOL_EXPORT void trax_car0_dyn_disc_phi_fun_jac_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int trax_car0_dyn_disc_phi_fun_jac_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int trax_car0_dyn_disc_phi_fun_jac_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real trax_car0_dyn_disc_phi_fun_jac_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* trax_car0_dyn_disc_phi_fun_jac_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* trax_car0_dyn_disc_phi_fun_jac_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* trax_car0_dyn_disc_phi_fun_jac_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* trax_car0_dyn_disc_phi_fun_jac_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int trax_car0_dyn_disc_phi_fun_jac_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
