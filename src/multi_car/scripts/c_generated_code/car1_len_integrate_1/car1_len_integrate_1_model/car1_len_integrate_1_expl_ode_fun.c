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
  #define CASADI_PREFIX(ID) car1_len_integrate_1_expl_ode_fun_ ## ID
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

/* car1_len_integrate_1_expl_ode_fun:(i0[5],i1[],i2[])->(o0[5]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3, a4, a5, a6, a7, a8;
  a0=1.;
  if (res[0]!=0) res[0][0]=a0;
  a0=2.7390591533033826e-01;
  a1=2.;
  a2=-1.3695295766515098e-01;
  a3=arg[0]? arg[0][0] : 0;
  a2=(a2*a3);
  a2=(a1*a2);
  a0=(a0+a2);
  a2=3.;
  a4=-1.1763033437455368e-14;
  a5=casadi_sq(a3);
  a4=(a4*a5);
  a4=(a2*a4);
  a0=(a0+a4);
  a4=4.;
  a5=-6.0114450199362596e-17;
  a6=casadi_sq(a3);
  a6=(a3*a6);
  a5=(a5*a6);
  a5=(a4*a5);
  a0=(a0+a5);
  a5=5.;
  a6=7.0050584297890097e-16;
  a7=casadi_sq(a3);
  a7=casadi_sq(a7);
  a6=(a6*a7);
  a6=(a5*a6);
  a0=(a0+a6);
  if (res[0]!=0) res[0][1]=a0;
  a6=-9.6676974059994747e-01;
  a7=8.5838487030233357e-01;
  a7=(a7*a3);
  a7=(a1*a7);
  a6=(a6+a7);
  a7=5.5989405883711420e-15;
  a8=casadi_sq(a3);
  a7=(a7*a8);
  a7=(a2*a7);
  a6=(a6+a7);
  a7=-1.2754477207970630e-15;
  a8=casadi_sq(a3);
  a8=(a3*a8);
  a7=(a7*a8);
  a7=(a4*a7);
  a6=(a6+a7);
  a7=2.1271037021637919e-16;
  a8=casadi_sq(a3);
  a8=casadi_sq(a8);
  a7=(a7*a8);
  a7=(a5*a7);
  a6=(a6+a7);
  if (res[0]!=0) res[0][2]=a6;
  a7=-2.3387276819641367e-04;
  a8=1.1693638409820935e-04;
  a8=(a8*a3);
  a1=(a1*a8);
  a7=(a7+a1);
  a1=-1.3886964025006210e-18;
  a8=casadi_sq(a3);
  a1=(a1*a8);
  a2=(a2*a1);
  a7=(a7+a2);
  a2=-1.0351018864904509e-19;
  a1=casadi_sq(a3);
  a1=(a3*a1);
  a2=(a2*a1);
  a4=(a4*a2);
  a7=(a7+a4);
  a4=8.7974541363880506e-20;
  a3=casadi_sq(a3);
  a3=casadi_sq(a3);
  a4=(a4*a3);
  a5=(a5*a4);
  a7=(a7+a5);
  if (res[0]!=0) res[0][3]=a7;
  a0=casadi_sq(a0);
  a6=casadi_sq(a6);
  a0=(a0+a6);
  a7=casadi_sq(a7);
  a0=(a0+a7);
  a0=sqrt(a0);
  if (res[0]!=0) res[0][4]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int car1_len_integrate_1_expl_ode_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int car1_len_integrate_1_expl_ode_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int car1_len_integrate_1_expl_ode_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void car1_len_integrate_1_expl_ode_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int car1_len_integrate_1_expl_ode_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void car1_len_integrate_1_expl_ode_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void car1_len_integrate_1_expl_ode_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void car1_len_integrate_1_expl_ode_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int car1_len_integrate_1_expl_ode_fun_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int car1_len_integrate_1_expl_ode_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real car1_len_integrate_1_expl_ode_fun_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car1_len_integrate_1_expl_ode_fun_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car1_len_integrate_1_expl_ode_fun_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* car1_len_integrate_1_expl_ode_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* car1_len_integrate_1_expl_ode_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int car1_len_integrate_1_expl_ode_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
