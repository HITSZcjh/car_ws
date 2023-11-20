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
  #define CASADI_PREFIX(ID) traz_car1_constr_h_fun_jac_uxt_zt_ ## ID
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

static const casadi_int casadi_s0[16] = {12, 1, 0, 12, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const casadi_int casadi_s1[3] = {0, 0, 0};
static const casadi_int casadi_s2[13] = {9, 1, 0, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8};
static const casadi_int casadi_s3[46] = {12, 9, 0, 1, 7, 8, 14, 20, 25, 29, 32, 34, 0, 0, 1, 2, 3, 4, 5, 6, 6, 7, 8, 9, 10, 11, 1, 2, 3, 4, 5, 7, 2, 3, 4, 5, 8, 3, 4, 5, 9, 4, 5, 10, 5, 11};
static const casadi_int casadi_s4[3] = {9, 0, 0};

/* traz_car1_constr_h_fun_jac_uxt_zt:(i0[12],i1[],i2[],i3[])->(o0[9],o1[12x9,34nz],o2[9x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][0] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a1=2.;
  a2=arg[0]? arg[0][1] : 0;
  a3=(a1*a2);
  a0=(a0+a3);
  a3=4.;
  a4=arg[0]? arg[0][2] : 0;
  a5=(a3*a4);
  a0=(a0+a5);
  a5=8.;
  a6=arg[0]? arg[0][3] : 0;
  a7=(a5*a6);
  a0=(a0+a7);
  a7=16.;
  a8=arg[0]? arg[0][4] : 0;
  a9=(a7*a8);
  a0=(a0+a9);
  a9=32.;
  a10=arg[0]? arg[0][5] : 0;
  a11=(a9*a10);
  a0=(a0+a11);
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[0]? arg[0][6] : 0;
  if (res[0]!=0) res[0][2]=a0;
  a11=arg[0]? arg[0][7] : 0;
  a12=(a1*a11);
  a0=(a0+a12);
  a12=arg[0]? arg[0][8] : 0;
  a13=(a3*a12);
  a0=(a0+a13);
  a13=arg[0]? arg[0][9] : 0;
  a14=(a5*a13);
  a0=(a0+a14);
  a14=arg[0]? arg[0][10] : 0;
  a15=(a7*a14);
  a0=(a0+a15);
  a15=arg[0]? arg[0][11] : 0;
  a16=(a9*a15);
  a0=(a0+a16);
  if (res[0]!=0) res[0][3]=a0;
  a0=(a3*a4);
  a2=(a2+a0);
  a0=12.;
  a16=(a0*a6);
  a2=(a2+a16);
  a16=(a9*a8);
  a2=(a2+a16);
  a16=80.;
  a17=(a16*a10);
  a2=(a2+a17);
  a2=(a2-a11);
  if (res[0]!=0) res[0][4]=a2;
  a4=(a1*a4);
  a2=(a0*a6);
  a4=(a4+a2);
  a2=48.;
  a11=(a2*a8);
  a4=(a4+a11);
  a11=160.;
  a17=(a11*a10);
  a4=(a4+a17);
  a17=-2.;
  a12=(a17*a12);
  a4=(a4+a12);
  if (res[0]!=0) res[0][5]=a4;
  a4=6.;
  a6=(a4*a6);
  a12=(a2*a8);
  a6=(a6+a12);
  a12=240.;
  a18=(a12*a10);
  a6=(a6+a18);
  a18=-6.;
  a13=(a18*a13);
  a6=(a6+a13);
  if (res[0]!=0) res[0][6]=a6;
  a6=24.;
  a8=(a6*a8);
  a13=(a12*a10);
  a8=(a8+a13);
  a13=-24.;
  a14=(a13*a14);
  a8=(a8+a14);
  if (res[0]!=0) res[0][7]=a8;
  a8=120.;
  a10=(a8*a10);
  a14=-120.;
  a15=(a14*a15);
  a10=(a10+a15);
  if (res[0]!=0) res[0][8]=a10;
  a10=1.;
  if (res[1]!=0) res[1][0]=a10;
  if (res[1]!=0) res[1][1]=a10;
  if (res[1]!=0) res[1][2]=a1;
  if (res[1]!=0) res[1][3]=a3;
  if (res[1]!=0) res[1][4]=a5;
  if (res[1]!=0) res[1][5]=a7;
  if (res[1]!=0) res[1][6]=a9;
  if (res[1]!=0) res[1][7]=a10;
  if (res[1]!=0) res[1][8]=a10;
  if (res[1]!=0) res[1][9]=a1;
  if (res[1]!=0) res[1][10]=a3;
  if (res[1]!=0) res[1][11]=a5;
  if (res[1]!=0) res[1][12]=a7;
  if (res[1]!=0) res[1][13]=a9;
  if (res[1]!=0) res[1][14]=a10;
  if (res[1]!=0) res[1][15]=a3;
  if (res[1]!=0) res[1][16]=a0;
  if (res[1]!=0) res[1][17]=a9;
  if (res[1]!=0) res[1][18]=a16;
  a16=-1.;
  if (res[1]!=0) res[1][19]=a16;
  if (res[1]!=0) res[1][20]=a1;
  if (res[1]!=0) res[1][21]=a0;
  if (res[1]!=0) res[1][22]=a2;
  if (res[1]!=0) res[1][23]=a11;
  if (res[1]!=0) res[1][24]=a17;
  if (res[1]!=0) res[1][25]=a4;
  if (res[1]!=0) res[1][26]=a2;
  if (res[1]!=0) res[1][27]=a12;
  if (res[1]!=0) res[1][28]=a18;
  if (res[1]!=0) res[1][29]=a6;
  if (res[1]!=0) res[1][30]=a12;
  if (res[1]!=0) res[1][31]=a13;
  if (res[1]!=0) res[1][32]=a8;
  if (res[1]!=0) res[1][33]=a14;
  return 0;
}

CASADI_SYMBOL_EXPORT int traz_car1_constr_h_fun_jac_uxt_zt(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int traz_car1_constr_h_fun_jac_uxt_zt_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int traz_car1_constr_h_fun_jac_uxt_zt_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void traz_car1_constr_h_fun_jac_uxt_zt_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int traz_car1_constr_h_fun_jac_uxt_zt_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void traz_car1_constr_h_fun_jac_uxt_zt_release(int mem) {
}

CASADI_SYMBOL_EXPORT void traz_car1_constr_h_fun_jac_uxt_zt_incref(void) {
}

CASADI_SYMBOL_EXPORT void traz_car1_constr_h_fun_jac_uxt_zt_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int traz_car1_constr_h_fun_jac_uxt_zt_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int traz_car1_constr_h_fun_jac_uxt_zt_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real traz_car1_constr_h_fun_jac_uxt_zt_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* traz_car1_constr_h_fun_jac_uxt_zt_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* traz_car1_constr_h_fun_jac_uxt_zt_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* traz_car1_constr_h_fun_jac_uxt_zt_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    case 3: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* traz_car1_constr_h_fun_jac_uxt_zt_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    case 1: return casadi_s3;
    case 2: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int traz_car1_constr_h_fun_jac_uxt_zt_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
