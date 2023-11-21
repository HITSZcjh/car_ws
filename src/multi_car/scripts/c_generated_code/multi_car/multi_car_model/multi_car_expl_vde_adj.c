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
  #define CASADI_PREFIX(ID) multi_car_expl_vde_adj_ ## ID
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

static const casadi_int casadi_s0[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s1[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s2[266] = {262, 1, 0, 262, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260, 261};
static const casadi_int casadi_s3[24] = {20, 1, 0, 20, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};

/* multi_car_expl_vde_adj:(i0[14],i1[14],i2[6],i3[262])->(o0[20]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3, a4, a5, a6;
  a0=0.;
  if (res[0]!=0) res[0][0]=a0;
  if (res[0]!=0) res[0][1]=a0;
  a1=arg[0]? arg[0][2] : 0;
  a2=cos(a1);
  a3=arg[0]? arg[0][3] : 0;
  a4=arg[1]? arg[1][1] : 0;
  a5=(a3*a4);
  a2=(a2*a5);
  a5=sin(a1);
  a6=arg[1]? arg[1][0] : 0;
  a3=(a3*a6);
  a5=(a5*a3);
  a2=(a2-a5);
  if (res[0]!=0) res[0][2]=a2;
  a2=sin(a1);
  a2=(a2*a4);
  a1=cos(a1);
  a1=(a1*a6);
  a2=(a2+a1);
  if (res[0]!=0) res[0][3]=a2;
  a2=arg[1]? arg[1][2] : 0;
  if (res[0]!=0) res[0][4]=a2;
  if (res[0]!=0) res[0][5]=a0;
  a2=arg[1]? arg[1][5] : 0;
  if (res[0]!=0) res[0][6]=a2;
  if (res[0]!=0) res[0][7]=a0;
  if (res[0]!=0) res[0][8]=a0;
  a2=arg[0]? arg[0][9] : 0;
  a1=cos(a2);
  a6=arg[0]? arg[0][10] : 0;
  a4=arg[1]? arg[1][8] : 0;
  a5=(a6*a4);
  a1=(a1*a5);
  a5=sin(a2);
  a3=arg[1]? arg[1][7] : 0;
  a6=(a6*a3);
  a5=(a5*a6);
  a1=(a1-a5);
  if (res[0]!=0) res[0][9]=a1;
  a1=sin(a2);
  a1=(a1*a4);
  a2=cos(a2);
  a2=(a2*a3);
  a1=(a1+a2);
  if (res[0]!=0) res[0][10]=a1;
  a1=arg[1]? arg[1][9] : 0;
  if (res[0]!=0) res[0][11]=a1;
  if (res[0]!=0) res[0][12]=a0;
  a0=arg[1]? arg[1][12] : 0;
  if (res[0]!=0) res[0][13]=a0;
  a0=arg[1]? arg[1][3] : 0;
  if (res[0]!=0) res[0][14]=a0;
  a0=arg[1]? arg[1][4] : 0;
  if (res[0]!=0) res[0][15]=a0;
  a0=arg[1]? arg[1][6] : 0;
  if (res[0]!=0) res[0][16]=a0;
  a0=arg[1]? arg[1][10] : 0;
  if (res[0]!=0) res[0][17]=a0;
  a0=arg[1]? arg[1][11] : 0;
  if (res[0]!=0) res[0][18]=a0;
  a0=arg[1]? arg[1][13] : 0;
  if (res[0]!=0) res[0][19]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int multi_car_expl_vde_adj(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int multi_car_expl_vde_adj_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int multi_car_expl_vde_adj_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void multi_car_expl_vde_adj_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int multi_car_expl_vde_adj_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void multi_car_expl_vde_adj_release(int mem) {
}

CASADI_SYMBOL_EXPORT void multi_car_expl_vde_adj_incref(void) {
}

CASADI_SYMBOL_EXPORT void multi_car_expl_vde_adj_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int multi_car_expl_vde_adj_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int multi_car_expl_vde_adj_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real multi_car_expl_vde_adj_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* multi_car_expl_vde_adj_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* multi_car_expl_vde_adj_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* multi_car_expl_vde_adj_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* multi_car_expl_vde_adj_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int multi_car_expl_vde_adj_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
