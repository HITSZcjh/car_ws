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
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a5, a6, a7, a8, a9;
  a0=1.;
  if (res[0]!=0) res[0][0]=a0;
  a0=4.3180632077128411e-01;
  a1=2.;
  a2=-2.1590316038565099e-01;
  a3=arg[0]? arg[0][0] : 0;
  a4=(a2*a3);
  a4=(a1*a4);
  a0=(a0+a4);
  a4=3.;
  a5=6.3132474531808760e-15;
  a6=casadi_sq(a3);
  a6=(a5*a6);
  a6=(a4*a6);
  a0=(a0+a6);
  a6=4.;
  a7=-1.0610549310894335e-15;
  a8=casadi_sq(a3);
  a9=(a3*a8);
  a9=(a7*a9);
  a9=(a6*a9);
  a0=(a0+a9);
  a9=5.;
  a10=6.9885238944000946e-17;
  a11=casadi_sq(a3);
  a12=casadi_sq(a11);
  a12=(a10*a12);
  a12=(a9*a12);
  a0=(a0+a12);
  if (res[0]!=0) res[0][1]=a0;
  a12=-1.3558291160182468e-02;
  a13=-3.6822085441990859e-01;
  a14=(a13*a3);
  a14=(a1*a14);
  a12=(a12+a14);
  a14=-8.7820069986603468e-17;
  a15=casadi_sq(a3);
  a15=(a14*a15);
  a15=(a4*a15);
  a12=(a12+a15);
  a15=-1.9974992678890871e-17;
  a16=casadi_sq(a3);
  a17=(a3*a16);
  a17=(a15*a17);
  a17=(a6*a17);
  a12=(a12+a17);
  a17=1.0206488354374291e-17;
  a18=casadi_sq(a3);
  a19=casadi_sq(a18);
  a19=(a17*a19);
  a19=(a9*a19);
  a12=(a12+a19);
  if (res[0]!=0) res[0][2]=a12;
  a19=1.2070466994483009e+00;
  a20=-6.0352334972409194e-01;
  a21=(a20*a3);
  a21=(a1*a21);
  a19=(a19+a21);
  a21=-3.5665101487957189e-14;
  a22=casadi_sq(a3);
  a22=(a21*a22);
  a22=(a4*a22);
  a19=(a19+a22);
  a22=5.9411943181263905e-16;
  a23=casadi_sq(a3);
  a24=(a3*a23);
  a24=(a22*a24);
  a24=(a6*a24);
  a19=(a19+a24);
  a24=1.3070327884073397e-15;
  a25=casadi_sq(a3);
  a26=casadi_sq(a25);
  a26=(a24*a26);
  a26=(a9*a26);
  a19=(a19+a26);
  if (res[0]!=0) res[0][3]=a19;
  a26=casadi_sq(a0);
  a27=casadi_sq(a12);
  a26=(a26+a27);
  a27=casadi_sq(a19);
  a26=(a26+a27);
  a26=sqrt(a26);
  if (res[0]!=0) res[0][4]=a26;
  a27=0.;
  if (res[1]!=0) res[1][0]=a27;
  a28=arg[1]? arg[1][0] : 0;
  a29=(a2*a28);
  a29=(a1*a29);
  a30=(a3+a3);
  a31=(a30*a28);
  a31=(a5*a31);
  a31=(a4*a31);
  a29=(a29+a31);
  a31=(a8*a28);
  a32=(a3+a3);
  a33=(a32*a28);
  a33=(a3*a33);
  a31=(a31+a33);
  a31=(a7*a31);
  a31=(a6*a31);
  a29=(a29+a31);
  a11=(a11+a11);
  a31=(a3+a3);
  a33=(a31*a28);
  a33=(a11*a33);
  a33=(a10*a33);
  a33=(a9*a33);
  a29=(a29+a33);
  if (res[1]!=0) res[1][1]=a29;
  a33=(a13*a28);
  a33=(a1*a33);
  a34=(a3+a3);
  a35=(a34*a28);
  a35=(a14*a35);
  a35=(a4*a35);
  a33=(a33+a35);
  a35=(a16*a28);
  a36=(a3+a3);
  a37=(a36*a28);
  a37=(a3*a37);
  a35=(a35+a37);
  a35=(a15*a35);
  a35=(a6*a35);
  a33=(a33+a35);
  a18=(a18+a18);
  a35=(a3+a3);
  a37=(a35*a28);
  a37=(a18*a37);
  a37=(a17*a37);
  a37=(a9*a37);
  a33=(a33+a37);
  if (res[1]!=0) res[1][2]=a33;
  a37=(a20*a28);
  a37=(a1*a37);
  a38=(a3+a3);
  a39=(a38*a28);
  a39=(a21*a39);
  a39=(a4*a39);
  a37=(a37+a39);
  a39=(a23*a28);
  a40=(a3+a3);
  a41=(a40*a28);
  a41=(a3*a41);
  a39=(a39+a41);
  a39=(a22*a39);
  a39=(a6*a39);
  a37=(a37+a39);
  a25=(a25+a25);
  a39=(a3+a3);
  a28=(a39*a28);
  a28=(a25*a28);
  a28=(a24*a28);
  a28=(a9*a28);
  a37=(a37+a28);
  if (res[1]!=0) res[1][3]=a37;
  a0=(a0+a0);
  a29=(a0*a29);
  a12=(a12+a12);
  a33=(a12*a33);
  a29=(a29+a33);
  a19=(a19+a19);
  a37=(a19*a37);
  a29=(a29+a37);
  a26=(a26+a26);
  a29=(a29/a26);
  if (res[1]!=0) res[1][4]=a29;
  if (res[1]!=0) res[1][5]=a27;
  a29=arg[1]? arg[1][5] : 0;
  a37=(a2*a29);
  a37=(a1*a37);
  a33=(a30*a29);
  a33=(a5*a33);
  a33=(a4*a33);
  a37=(a37+a33);
  a33=(a8*a29);
  a28=(a32*a29);
  a28=(a3*a28);
  a33=(a33+a28);
  a33=(a7*a33);
  a33=(a6*a33);
  a37=(a37+a33);
  a33=(a31*a29);
  a33=(a11*a33);
  a33=(a10*a33);
  a33=(a9*a33);
  a37=(a37+a33);
  if (res[1]!=0) res[1][6]=a37;
  a33=(a13*a29);
  a33=(a1*a33);
  a28=(a34*a29);
  a28=(a14*a28);
  a28=(a4*a28);
  a33=(a33+a28);
  a28=(a16*a29);
  a41=(a36*a29);
  a41=(a3*a41);
  a28=(a28+a41);
  a28=(a15*a28);
  a28=(a6*a28);
  a33=(a33+a28);
  a28=(a35*a29);
  a28=(a18*a28);
  a28=(a17*a28);
  a28=(a9*a28);
  a33=(a33+a28);
  if (res[1]!=0) res[1][7]=a33;
  a28=(a20*a29);
  a28=(a1*a28);
  a41=(a38*a29);
  a41=(a21*a41);
  a41=(a4*a41);
  a28=(a28+a41);
  a41=(a23*a29);
  a42=(a40*a29);
  a42=(a3*a42);
  a41=(a41+a42);
  a41=(a22*a41);
  a41=(a6*a41);
  a28=(a28+a41);
  a29=(a39*a29);
  a29=(a25*a29);
  a29=(a24*a29);
  a29=(a9*a29);
  a28=(a28+a29);
  if (res[1]!=0) res[1][8]=a28;
  a37=(a0*a37);
  a33=(a12*a33);
  a37=(a37+a33);
  a28=(a19*a28);
  a37=(a37+a28);
  a37=(a37/a26);
  if (res[1]!=0) res[1][9]=a37;
  if (res[1]!=0) res[1][10]=a27;
  a37=arg[1]? arg[1][10] : 0;
  a28=(a2*a37);
  a28=(a1*a28);
  a33=(a30*a37);
  a33=(a5*a33);
  a33=(a4*a33);
  a28=(a28+a33);
  a33=(a8*a37);
  a29=(a32*a37);
  a29=(a3*a29);
  a33=(a33+a29);
  a33=(a7*a33);
  a33=(a6*a33);
  a28=(a28+a33);
  a33=(a31*a37);
  a33=(a11*a33);
  a33=(a10*a33);
  a33=(a9*a33);
  a28=(a28+a33);
  if (res[1]!=0) res[1][11]=a28;
  a33=(a13*a37);
  a33=(a1*a33);
  a29=(a34*a37);
  a29=(a14*a29);
  a29=(a4*a29);
  a33=(a33+a29);
  a29=(a16*a37);
  a41=(a36*a37);
  a41=(a3*a41);
  a29=(a29+a41);
  a29=(a15*a29);
  a29=(a6*a29);
  a33=(a33+a29);
  a29=(a35*a37);
  a29=(a18*a29);
  a29=(a17*a29);
  a29=(a9*a29);
  a33=(a33+a29);
  if (res[1]!=0) res[1][12]=a33;
  a29=(a20*a37);
  a29=(a1*a29);
  a41=(a38*a37);
  a41=(a21*a41);
  a41=(a4*a41);
  a29=(a29+a41);
  a41=(a23*a37);
  a42=(a40*a37);
  a42=(a3*a42);
  a41=(a41+a42);
  a41=(a22*a41);
  a41=(a6*a41);
  a29=(a29+a41);
  a37=(a39*a37);
  a37=(a25*a37);
  a37=(a24*a37);
  a37=(a9*a37);
  a29=(a29+a37);
  if (res[1]!=0) res[1][13]=a29;
  a28=(a0*a28);
  a33=(a12*a33);
  a28=(a28+a33);
  a29=(a19*a29);
  a28=(a28+a29);
  a28=(a28/a26);
  if (res[1]!=0) res[1][14]=a28;
  if (res[1]!=0) res[1][15]=a27;
  a28=arg[1]? arg[1][15] : 0;
  a29=(a2*a28);
  a29=(a1*a29);
  a33=(a30*a28);
  a33=(a5*a33);
  a33=(a4*a33);
  a29=(a29+a33);
  a33=(a8*a28);
  a37=(a32*a28);
  a37=(a3*a37);
  a33=(a33+a37);
  a33=(a7*a33);
  a33=(a6*a33);
  a29=(a29+a33);
  a33=(a31*a28);
  a33=(a11*a33);
  a33=(a10*a33);
  a33=(a9*a33);
  a29=(a29+a33);
  if (res[1]!=0) res[1][16]=a29;
  a33=(a13*a28);
  a33=(a1*a33);
  a37=(a34*a28);
  a37=(a14*a37);
  a37=(a4*a37);
  a33=(a33+a37);
  a37=(a16*a28);
  a41=(a36*a28);
  a41=(a3*a41);
  a37=(a37+a41);
  a37=(a15*a37);
  a37=(a6*a37);
  a33=(a33+a37);
  a37=(a35*a28);
  a37=(a18*a37);
  a37=(a17*a37);
  a37=(a9*a37);
  a33=(a33+a37);
  if (res[1]!=0) res[1][17]=a33;
  a37=(a20*a28);
  a37=(a1*a37);
  a41=(a38*a28);
  a41=(a21*a41);
  a41=(a4*a41);
  a37=(a37+a41);
  a41=(a23*a28);
  a42=(a40*a28);
  a42=(a3*a42);
  a41=(a41+a42);
  a41=(a22*a41);
  a41=(a6*a41);
  a37=(a37+a41);
  a28=(a39*a28);
  a28=(a25*a28);
  a28=(a24*a28);
  a28=(a9*a28);
  a37=(a37+a28);
  if (res[1]!=0) res[1][18]=a37;
  a29=(a0*a29);
  a33=(a12*a33);
  a29=(a29+a33);
  a37=(a19*a37);
  a29=(a29+a37);
  a29=(a29/a26);
  if (res[1]!=0) res[1][19]=a29;
  if (res[1]!=0) res[1][20]=a27;
  a27=arg[1]? arg[1][20] : 0;
  a2=(a2*a27);
  a2=(a1*a2);
  a30=(a30*a27);
  a5=(a5*a30);
  a5=(a4*a5);
  a2=(a2+a5);
  a8=(a8*a27);
  a32=(a32*a27);
  a32=(a3*a32);
  a8=(a8+a32);
  a7=(a7*a8);
  a7=(a6*a7);
  a2=(a2+a7);
  a31=(a31*a27);
  a11=(a11*a31);
  a10=(a10*a11);
  a10=(a9*a10);
  a2=(a2+a10);
  if (res[1]!=0) res[1][21]=a2;
  a13=(a13*a27);
  a13=(a1*a13);
  a34=(a34*a27);
  a14=(a14*a34);
  a14=(a4*a14);
  a13=(a13+a14);
  a16=(a16*a27);
  a36=(a36*a27);
  a36=(a3*a36);
  a16=(a16+a36);
  a15=(a15*a16);
  a15=(a6*a15);
  a13=(a13+a15);
  a35=(a35*a27);
  a18=(a18*a35);
  a17=(a17*a18);
  a17=(a9*a17);
  a13=(a13+a17);
  if (res[1]!=0) res[1][22]=a13;
  a20=(a20*a27);
  a1=(a1*a20);
  a38=(a38*a27);
  a21=(a21*a38);
  a4=(a4*a21);
  a1=(a1+a4);
  a23=(a23*a27);
  a40=(a40*a27);
  a3=(a3*a40);
  a23=(a23+a3);
  a22=(a22*a23);
  a6=(a6*a22);
  a1=(a1+a6);
  a39=(a39*a27);
  a25=(a25*a39);
  a24=(a24*a25);
  a9=(a9*a24);
  a1=(a1+a9);
  if (res[1]!=0) res[1][23]=a1;
  a0=(a0*a2);
  a12=(a12*a13);
  a0=(a0+a12);
  a19=(a19*a1);
  a0=(a0+a19);
  a0=(a0/a26);
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

CASADI_SYMBOL_EXPORT casadi_real car0_len_integrate_1_expl_vde_forw_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car0_len_integrate_1_expl_vde_forw_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* car0_len_integrate_1_expl_vde_forw_name_out(casadi_int i){
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
