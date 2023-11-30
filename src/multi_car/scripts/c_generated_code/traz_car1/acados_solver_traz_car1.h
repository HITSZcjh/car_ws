/*
 * Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
 * Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
 * Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
 * Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#ifndef ACADOS_SOLVER_traz_car1_H_
#define ACADOS_SOLVER_traz_car1_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define TRAZ_CAR1_NX     12
#define TRAZ_CAR1_NZ     0
#define TRAZ_CAR1_NU     0
#define TRAZ_CAR1_NP     0
#define TRAZ_CAR1_NBX    0
#define TRAZ_CAR1_NBX0   0
#define TRAZ_CAR1_NBU    0
#define TRAZ_CAR1_NSBX   0
#define TRAZ_CAR1_NSBU   0
#define TRAZ_CAR1_NSH    0
#define TRAZ_CAR1_NSG    0
#define TRAZ_CAR1_NSPHI  0
#define TRAZ_CAR1_NSHN   0
#define TRAZ_CAR1_NSGN   0
#define TRAZ_CAR1_NSPHIN 0
#define TRAZ_CAR1_NSBXN  0
#define TRAZ_CAR1_NS     0
#define TRAZ_CAR1_NSN    0
#define TRAZ_CAR1_NG     0
#define TRAZ_CAR1_NBXN   0
#define TRAZ_CAR1_NGN    0
#define TRAZ_CAR1_NY0    0
#define TRAZ_CAR1_NY     0
#define TRAZ_CAR1_NYN    0
#define TRAZ_CAR1_N      1
#define TRAZ_CAR1_NH     9
#define TRAZ_CAR1_NPHI   0
#define TRAZ_CAR1_NHN    0
#define TRAZ_CAR1_NPHIN  0
#define TRAZ_CAR1_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct traz_car1_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */
    // dynamics

    external_function_param_casadi *discr_dyn_phi_fun;
    external_function_param_casadi *discr_dyn_phi_fun_jac_ut_xt;
    external_function_param_casadi *discr_dyn_phi_fun_jac_ut_xt_hess;


    // cost





    external_function_param_casadi ext_cost_e_fun;
    external_function_param_casadi ext_cost_e_fun_jac;
    external_function_param_casadi ext_cost_e_fun_jac_hess;

    // constraints
    external_function_param_casadi *nl_constr_h_fun_jac;
    external_function_param_casadi *nl_constr_h_fun;
    external_function_param_casadi *nl_constr_h_fun_jac_hess;




} traz_car1_solver_capsule;

ACADOS_SYMBOL_EXPORT traz_car1_solver_capsule * traz_car1_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int traz_car1_acados_free_capsule(traz_car1_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int traz_car1_acados_create(traz_car1_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int traz_car1_acados_reset(traz_car1_solver_capsule* capsule, int reset_qp_solver_mem);

/**
 * Generic version of traz_car1_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int traz_car1_acados_create_with_discretization(traz_car1_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int traz_car1_acados_update_time_steps(traz_car1_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int traz_car1_acados_update_qp_solver_cond_N(traz_car1_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int traz_car1_acados_update_params(traz_car1_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int traz_car1_acados_update_params_sparse(traz_car1_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);

ACADOS_SYMBOL_EXPORT int traz_car1_acados_solve(traz_car1_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int traz_car1_acados_free(traz_car1_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void traz_car1_acados_print_stats(traz_car1_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int traz_car1_acados_custom_update(traz_car1_solver_capsule* capsule, double* data, int data_len);


ACADOS_SYMBOL_EXPORT ocp_nlp_in *traz_car1_acados_get_nlp_in(traz_car1_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *traz_car1_acados_get_nlp_out(traz_car1_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *traz_car1_acados_get_sens_out(traz_car1_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *traz_car1_acados_get_nlp_solver(traz_car1_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *traz_car1_acados_get_nlp_config(traz_car1_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *traz_car1_acados_get_nlp_opts(traz_car1_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *traz_car1_acados_get_nlp_dims(traz_car1_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *traz_car1_acados_get_nlp_plan(traz_car1_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_traz_car1_H_
