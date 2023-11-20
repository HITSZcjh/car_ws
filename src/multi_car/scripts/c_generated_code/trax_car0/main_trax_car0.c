/*
 * Copyright (c) The acados authors.
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


// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_trax_car0.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#define NX     TRAX_CAR0_NX
#define NZ     TRAX_CAR0_NZ
#define NU     TRAX_CAR0_NU
#define NP     TRAX_CAR0_NP
#define NBX    TRAX_CAR0_NBX
#define NBX0   TRAX_CAR0_NBX0
#define NBU    TRAX_CAR0_NBU
#define NSBX   TRAX_CAR0_NSBX
#define NSBU   TRAX_CAR0_NSBU
#define NSH    TRAX_CAR0_NSH
#define NSG    TRAX_CAR0_NSG
#define NSPHI  TRAX_CAR0_NSPHI
#define NSHN   TRAX_CAR0_NSHN
#define NSGN   TRAX_CAR0_NSGN
#define NSPHIN TRAX_CAR0_NSPHIN
#define NSBXN  TRAX_CAR0_NSBXN
#define NS     TRAX_CAR0_NS
#define NSN    TRAX_CAR0_NSN
#define NG     TRAX_CAR0_NG
#define NBXN   TRAX_CAR0_NBXN
#define NGN    TRAX_CAR0_NGN
#define NY0    TRAX_CAR0_NY0
#define NY     TRAX_CAR0_NY
#define NYN    TRAX_CAR0_NYN
#define NH     TRAX_CAR0_NH
#define NPHI   TRAX_CAR0_NPHI
#define NHN    TRAX_CAR0_NHN
#define NPHIN  TRAX_CAR0_NPHIN
#define NR     TRAX_CAR0_NR


int main()
{

    trax_car0_solver_capsule *acados_ocp_capsule = trax_car0_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = TRAX_CAR0_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = trax_car0_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status)
    {
        printf("trax_car0_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config *nlp_config = trax_car0_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = trax_car0_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = trax_car0_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = trax_car0_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = trax_car0_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = trax_car0_acados_get_nlp_opts(acados_ocp_capsule);

    // initial condition
    int idxbx0[NBX0];

    double lbx0[NBX0];
    double ubx0[NBX0];

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    // initialization for state values
    double x_init[NX];
    x_init[0] = 0.0;
    x_init[1] = 0.0;
    x_init[2] = 0.0;
    x_init[3] = 0.0;
    x_init[4] = 0.0;
    x_init[5] = 0.0;
    x_init[6] = 0.0;
    x_init[7] = 0.0;
    x_init[8] = 0.0;
    x_init[9] = 0.0;
    x_init[10] = 0.0;
    x_init[11] = 0.0;

    // initial value for control input
    double u0[NU];

    // prepare evaluation
    int NTIMINGS = 1;
    double min_time = 1e12;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[NX * (N+1)];
    double utraj[NU * N];


    // solve ocp in loop
    int rti_phase = 0;

    for (int ii = 0; ii < NTIMINGS; ii++)
    {
        // initialize solution
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
        }
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
        ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
        status = trax_car0_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);
    }

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);

    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( NX, N+1, xtraj, NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( NU, N, utraj, NU );
    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

    printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS)
    {
        printf("trax_car0_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("trax_car0_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    trax_car0_acados_print_stats(acados_ocp_capsule);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);

    // free solver
    status = trax_car0_acados_free(acados_ocp_capsule);
    if (status) {
        printf("trax_car0_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = trax_car0_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("trax_car0_acados_free_capsule() returned status %d. \n", status);
    }

    return status;
}
