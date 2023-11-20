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
#include "acados_solver_car1_Arcsolver_1.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#define NX     CAR1_ARCSOLVER_1_NX
#define NZ     CAR1_ARCSOLVER_1_NZ
#define NU     CAR1_ARCSOLVER_1_NU
#define NP     CAR1_ARCSOLVER_1_NP
#define NBX    CAR1_ARCSOLVER_1_NBX
#define NBX0   CAR1_ARCSOLVER_1_NBX0
#define NBU    CAR1_ARCSOLVER_1_NBU
#define NSBX   CAR1_ARCSOLVER_1_NSBX
#define NSBU   CAR1_ARCSOLVER_1_NSBU
#define NSH    CAR1_ARCSOLVER_1_NSH
#define NSG    CAR1_ARCSOLVER_1_NSG
#define NSPHI  CAR1_ARCSOLVER_1_NSPHI
#define NSHN   CAR1_ARCSOLVER_1_NSHN
#define NSGN   CAR1_ARCSOLVER_1_NSGN
#define NSPHIN CAR1_ARCSOLVER_1_NSPHIN
#define NSBXN  CAR1_ARCSOLVER_1_NSBXN
#define NS     CAR1_ARCSOLVER_1_NS
#define NSN    CAR1_ARCSOLVER_1_NSN
#define NG     CAR1_ARCSOLVER_1_NG
#define NBXN   CAR1_ARCSOLVER_1_NBXN
#define NGN    CAR1_ARCSOLVER_1_NGN
#define NY0    CAR1_ARCSOLVER_1_NY0
#define NY     CAR1_ARCSOLVER_1_NY
#define NYN    CAR1_ARCSOLVER_1_NYN
#define NH     CAR1_ARCSOLVER_1_NH
#define NPHI   CAR1_ARCSOLVER_1_NPHI
#define NHN    CAR1_ARCSOLVER_1_NHN
#define NPHIN  CAR1_ARCSOLVER_1_NPHIN
#define NR     CAR1_ARCSOLVER_1_NR


int main()
{

    car1_Arcsolver_1_solver_capsule *acados_ocp_capsule = car1_Arcsolver_1_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = CAR1_ARCSOLVER_1_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = car1_Arcsolver_1_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status)
    {
        printf("car1_Arcsolver_1_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config *nlp_config = car1_Arcsolver_1_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = car1_Arcsolver_1_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = car1_Arcsolver_1_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = car1_Arcsolver_1_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = car1_Arcsolver_1_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = car1_Arcsolver_1_acados_get_nlp_opts(acados_ocp_capsule);

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
    x_init[12] = 0.0;
    x_init[13] = 0.0;
    x_init[14] = 0.0;
    x_init[15] = 0.0;
    x_init[16] = 0.0;
    x_init[17] = 0.0;
    x_init[18] = 0.0;
    x_init[19] = 0.0;
    x_init[20] = 0.0;
    x_init[21] = 0.0;
    x_init[22] = 0.0;
    x_init[23] = 0.0;
    x_init[24] = 0.0;
    x_init[25] = 0.0;
    x_init[26] = 0.0;
    x_init[27] = 0.0;
    x_init[28] = 0.0;
    x_init[29] = 0.0;
    x_init[30] = 0.0;
    x_init[31] = 0.0;
    x_init[32] = 0.0;
    x_init[33] = 0.0;
    x_init[34] = 0.0;
    x_init[35] = 0.0;
    x_init[36] = 0.0;
    x_init[37] = 0.0;
    x_init[38] = 0.0;
    x_init[39] = 0.0;
    x_init[40] = 0.0;
    x_init[41] = 0.0;
    x_init[42] = 0.0;
    x_init[43] = 0.0;
    x_init[44] = 0.0;
    x_init[45] = 0.0;
    x_init[46] = 0.0;
    x_init[47] = 0.0;
    x_init[48] = 0.0;
    x_init[49] = 0.0;
    x_init[50] = 0.0;
    x_init[51] = 0.0;
    x_init[52] = 0.0;
    x_init[53] = 0.0;
    x_init[54] = 0.0;
    x_init[55] = 0.0;
    x_init[56] = 0.0;
    x_init[57] = 0.0;
    x_init[58] = 0.0;
    x_init[59] = 0.0;

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
        status = car1_Arcsolver_1_acados_solve(acados_ocp_capsule);
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
        printf("car1_Arcsolver_1_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("car1_Arcsolver_1_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    car1_Arcsolver_1_acados_print_stats(acados_ocp_capsule);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);

    // free solver
    status = car1_Arcsolver_1_acados_free(acados_ocp_capsule);
    if (status) {
        printf("car1_Arcsolver_1_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = car1_Arcsolver_1_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("car1_Arcsolver_1_acados_free_capsule() returned status %d. \n", status);
    }

    return status;
}