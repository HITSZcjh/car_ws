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


// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_multi_car.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#define NX     MULTI_CAR_NX
#define NZ     MULTI_CAR_NZ
#define NU     MULTI_CAR_NU
#define NP     MULTI_CAR_NP
#define NBX    MULTI_CAR_NBX
#define NBX0   MULTI_CAR_NBX0
#define NBU    MULTI_CAR_NBU
#define NSBX   MULTI_CAR_NSBX
#define NSBU   MULTI_CAR_NSBU
#define NSH    MULTI_CAR_NSH
#define NSG    MULTI_CAR_NSG
#define NSPHI  MULTI_CAR_NSPHI
#define NSHN   MULTI_CAR_NSHN
#define NSGN   MULTI_CAR_NSGN
#define NSPHIN MULTI_CAR_NSPHIN
#define NSBXN  MULTI_CAR_NSBXN
#define NS     MULTI_CAR_NS
#define NSN    MULTI_CAR_NSN
#define NG     MULTI_CAR_NG
#define NBXN   MULTI_CAR_NBXN
#define NGN    MULTI_CAR_NGN
#define NY0    MULTI_CAR_NY0
#define NY     MULTI_CAR_NY
#define NYN    MULTI_CAR_NYN
#define NH     MULTI_CAR_NH
#define NPHI   MULTI_CAR_NPHI
#define NHN    MULTI_CAR_NHN
#define NPHIN  MULTI_CAR_NPHIN
#define NR     MULTI_CAR_NR


int main()
{

    multi_car_solver_capsule *acados_ocp_capsule = multi_car_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = MULTI_CAR_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = multi_car_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status)
    {
        printf("multi_car_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config *nlp_config = multi_car_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = multi_car_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = multi_car_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = multi_car_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = multi_car_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = multi_car_acados_get_nlp_opts(acados_ocp_capsule);

    // initial condition
    int idxbx0[NBX0];
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;
    idxbx0[7] = 7;
    idxbx0[8] = 8;
    idxbx0[9] = 9;
    idxbx0[10] = 10;
    idxbx0[11] = 11;

    double lbx0[NBX0];
    double ubx0[NBX0];
    lbx0[0] = -3;
    ubx0[0] = -3;
    lbx0[1] = 0;
    ubx0[1] = 0;
    lbx0[2] = 0;
    ubx0[2] = 0;
    lbx0[3] = 0;
    ubx0[3] = 0;
    lbx0[4] = 0;
    ubx0[4] = 0;
    lbx0[5] = 0;
    ubx0[5] = 0;
    lbx0[6] = 3;
    ubx0[6] = 3;
    lbx0[7] = 0;
    ubx0[7] = 0;
    lbx0[8] = 3.141592653589793;
    ubx0[8] = 3.141592653589793;
    lbx0[9] = 0;
    ubx0[9] = 0;
    lbx0[10] = 0;
    ubx0[10] = 0;
    lbx0[11] = 0;
    ubx0[11] = 0;

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
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;
    u0[3] = 0.0;
    u0[4] = 0.0;
    u0[5] = 0.0;
    // set parameters
    double p[NP];
    p[0] = [0];
    p[1] = [0];
    p[2] = [0];
    p[3] = [0];
    p[4] = [0];
    p[5] = [0];
    p[6] = [0];
    p[7] = [0];
    p[8] = [0];
    p[9] = [0];
    p[10] = [0];
    p[11] = [0];
    p[12] = [0];
    p[13] = [0];
    p[14] = [0];
    p[15] = [0];
    p[16] = [0];
    p[17] = [0];
    p[18] = [0];
    p[19] = [0];
    p[20] = [0];
    p[21] = [0];
    p[22] = [0];
    p[23] = [0];
    p[24] = [0];
    p[25] = [0];
    p[26] = [0];
    p[27] = [0];
    p[28] = [0];
    p[29] = [0];
    p[30] = [0];
    p[31] = [0];
    p[32] = [0];
    p[33] = [0];
    p[34] = [0];
    p[35] = [0];
    p[36] = [0];
    p[37] = [0];
    p[38] = [0];
    p[39] = [0];
    p[40] = [0];
    p[41] = [0];
    p[42] = [0];
    p[43] = [0];
    p[44] = [0];
    p[45] = [0];
    p[46] = [0];
    p[47] = [0];
    p[48] = [0];
    p[49] = [0];
    p[50] = [0];
    p[51] = [0];
    p[52] = [0];
    p[53] = [0];
    p[54] = [0];
    p[55] = [0];
    p[56] = [0];
    p[57] = [0];
    p[58] = [0];
    p[59] = [0];
    p[60] = [0];
    p[61] = [0];
    p[62] = [0];
    p[63] = [0];
    p[64] = [0];
    p[65] = [0];
    p[66] = [0];
    p[67] = [0];
    p[68] = [0];
    p[69] = [0];
    p[70] = [0];
    p[71] = [0];
    p[72] = [0];
    p[73] = [0];
    p[74] = [0];
    p[75] = [0];
    p[76] = [0];
    p[77] = [0];
    p[78] = [0];
    p[79] = [0];
    p[80] = [0];
    p[81] = [0];
    p[82] = [0];
    p[83] = [0];
    p[84] = [0];
    p[85] = [0];
    p[86] = [0];
    p[87] = [0];
    p[88] = [0];
    p[89] = [0];
    p[90] = [0];
    p[91] = [0];
    p[92] = [0];
    p[93] = [0];
    p[94] = [0];
    p[95] = [0];
    p[96] = [0];
    p[97] = [0];
    p[98] = [0];
    p[99] = [0];
    p[100] = [0];
    p[101] = [0];
    p[102] = [0];
    p[103] = [0];
    p[104] = [0];
    p[105] = [0];
    p[106] = [0];
    p[107] = [0];
    p[108] = [0];
    p[109] = [0];
    p[110] = [0];
    p[111] = [0];
    p[112] = [0];
    p[113] = [0];
    p[114] = [0];
    p[115] = [0];
    p[116] = [0];
    p[117] = [0];
    p[118] = [0];
    p[119] = [0];
    p[120] = [0];
    p[121] = [0];
    p[122] = [0];
    p[123] = [0];
    p[124] = [0];
    p[125] = [0];
    p[126] = [0];
    p[127] = [0];
    p[128] = [0];
    p[129] = [0];
    p[130] = [0];
    p[131] = [0];
    p[132] = [0];
    p[133] = [0];
    p[134] = [0];
    p[135] = [0];
    p[136] = [0];
    p[137] = [0];
    p[138] = [0];
    p[139] = [0];
    p[140] = [0];
    p[141] = [0];
    p[142] = [0];
    p[143] = [0];
    p[144] = [0];
    p[145] = [0];
    p[146] = [0];
    p[147] = [0];
    p[148] = [0];
    p[149] = [0];
    p[150] = [0];
    p[151] = [0];
    p[152] = [0];
    p[153] = [0];
    p[154] = [0];
    p[155] = [0];
    p[156] = [0];
    p[157] = [0];
    p[158] = [0];
    p[159] = [0];
    p[160] = [0];
    p[161] = [0];
    p[162] = [0];
    p[163] = [0];
    p[164] = [0];
    p[165] = [0];
    p[166] = [0];
    p[167] = [0];
    p[168] = [0];
    p[169] = [0];
    p[170] = [0];
    p[171] = [0];
    p[172] = [0];
    p[173] = [0];
    p[174] = [0];
    p[175] = [0];
    p[176] = [0];
    p[177] = [0];
    p[178] = [0];
    p[179] = [0];
    p[180] = [0];
    p[181] = [0];
    p[182] = [0];
    p[183] = [0];
    p[184] = [0];
    p[185] = [0];
    p[186] = [0];
    p[187] = [0];
    p[188] = [0];
    p[189] = [0];
    p[190] = [0];
    p[191] = [0];
    p[192] = [0];
    p[193] = [0];
    p[194] = [0];
    p[195] = [0];
    p[196] = [0];
    p[197] = [0];
    p[198] = [0];
    p[199] = [0];
    p[200] = [0];
    p[201] = [0];
    p[202] = [0];
    p[203] = [0];
    p[204] = [0];
    p[205] = [0];
    p[206] = [0];
    p[207] = [0];
    p[208] = [0];
    p[209] = [0];
    p[210] = [0];
    p[211] = [0];
    p[212] = [0];
    p[213] = [0];
    p[214] = [0];
    p[215] = [0];
    p[216] = [0];
    p[217] = [0];
    p[218] = [0];
    p[219] = [0];
    p[220] = [0];
    p[221] = [0];
    p[222] = [0];
    p[223] = [0];
    p[224] = [0];
    p[225] = [0];
    p[226] = [0];
    p[227] = [0];
    p[228] = [0];
    p[229] = [0];
    p[230] = [0];
    p[231] = [0];
    p[232] = [0];
    p[233] = [0];
    p[234] = [0];
    p[235] = [0];
    p[236] = [0];
    p[237] = [0];
    p[238] = [0];
    p[239] = [0];
    p[240] = [0];
    p[241] = [0];
    p[242] = [0];
    p[243] = [0];
    p[244] = [0];
    p[245] = [0];
    p[246] = [0];
    p[247] = [0];
    p[248] = [0];
    p[249] = [0];
    p[250] = [0];
    p[251] = [0];
    p[252] = [0];
    p[253] = [0];
    p[254] = [0];
    p[255] = [0];
    p[256] = [0];
    p[257] = [0];
    p[258] = [0];
    p[259] = [0];
    p[260] = [0];
    p[261] = [0];

    for (int ii = 0; ii <= N; ii++)
    {
        multi_car_acados_update_params(acados_ocp_capsule, ii, p, NP);
    }
  

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
        status = multi_car_acados_solve(acados_ocp_capsule);
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
        printf("multi_car_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("multi_car_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    multi_car_acados_print_stats(acados_ocp_capsule);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);

    // free solver
    status = multi_car_acados_free(acados_ocp_capsule);
    if (status) {
        printf("multi_car_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = multi_car_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("multi_car_acados_free_capsule() returned status %d. \n", status);
    }

    return status;
}
