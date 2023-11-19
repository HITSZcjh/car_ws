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
#include "acados_c/sim_interface.h"
#include "acados_sim_solver_car0_Arcsolver_1.h"

#define NX     CAR0_ARCSOLVER_1_NX
#define NZ     CAR0_ARCSOLVER_1_NZ
#define NU     CAR0_ARCSOLVER_1_NU
#define NP     CAR0_ARCSOLVER_1_NP


int main()
{
    int status = 0;
    sim_solver_capsule *capsule = car0_Arcsolver_1_acados_sim_solver_create_capsule();
    status = car0_Arcsolver_1_acados_sim_create(capsule);

    if (status)
    {
        printf("acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    sim_config *acados_sim_config = car0_Arcsolver_1_acados_get_sim_config(capsule);
    sim_in *acados_sim_in = car0_Arcsolver_1_acados_get_sim_in(capsule);
    sim_out *acados_sim_out = car0_Arcsolver_1_acados_get_sim_out(capsule);
    void *acados_sim_dims = car0_Arcsolver_1_acados_get_sim_dims(capsule);

    // initial condition
    double x_current[NX];
    x_current[0] = 0.0;
    x_current[1] = 0.0;
    x_current[2] = 0.0;
    x_current[3] = 0.0;
    x_current[4] = 0.0;
    x_current[5] = 0.0;
    x_current[6] = 0.0;
    x_current[7] = 0.0;
    x_current[8] = 0.0;
    x_current[9] = 0.0;
    x_current[10] = 0.0;
    x_current[11] = 0.0;
    x_current[12] = 0.0;
    x_current[13] = 0.0;
    x_current[14] = 0.0;
    x_current[15] = 0.0;
    x_current[16] = 0.0;
    x_current[17] = 0.0;
    x_current[18] = 0.0;
    x_current[19] = 0.0;
    x_current[20] = 0.0;
    x_current[21] = 0.0;
    x_current[22] = 0.0;
    x_current[23] = 0.0;
    x_current[24] = 0.0;
    x_current[25] = 0.0;
    x_current[26] = 0.0;
    x_current[27] = 0.0;
    x_current[28] = 0.0;
    x_current[29] = 0.0;
    x_current[30] = 0.0;
    x_current[31] = 0.0;
    x_current[32] = 0.0;
    x_current[33] = 0.0;
    x_current[34] = 0.0;
    x_current[35] = 0.0;
    x_current[36] = 0.0;
    x_current[37] = 0.0;
    x_current[38] = 0.0;
    x_current[39] = 0.0;
    x_current[40] = 0.0;
    x_current[41] = 0.0;
    x_current[42] = 0.0;
    x_current[43] = 0.0;
    x_current[44] = 0.0;
    x_current[45] = 0.0;
    x_current[46] = 0.0;
    x_current[47] = 0.0;
    x_current[48] = 0.0;
    x_current[49] = 0.0;
    x_current[50] = 0.0;
    x_current[51] = 0.0;
    x_current[52] = 0.0;
    x_current[53] = 0.0;
    x_current[54] = 0.0;
    x_current[55] = 0.0;
    x_current[56] = 0.0;
    x_current[57] = 0.0;
    x_current[58] = 0.0;
    x_current[59] = 0.0;

  
    printf("main_sim: initial state not defined, should be in lbx_0, using zero vector.");


    // initial value for control input
    double u0[NU];

    int n_sim_steps = 3;
    // solve ocp in loop
    for (int ii = 0; ii < n_sim_steps; ii++)
    {
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "x", x_current);
        status = car0_Arcsolver_1_acados_sim_solve(capsule);

        if (status != ACADOS_SUCCESS)
        {
            printf("acados_solve() failed with status %d.\n", status);
        }

        sim_out_get(acados_sim_config, acados_sim_dims,
               acados_sim_out, "x", x_current);
        
        printf("\nx_current, %d\n", ii);
        for (int jj = 0; jj < NX; jj++)
        {
            printf("%e\n", x_current[jj]);
        }
    }

    printf("\nPerformed %d simulation steps with acados integrator successfully.\n\n", n_sim_steps);

    // free solver
    status = car0_Arcsolver_1_acados_sim_free(capsule);
    if (status) {
        printf("car0_Arcsolver_1_acados_sim_free() returned status %d. \n", status);
    }

    car0_Arcsolver_1_acados_sim_solver_free_capsule(capsule);

    return status;
}
