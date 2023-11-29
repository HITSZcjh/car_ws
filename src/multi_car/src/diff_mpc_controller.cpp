#include "diff_mpc_controller.hpp"
#include <iostream>
#include <chrono>
#include "acados/utils/print.h"
namespace DiffMPC
{
    DiffMPCController::DiffMPCController(double vel_max, double vel_min, double omega_max, double omega_min, 
    double d_v_max, double d_v_min, double d_omega_max, double d_omega_min)
    {
        acados_ocp_capsule = diff_car_controller_acados_create_capsule();

        double *new_time_steps = NULL;
        int status = diff_car_controller_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);
        if (status)
        {
            printf("diff_car_controller_acados_create_with_discretization() returned status %d. Exiting.\n", status);
            exit(1);
        }

        nlp_config = diff_car_controller_acados_get_nlp_config(acados_ocp_capsule);
        nlp_dims = diff_car_controller_acados_get_nlp_dims(acados_ocp_capsule);
        nlp_in = diff_car_controller_acados_get_nlp_in(acados_ocp_capsule);
        nlp_out = diff_car_controller_acados_get_nlp_out(acados_ocp_capsule);
        nlp_solver = diff_car_controller_acados_get_nlp_solver(acados_ocp_capsule);
        nlp_opts = diff_car_controller_acados_get_nlp_opts(acados_ocp_capsule);

        // int idxbx0[NBX0];
        idxbx0[0] = 0;
        idxbx0[1] = 1;
        idxbx0[2] = 2;
        idxbx0[3] = 3;
        idxbx0[4] = 4;
        // double lbx0[NBX0];
        // double ubx0[NBX0];
        lbx0[0] = 0;
        ubx0[0] = 0;
        lbx0[1] = 0;
        ubx0[1] = 0;
        lbx0[2] = 0;
        ubx0[2] = 0;
        lbx0[3] = 0;
        ubx0[3] = 0;
        lbx0[4] = 0;
        ubx0[4] = 0;
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

        // double idxbx[NBX];
        idxbx[0] = 3;
        idxbx[1] = 4;
        // double lbx[NBX];
        // double ubx[NBX];
        lbx[0] = vel_min;
        ubx[0] = vel_max;
        lbx[1] = omega_min;
        ubx[1] = omega_max;
        for (int i = 1; i < N + 1; i++)
        {
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbx", idxbx);
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbx", lbx);
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubx", ubx);
        }

        // double idxbu[NBU];
        idxbu[0] = 0;
        idxbu[1] = 1;
        // double lbu[NBU];
        // double ubu[NBU];
        lbu[0] = d_v_min;
        ubu[0] = d_v_max;
        lbu[1] = d_omega_min;
        ubu[1] = d_omega_max;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
        }

        double x0[NX];
        x0[0] = 0.0;
        x0[1] = 0.0;
        x0[2] = 0.0;
        x0[3] = 0.0;
        x0[4] = 0.0;
        double u0[NU];
        u0[0] = 0.0;
        u0[1] = 0.0;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
        }
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0);

    }
    Vector2d DiffMPCController::Solve(double x0[NX], double yref[N][NX + NU], double yref_e[NX])
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x0);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x0);
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "y_ref", yref[i]);
        }
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "y_ref", yref_e);
        for(int i = 0;i<10;i++)
        {
            int rti_phase = 0;
            ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
            int acados_status = diff_car_controller_acados_solve(acados_ocp_capsule);

            rti_phase = 1;
            ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
            acados_status = diff_car_controller_acados_solve(acados_ocp_capsule);

            if (acados_status != 0)
            {
                printf("error in diff_car_controller_acados_solve()! Exiting.\n");
                exit(1);
            }
        }

        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", u);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        // std::cout << "MPC solve time: " << duration.count() << " microseconds" << std::endl;
        return Vector2d(u[0], u[1]);
    }
    DiffMPCController::~DiffMPCController()
    {
        int status;
        status = diff_car_controller_acados_free(acados_ocp_capsule);
        if (status)
        {
            printf("diff_car_controller_acados_free() returned status %d. \n", status);
        }
        status = diff_car_controller_acados_free_capsule(acados_ocp_capsule);
        if (status)
        {
            printf("diff_car_controller_acados_free_capsule() returned status %d. \n", status);
        }
    }
}

// int main()
// {
//     DiffMPC::DiffMPCController diff_mpc_controller;
//     double x0[DiffMPC::NX] = {0};
//     double yref[DiffMPC::N][DiffMPC::NX + DiffMPC::NU] = {0};
//     double yref_e[DiffMPC::NX] = {0};
//     Eigen::Vector2d u = diff_mpc_controller.Solve(x0, yref, yref_e);
//     std::cout << "u:" << u << std::endl;
//     return 0;
// }