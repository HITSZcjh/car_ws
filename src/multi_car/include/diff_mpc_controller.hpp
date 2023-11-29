#pragma once
#include <Eigen/Eigen>
#include "acados_solver_diff_car_controller.h"
namespace DiffMPC
{
    using namespace Eigen;
    constexpr int N = DIFF_CAR_CONTROLLER_N;
    constexpr int NX = DIFF_CAR_CONTROLLER_NX;
    constexpr int NU = DIFF_CAR_CONTROLLER_NU;
    constexpr int NBX0 = DIFF_CAR_CONTROLLER_NBX0;
    constexpr int NBX = DIFF_CAR_CONTROLLER_NBX;
    constexpr int NBU = DIFF_CAR_CONTROLLER_NBU;
    class DiffMPCController
    {
    private:
        diff_car_controller_solver_capsule *acados_ocp_capsule;
        ocp_nlp_config *nlp_config;
        ocp_nlp_dims *nlp_dims;
        ocp_nlp_in *nlp_in;
        ocp_nlp_out *nlp_out;
        ocp_nlp_solver *nlp_solver;
        void *nlp_opts;
        // double x0[NX];
        // double u0[NU];
        int idxbx0[NBX0];
        double lbx0[NBX0];
        double ubx0[NBX0];
        int idxbx[NBX];
        double lbx[NBX];
        double ubx[NBX];
        int idxbu[NBU];
        double lbu[NBU];
        double ubu[NBU];
        /* data */
    public:
        double u[NU];
        DiffMPCController(double vel_max, double vel_min, double omega_max, double omega_min,
        double d_v_max, double d_v_min, double d_omega_max, double d_omega_min);
        Vector2d Solve(double x0[NX], double yref[N][NX+NU], double yref_e[NX]);
        ~DiffMPCController();
    };
    
} // namespace DiffMPC