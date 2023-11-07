#include "traj.hpp"

namespace Traj
{
    void CircleTrajectory::step(double t)
    {
        pos[0] = origin[0] + radius * sin(omega * t + theta);
        pos[1] = origin[1] + radius - radius * cos(omega * t + theta);
        vel[0] = omega * radius * cos(omega * t + theta);
        vel[1] = omega * radius * sin(omega * t + theta);
    }

    int PolyTrajectory::Factorial(int x)
    {
        int fac = 1;
        for (int i = x; i > 0; i--)
            fac = fac * i;
        return fac;
    }

    Eigen::VectorXd PolyTrajectory::timeAllocation(const Eigen::MatrixXd &Path)
    {
        Eigen::VectorXd t(Path.rows() - 1);

        /*

        STEP 1: Learn the "trapezoidal velocity" of "TIme Allocation" in L5, then finish this timeAllocation function

        variable declaration: max_vel, max_acc: max_vel = 1.0, max_acc = 1.0 in this homework, you can change these in the test.launch

        You need to return a variable "t" contains t allocation, which's type is VectorXd

        The t allocation is many relative timeline but not one common timeline

        */
        double mid_num1 = max_vel * max_vel / max_acc;
        double mid_num2 = 2.0 * max_vel / max_acc;
        for (int i = 0; i < t.size(); i++)
        {
            double length = (Path.row(i) - Path.row(i + 1)).norm();
            if (length < mid_num1)
            {
                t(i) = sqrt(length / max_acc);
            }
            else
            {
                t(i) = mid_num2 + (length - mid_num1) / max_vel;
            }
        }

        return t;
    }

    void PolyTrajectory::PolyQPGeneration()
    {
        // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
        Time = timeAllocation(Path);
        n_segment = Time.size();                                   // the number of segments
        PolyCoeff = Eigen::MatrixXd::Zero(n_segment, 3 * p_num1d); // position(x,y,z), so we need (3 * p_num1d) coefficients
        Eigen::VectorXd Px(p_num1d * n_segment), Py(p_num1d * n_segment), Pz(p_num1d * n_segment);

        /*   Produce Mapping Matrix A to the entire trajectory, A is a mapping matrix that maps polynomial coefficients to derivatives.   */

        Eigen::MatrixXd M = Eigen::MatrixXd::Zero(p_num1d * n_segment, p_num1d * n_segment);
        for (int i = 0; i < n_segment; i++)
        {
            int mid_num1 = i * p_num1d;
            for (int j = 0; j < d_order; j++)
            {
                M(mid_num1 + j, mid_num1 + j) = Factorial(j);
            }
            for (int j = 0; j < d_order; j++)
            {
                for (int k = j; k < p_num1d; k++)
                    M(mid_num1 + d_order + j, mid_num1 + k) = Factorial(k) / Factorial(k - j) * pow(Time(i), k - j);
            }
        }

        int df_size = 3 + (n_segment - 1) + 3;

        Eigen::MatrixXd C_T = Eigen::MatrixXd::Zero(p_num1d * n_segment, d_order * (n_segment + 1));
        C_T(0, 0) = 1;
        C_T(1, 1) = 1;
        C_T(2, 2) = 1;
        C_T(d_order * (2 * n_segment - 1), df_size - 3) = 1;
        C_T(d_order * (2 * n_segment - 1) + 1, df_size - 2) = 1;
        C_T(d_order * (2 * n_segment - 1) + 2, df_size - 1) = 1;
        for (int i = 3; i < d_order; i++)
        {
            C_T(i, df_size + (i - 2) - 1) = 1;
            C_T(d_order * (2 * n_segment - 1) + i, d_order * (n_segment + 1) - (d_order - 1 - 3) + (i - 3) - 1) = 1;
        }
        for (int i = 0; i < (n_segment - 1); i++)
        {
            C_T((2 * i + 1) * d_order, 3 + i) = 1;
            C_T((2 * i + 2) * d_order, 3 + i) = 1;
            for (int j = 1; j < d_order; j++)
            {
                C_T((2 * i + 1) * d_order + j, df_size + (d_order - 3) + i * (d_order - 1) + j - 1) = 1;
                C_T((2 * i + 2) * d_order + j, df_size + (d_order - 3) + i * (d_order - 1) + j - 1) = 1;
            }
        }

        Eigen::VectorXd mid_num1 = Eigen::VectorXd::Zero(p_order - d_order + 1);
        for (int i = 0; i < mid_num1.size(); i++)
            mid_num1(i) = Factorial(d_order + i) / Factorial(i);
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(p_num1d * n_segment, p_num1d * n_segment);
        for (int i = 0; i < n_segment; i++)
        {
            for (int j = 0; j < p_order - d_order + 1; j++)
                for (int k = 0; k < p_order - d_order + 1; k++)
                {
                    Q(j + d_order + i * p_num1d, k + d_order + i * p_num1d) = mid_num1(j) * mid_num1(k) * pow(Time(i), j + k + 1) / (j + k + 1);
                }
        }
        Eigen::MatrixXd M_inverse = M.inverse();
        Eigen::MatrixXd R = C_T.transpose() * M_inverse.transpose() * Q * M_inverse * C_T;

        Eigen::MatrixXd df(Path.rows() + 4, 3);
        df.row(0) = Path.row(0);
        df.row(1) = BoundVel.row(0);
        df.row(2) = BoundAcc.row(0);
        df.block(3, 0, Path.rows() - 1, 3) = Path.block(1, 0, Path.rows() - 1, 3);
        df.row(df.rows() - 2) = BoundVel.row(1);
        df.row(df.rows() - 1) = BoundAcc.row(1);

        Eigen::MatrixXd Rpp = R.block(df_size, df_size, R.rows() - df_size, R.cols() - df_size);
        Eigen::MatrixXd Rfp = R.block(0, df_size, df_size, R.cols() - df_size);

        Eigen::MatrixXd dp = -Rpp.inverse() * Rfp.transpose() * df;

        Eigen::MatrixXd dmid(df.rows() + dp.rows(), df.cols());
        dmid << df, dp;
        Eigen::MatrixXd temp = M_inverse * C_T * dmid;
        for (int i = 0; i < n_segment; i++)
        {
            PolyCoeff.row(i) << temp.block(i * p_num1d, 0, p_num1d, 1).transpose(), temp.block(i * p_num1d, 1, p_num1d, 1).transpose(), temp.block(i * p_num1d, 2, p_num1d, 1).transpose();
        }
    }

    void PolyTrajectory::TimeScaling(double alpha)
    {
        Eigen::VectorXd time_new = Eigen::VectorXd::Zero(Time.size());
        for (int i = 0; i < Time.size(); i++)
        {
            time_new(i) = Time(i) / alpha;
        }
        Time = time_new;

        for (int i = 0; i < PolyCoeff.rows(); i++)
        {
            for (int j = 0; j < p_num1d; j++)
            {
                PolyCoeff(i, j) = PolyCoeff(i, j) * pow(alpha, j);
                PolyCoeff(i, j + p_num1d) = PolyCoeff(i, j + p_num1d) * pow(alpha, j);
                PolyCoeff(i, j + 2 * p_num1d) = PolyCoeff(i, j + 2 * p_num1d) * pow(alpha, j);
            }
        }
    }

    void PolyTrajectory::step(double t)
    {
        int segment_idx = 0;
        while (t > Time(segment_idx))
        {
            t -= Time(segment_idx);
            segment_idx++;
        }
        segment_idx = std::min(segment_idx, n_segment - 1);
        Eigen::VectorXd time_vector = Eigen::VectorXd::Zero(p_num1d);
        for (int i = 0; i < p_num1d; i++)
        {
            time_vector(i) = pow(t, i);
        }
        pos[0] = PolyCoeff.row(segment_idx).segment(0, p_num1d) * time_vector;
        pos[1] = PolyCoeff.row(segment_idx).segment(p_num1d, p_num1d) * time_vector;
        pos[2] = PolyCoeff.row(segment_idx).segment(2 * p_num1d, p_num1d) * time_vector;

        for (int i = 0; i < p_num1d; i++)
        {
            time_vector(i) = i * pow(t, i - 1);
        }

        vel[0] = PolyCoeff.row(segment_idx).segment(0, p_num1d) * time_vector;
        vel[1] = PolyCoeff.row(segment_idx).segment(p_num1d, p_num1d) * time_vector;
        vel[2] = PolyCoeff.row(segment_idx).segment(2 * p_num1d, p_num1d) * time_vector;
    }

    void PolyTrajectory::sample2D(int num, Eigen::MatrixX2d& point_list)
    {
        double total_time = Time[0];
        double delta_t = total_time / (num-1);

        for(int i=0;i<num;i++)
        {
            Eigen::VectorXd time_vector = Eigen::VectorXd::Zero(p_num1d);
            for (int i = 0; i < p_num1d; i++)
            {
                time_vector(i) = pow(i*delta_t, i);
            }
            pos[0] = PolyCoeff.row(0).segment(0, p_num1d) * time_vector;
            pos[1] = PolyCoeff.row(0).segment(p_num1d, p_num1d) * time_vector;
            point_list.row(i) = pos.segment(0,2);
        }
    }
}