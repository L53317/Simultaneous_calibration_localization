
#pragma once
#include <ceres/ceres.h>
#include <ceres/rotation.h>

template <typename T> inline
void QuaternionInverse(const T q[4], T q_inverse[4])
{
    q_inverse[0] = q[0];
    q_inverse[1] = -q[1];
    q_inverse[2] = -q[2];
    q_inverse[3] = -q[3];
};

struct TError
{
    /* Factor of global constraints. */
    TError(double t_x, double t_y, double t_z, double var)
                  :t_x(t_x), t_y(t_y), t_z(t_z), var(var){}

    template <typename T>
    /* Overload () for members. */
    bool operator()(const T* tj, T* residuals) const
    {
        residuals[0] = (tj[0] - T(t_x)) / T(var);
        residuals[1] = (tj[1] - T(t_y)) / T(var);
        residuals[2] = (tj[2] - T(t_z)) / T(var);

        return true;
    }

    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z, const double var)
    { /* <type, residual dimension(res_d), input_d> */
      return (new ceres::AutoDiffCostFunction<TError, 3, 3> (new TError(t_x, t_y, t_z, var)));
    }

    double t_x, t_y, t_z, var;
};

struct RelativeRTError
{
    /* Factor from one pose to next pose, between two poses. */
    RelativeRTError(double t_x, double t_y, double t_z,
                    double q_w, double q_x, double q_y, double q_z,
                    double t_var, double q_var)
                  :t_x(t_x), t_y(t_y), t_z(t_z), 
                   q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
                   t_var(t_var), q_var(q_var){}

    template <typename T>
    bool operator()(const T* const w_q_i, const T* ti, const T* w_q_j, const T* tj, T* residuals) const
    {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];

        T i_q_w[4];
        QuaternionInverse(w_q_i, i_q_w);

        T t_i_ij[3];
        ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);

        residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_var);
        residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_var);
        residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_var);

        T relative_q[4];
        relative_q[0] = T(q_w);
        relative_q[1] = T(q_x);
        relative_q[2] = T(q_y);
        relative_q[3] = T(q_z);

        T q_i_j[4];
        ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j);

        T relative_q_inv[4];
        QuaternionInverse(relative_q, relative_q_inv);

        T error_q[4];
        ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q);

        residuals[3] = T(2) * error_q[1] / T(q_var);
        residuals[4] = T(2) * error_q[2] / T(q_var);
        residuals[5] = T(2) * error_q[3] / T(q_var);

        return true;
    }

    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double q_w, const double q_x, const double q_y, const double q_z,
                                       const double t_var, const double q_var) 
    {
      /* <type, residual dimension(res_d), input_q_d, input_t_d, input_q_d, input_t_d> */
      return (new ceres::AutoDiffCostFunction< RelativeRTError, 6, 4, 3, 4, 3>
                (new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var, q_var)));
    }

    double t_x, t_y, t_z, t_norm;
    double q_w, q_x, q_y, q_z;
    double t_var, q_var;
};

struct DError
{
    /* Factor of global constraints.
       t_x, t_y, t_z, distance, var: the measurments (or observation with measurment)
     */
    DError(double t_x, double t_y, double t_z, double distance, double var)
                  :t_x(t_x), t_y(t_y), t_z(t_z), distance(distance), var(var){}

    template <typename T>
    /* Overload () for members.
       tj: the (first) optimization variable array;
       residuals: the output (difference).
     */
    bool operator()(const T* tj, T* residuals) const
    {
        // residuals[0] = (tj[0] - T(t_x)) / T(var);
        // residuals[1] = (tj[1] - T(t_y)) / T(var);
        // residuals[2] = (tj[2] - T(t_z)) / T(var);
        residuals[0] = (sqrt((tj[0] - T(t_x)) * (tj[0] - T(t_x)) +
                             (tj[1] - T(t_y)) * (tj[1] - T(t_y)) +
                             (tj[2] - T(t_z)) * (tj[2] - T(t_z))) - T(distance)) / T(var);
        /* Must be manifold? Or may not converge. */

        return true;
    }

    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double distance, const double var)
    {
      /* input parameters x: {tj[0],tj[1],tj[2]}; not {tj[0],tj[1],tj[2], distance} */
      return (new ceres::AutoDiffCostFunction<DError, 1, 3> (new DError(t_x, t_y, t_z, distance, var)));
    }

    double t_x, t_y, t_z, distance, var;
};

struct PError
{
    /* Factor of global constraints. */
    PError(double var) : var(var){}

    template <typename T>
    /* Overload () for members. */
    bool operator()(const T* tj, const T* tj1, T* residuals) const
    {
        residuals[0] = (tj[0] - tj1[0]) / T(var);
        residuals[1] = (tj[1] - tj1[1]) / T(var);
        residuals[2] = (tj[2] - tj1[2]) / T(var);

        return true;
    }

    static ceres::CostFunction* Create(const double var)
    { /* <type, residual dimension(res_d), input1_d, input2_d> */
      return (new ceres::AutoDiffCostFunction<PError, 3, 3, 3> (new PError(var)));
    }

    double var;
};