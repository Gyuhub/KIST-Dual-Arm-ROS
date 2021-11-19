#ifndef CUSTOMMATH_H
#define CUSTOMMATH_H

#define DEG2RAD (0.01745329251994329576923690768489)
#define RAD2DEG 1/DEG2RAD
#define GRAVITY 9.80665
#define PI 3.1415926535897932384626433

#include "math.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;


namespace CustomMath
{

// pseudo inverse
static MatrixXd pseudoInverseSVD(const MatrixXd& a, double epsilon = std::numeric_limits<double>::epsilon())
{
    JacobiSVD<MatrixXd> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);

    MatrixXd ainv(a.cols(), a.rows());
    ainv.noalias() = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();

    return ainv;
}

static MatrixXd pseudoInverseQR(const MatrixXd& A)
{
    CompleteOrthogonalDecomposition<MatrixXd> cod(A);
    return cod.pseudoInverse();
}

static MatrixXd OneSidedInverse(const MatrixXd& A) //no use
{
    MatrixXd Ainv(A.cols(), A.rows());

    if (A.cols() > A.rows()) //left inverse
    {
        MatrixXd AAT(A.rows(), A.rows());
        AAT = A * A.transpose();
        MatrixXd AATinv(A.rows(), A.rows());
        AATinv = AAT.inverse();

        Ainv = A.transpose() * AATinv;
    }
    else //right inverse
    {
        MatrixXd ATA(A.cols(), A.cols());
        ATA = A.transpose() * A;
        MatrixXd ATAinv(A.cols(), A.cols());
        ATAinv = ATA.inverse();
        Ainv = ATAinv * A.transpose();
    }

    return Ainv;
}

static MatrixXd WeightedPseudoInverse(const MatrixXd& Q, const MatrixXd& W, const bool Is_W_fullrank)
{
    MatrixXd invQ(Q.cols(), Q.rows());
    MatrixXd transposeQ(Q.cols(), Q.rows());
    MatrixXd Winv(W.cols(), W.rows());
    if (Is_W_fullrank == true)
    {
        Winv = W.inverse();
    }
    else
    {
        Winv = pseudoInverseQR(W);
        //Winv = pseudoInverseSVD(W, 0.0001);
    }
    transposeQ = Q.transpose();
    MatrixXd tmp1(W.cols(), Q.rows());
    tmp1.noalias() = Winv * transposeQ;
    MatrixXd tmp2(Q.rows(), W.rows());
    tmp2.noalias() = Q * Winv;
    MatrixXd tmp3(Q.rows(), Q.rows());
    tmp3.noalias() = tmp2 * transposeQ;
    MatrixXd tmp3inv(Q.rows(), Q.rows());
    tmp3inv = tmp3.inverse();
    invQ.noalias() = tmp1 * tmp3inv;
    //invQ = Winv*transposeQ*pseudoInverseQR(Q*Winv*transposeQ);
    //invQ = Winv*transposeQ*pseudoInverseSVD(Q*Winv*transposeQ);

    return invQ;
}

static MatrixXd DampedWeightedPseudoInverse(const MatrixXd& Q, const MatrixXd& W, const bool Is_W_fullrank)
{
    MatrixXd invQ(Q.cols(), Q.rows());
    MatrixXd transposeQ(Q.cols(), Q.rows());
    MatrixXd Winv(W.cols(), W.rows());
    if (Is_W_fullrank == true)
    {
        Winv = W.inverse();
    }
    else
    {
        Winv = pseudoInverseQR(W);
        //Winv = pseudoInverseSVD(W, 0.0001);
    }
    transposeQ = Q.transpose();
    double sigma = 0.01;
    MatrixXd Id(Q.rows(), Q.rows());
    Id.setIdentity();
    //invQ = Winv*transposeQ*pseudoInverseQR(Q*Winv*transposeQ + sigma*Id);
    MatrixXd tmp1(W.cols(), Q.rows());
    tmp1.noalias() = Winv * transposeQ;
    MatrixXd tmp2(Q.rows(), W.rows());
    tmp2.noalias() = Q * Winv;
    MatrixXd tmp3(Q.rows(), Q.rows());
    tmp3.noalias() = tmp2 * transposeQ;
    MatrixXd tmp4(Q.rows(), Q.rows());
    tmp4.noalias() = tmp3 + sigma * Id;
    MatrixXd tmp4inv(Q.rows(), Q.rows());
    //tmp4inv = pseudoInverseSVD(tmp4, 0.0001);
    tmp4inv = pseudoInverseQR(tmp4);

    invQ.noalias() = tmp1 * tmp4inv;

    return invQ;
}

// lowpass filter
static double VelLowpassFilter(double rT, //delT
    double rWn, //cutoff freq (rad/sec)
    double rQ_pre, //previous position
    double rQ, //current position
    double rVel_pre //previous filtered velocity
)
{
    double rA;
    double rB;
    double rC;
    double rD;

    double rVel;

    rA = 2.0 * rWn;
    rB = (-2.0) * rWn;
    rC = 2.0 + rT * rWn;
    rD = rT * rWn - 2.0;

    rVel = ((-(rD)) * rVel_pre + (rA)*rQ + (rB)*rQ_pre) / (rC);

    return(rVel);
}

static double LowPassFilter(double dT, double Wc, double X, double preY) //sampling time, cutoff freq, input, previous output
{
    double tau = 1.0 / Wc;
    double y = tau / (tau + dT) * preY + dT / (tau + dT) * X;
    return y;
}



static Eigen::Vector3d GetBodyRotationAngle(Eigen::Matrix3d RotMat)
{
    double rollangle, pitchangle, yawangle;
    Eigen::Vector3d BodyAngle;
    BodyAngle.setZero();

    //  pitchangle = atan2(-RotMat(2,0),sqrt(RotMat(0,0)*RotMat(0,0)+RotMat(1,0)*RotMat(1,0)));
    //  yawangle = atan2(RotMat(1,0)/cos(pitchangle),RotMat(0,0)/cos(pitchangle));
    //  rollangle = atan2(RotMat(2,1)/cos(pitchangle), RotMat(2,2)/cos(pitchangle));

    double threshold = 0.001;
    pitchangle = -asin(RotMat(2, 0));
    if (RotMat(2, 0) > 1.0 - threshold && RotMat(2, 0) < 1.0 + threshold) //when RotMat(2,0) == 1
    {//Gimbal lock, pitch = -90deg
        rollangle = atan2(-RotMat(0, 1), -RotMat(0, 2));
        yawangle = 0.0;
    }
    else if (RotMat(2, 0) < -1.0 + threshold && RotMat(2, 0) > -1.0 - threshold) //when RotMat(2,0) == -1
    {//Gimbal lock, pitch = 90deg
        rollangle = atan2(RotMat(0, 1), RotMat(0, 2));
        yawangle = 0.0;
    }
    else //general solution
    {
        rollangle = atan2(RotMat(2, 1), RotMat(2, 2));
        yawangle = atan2(RotMat(1, 0), RotMat(0, 0));
    }

    BodyAngle(0) = rollangle;
    BodyAngle(1) = pitchangle;
    BodyAngle(2) = yawangle;

    return BodyAngle;
}

static double GetBodyPitchAngle(Eigen::Matrix3d RotMat)
{
    double pitchangle;
    pitchangle = -asin(RotMat(2, 0));

    return pitchangle;
}

static double GetBodyRollAngle(Eigen::Matrix3d RotMat)
{
    double rollangle, pitchangle;
    double threshold = 0.001;
    pitchangle = -asin(RotMat(2, 0));
    if (RotMat(2, 0) > 1.0 - threshold && RotMat(2, 0) < 1.0 + threshold) //when RotMat(2,0) == 1
    {//Gimbal lock, pitch = -90deg
        rollangle = atan2(-RotMat(0, 1), -RotMat(0, 2));
    }
    else if (RotMat(2, 0) < -1.0 + threshold && RotMat(2, 0) > -1.0 - threshold) //when RotMat(2,0) == -1
    {//Gimbal lock, pitch = 90deg
        rollangle = atan2(RotMat(0, 1), RotMat(0, 2));
    }
    else //general solution
    {
        rollangle = atan2(RotMat(2, 1), RotMat(2, 2));
    }

    return rollangle;
}

static double GetBodyYawAngle(Eigen::Matrix3d RotMat)
{
    double pitchangle, yawangle;

    double threshold = 0.001;
    pitchangle = -asin(RotMat(2, 0));
    if (RotMat(2, 0) > 1.0 - threshold && RotMat(2, 0) < 1.0 + threshold) //when RotMat(2,0) == 1
    {//Gimbal lock, pitch = -90deg
        yawangle = 0.0;
    }
    else if (RotMat(2, 0) < -1.0 + threshold && RotMat(2, 0) > -1.0 - threshold) //when RotMat(2,0) == -1
    {//Gimbal lock, pitch = 90deg
        yawangle = 0.0;
    }
    else //general solution
    {
        yawangle = atan2(RotMat(1, 0), RotMat(0, 0));
    }

    return yawangle;
}

static Eigen::Matrix3d GetBodyRotationMatrix(double Roll, double Pitch, double Yaw)
{
    Eigen::Matrix3d R_yaw;
    R_yaw.setZero();
    //yawŽÂ zÃà¿¡ ŽëÇÑ ÈžÀü
    R_yaw(2, 2) = 1.0;
    R_yaw(0, 0) = cos(Yaw);
    R_yaw(0, 1) = -sin(Yaw);
    R_yaw(1, 0) = sin(Yaw);
    R_yaw(1, 1) = cos(Yaw);

    Eigen::Matrix3d R_pitch;
    R_pitch.setZero();
    //pitchŽÂ yÃà¿¡ ŽëÇÑ ÈžÀü
    R_pitch(1, 1) = 1.0;
    R_pitch(0, 0) = cos(Pitch);
    R_pitch(2, 2) = cos(Pitch);
    R_pitch(0, 2) = sin(Pitch);
    R_pitch(2, 0) = -sin(Pitch);

    Eigen::Matrix3d R_roll;
    R_roll.setZero();
    //rollÀº xÃà¿¡ ŽëÇÑ ÈžÀü
    R_roll(0, 0) = 1.0;
    R_roll(1, 1) = cos(Roll);
    R_roll(2, 2) = cos(Roll);
    R_roll(1, 2) = -sin(Roll);
    R_roll(2, 1) = sin(Roll);

    Eigen::Matrix3d RGyro;
    RGyro.noalias() = R_yaw * R_pitch * R_roll;
    //dMatrix RGyro_inv(3,3);
    //RGyro.inv(RGyro_inv);

    //return RGyro_inv;
    return RGyro;
}

static Eigen::Matrix3d GetBodyRotationMatrix(Vector3d Vec)
{
    double Roll = Vec(0);
    double Pitch = Vec(1);
    double Yaw = Vec(2);
    
    Eigen::Matrix3d R_yaw;
    R_yaw.setZero();
    //yawŽÂ zÃà¿¡ ŽëÇÑ ÈžÀü
    R_yaw(2, 2) = 1.0;
    R_yaw(0, 0) = cos(Yaw);
    R_yaw(0, 1) = -sin(Yaw);
    R_yaw(1, 0) = sin(Yaw);
    R_yaw(1, 1) = cos(Yaw);

    Eigen::Matrix3d R_pitch;
    R_pitch.setZero();
    //pitchŽÂ yÃà¿¡ ŽëÇÑ ÈžÀü
    R_pitch(1, 1) = 1.0;
    R_pitch(0, 0) = cos(Pitch);
    R_pitch(2, 2) = cos(Pitch);
    R_pitch(0, 2) = sin(Pitch);
    R_pitch(2, 0) = -sin(Pitch);

    Eigen::Matrix3d R_roll;
    R_roll.setZero();
    //rollÀº xÃà¿¡ ŽëÇÑ ÈžÀü
    R_roll(0, 0) = 1.0;
    R_roll(1, 1) = cos(Roll);
    R_roll(2, 2) = cos(Roll);
    R_roll(1, 2) = -sin(Roll);
    R_roll(2, 1) = sin(Roll);

    Eigen::Matrix3d RGyro;
    RGyro.noalias() = R_yaw * R_pitch * R_roll;
    //dMatrix RGyro_inv(3,3);
    //RGyro.inv(RGyro_inv);

    //return RGyro_inv;
    return RGyro;
}

static Eigen::Matrix3d rotateWithZ(double yaw_angle)
{
    Eigen::Matrix3d rotate_wth_z(3, 3);

    rotate_wth_z(0, 0) = cos(yaw_angle);
    rotate_wth_z(1, 0) = sin(yaw_angle);
    rotate_wth_z(2, 0) = 0.0;

    rotate_wth_z(0, 1) = -sin(yaw_angle);
    rotate_wth_z(1, 1) = cos(yaw_angle);
    rotate_wth_z(2, 1) = 0.0;

    rotate_wth_z(0, 2) = 0.0;
    rotate_wth_z(1, 2) = 0.0;
    rotate_wth_z(2, 2) = 1.0;

    return rotate_wth_z;
}

static Eigen::Matrix3d rotateWithY(double pitch_angle)
{
    Eigen::Matrix3d rotate_wth_y(3, 3);

    rotate_wth_y(0, 0) = cos(pitch_angle);
    rotate_wth_y(1, 0) = 0.0;
    rotate_wth_y(2, 0) = -sin(pitch_angle);

    rotate_wth_y(0, 1) = 0.0;
    rotate_wth_y(1, 1) = 1.0;
    rotate_wth_y(2, 1) = 0.0;

    rotate_wth_y(0, 2) = sin(pitch_angle);
    rotate_wth_y(1, 2) = 0.0;
    rotate_wth_y(2, 2) = cos(pitch_angle);

    return rotate_wth_y;
}

static Eigen::Matrix3d rotateWithX(double roll_angle)
{
    Eigen::Matrix3d rotate_wth_x(3, 3);

    rotate_wth_x(0, 0) = 1.0;
    rotate_wth_x(1, 0) = 0.0;
    rotate_wth_x(2, 0) = 0.0;

    rotate_wth_x(0, 1) = 0.0;
    rotate_wth_x(1, 1) = cos(roll_angle);
    rotate_wth_x(2, 1) = sin(roll_angle);

    rotate_wth_x(0, 2) = 0.0;
    rotate_wth_x(1, 2) = -sin(roll_angle);
    rotate_wth_x(2, 2) = cos(roll_angle);

    return rotate_wth_x;
}

static Eigen::Matrix3d skew(Eigen::Vector3d src)
{
    Eigen::Matrix3d skew;
    skew.setZero();
    skew(0, 1) = -src(2);
    skew(0, 2) = src(1);
    skew(1, 0) = src(2);
    skew(1, 2) = -src(0);
    skew(2, 0) = -src(1);
    skew(2, 1) = src(0);

    return skew;
}
/*
static Eigen::Vector3d getPhi(Eigen::Matrix3d current_rotation, Eigen::Matrix3d desired_rotation)
{
    Eigen::Vector3d phi;
    Eigen::Vector3d s[3], v[3], w[3];

    for (int i = 0; i < 3; i++)
    {
        v[i] = current_rotation.block<3, 1>(0, i);
        w[i] = desired_rotation.block<3, 1>(0, i);
        s[i] = v[i].cross(w[i]);
    }
    phi = s[0] + s[1] + s[2];
    phi = -0.5 * phi;

    return phi;
}
*/
static Eigen::Vector3d getPhi(Eigen::Matrix3d& RotationMtx, Eigen::Matrix3d& DesiredRotationMtx) //Orientation 구성
{
    //Get SkewSymmetric
    Eigen::Matrix3d s1_skew;
    Eigen::Matrix3d s2_skew;
    Eigen::Matrix3d s3_skew;

    Eigen::Vector3d RotMtxcol1;
    Eigen::Vector3d RotMtxcol2;
    Eigen::Vector3d RotMtxcol3;
    for (int i = 0; i < 3; i++)
    {
        RotMtxcol1(i) = RotationMtx(i, 0);
        RotMtxcol2(i) = RotationMtx(i, 1);
        RotMtxcol3(i) = RotationMtx(i, 2);
    }

    s1_skew = skew(RotMtxcol1);
    s2_skew = skew(RotMtxcol2);
    s3_skew = skew(RotMtxcol3);
    /////////////////////////////////////////////////////////////

    Eigen::Vector3d s1d;
    Eigen::Vector3d s2d;
    Eigen::Vector3d s3d;
    for (int i = 0; i < 3; i++)
    {
        s1d(i) = DesiredRotationMtx(i, 0);
        s2d(i) = DesiredRotationMtx(i, 1);
        s3d(i) = DesiredRotationMtx(i, 2);
    }

    Eigen::Vector3d s1f;
    Eigen::Vector3d s2f;
    Eigen::Vector3d s3f;

    s1f = s1_skew * s1d;
    s2f = s2_skew * s2d;
    s3f = s3_skew * s3d;
    /////////////////////////////////////////////////////////////
    //phi.resize(3);
    Eigen::Vector3d phi;
    phi = (s1f + s2f + s3f) * (-1.0 / 2.0);
    return phi;
}

static Eigen::Vector3d OrientationVelocity(Eigen::Matrix3d Rot, Eigen::Matrix3d Rotdot) //rotation matrix, derivative of rotation matrix
{
    Eigen::Matrix3d RdotRT;
    RdotRT = Rotdot * Rot.transpose();
    Eigen::Vector3d OriVel;
    OriVel(0) = RdotRT(2, 1);
    OriVel(1) = RdotRT(0, 2);
    OriVel(2) = RdotRT(1, 0);

    return OriVel;
}


static double Cubic(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f)
{
    double rx_q;

    if (rT < rT_0)
    {
        rx_q = rx_0;
    }
    else if (rT > rT_f)
    {
        rx_q = rx_f;
    }
    else {

        rx_q = rx_0 + rx_dot_0 * (rT - rT_0)
            + (3.0 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0)) - 2.0 * rx_dot_0 / (rT_f - rT_0) - rx_dot_f / (rT_f - rT_0)) * (rT - rT_0) * (rT - rT_0)
            + (-2.0 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0)) + (rx_dot_0 + rx_dot_f) / ((rT_f - rT_0) * (rT_f - rT_0))) * (rT - rT_0) * (rT - rT_0) * (rT - rT_0);
    }
    return (rx_q);
}

static double CubicDot(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f)
{
    double rx_q_dot;

    if (rT < rT_0)
    {
        rx_q_dot = rx_dot_0;
    }
    else if (rT > rT_f)
    {
        rx_q_dot = rx_dot_f;
    }
    else {
        rx_q_dot = rx_dot_0 + 2.0 * (3.0 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0)) - 2.0 * rx_dot_0 / (rT_f - rT_0) - rx_dot_f / (rT_f - rT_0)) * (rT - rT_0)
            + 3.0 * (-2.0 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0)) + (rx_dot_0 + rx_dot_f) / ((rT_f - rT_0) * (rT_f - rT_0))) * (rT - rT_0) * (rT - rT_0);
    }
    return (rx_q_dot);
}

static double Min(double val1, double val2)
{
    double min_val = 0.0;

    if (val1 >= val2)
    {
        min_val = val2;
    }
    else
    {
        min_val = val1;
    }

    return min_val;
}

static double Max(double val1, double val2)
{
    double max_val = 0.0;

    if (val1 >= val2)
    {
        max_val = val1;
    }
    else
    {
        max_val = val2;
    }

    return max_val;
}

static double SwitchFunction(double start, double end, double val)
{
    double alpha = 0.0;
    alpha = Cubic(val, start, end, 0.0, 0.0, 1.0, 0.0);
    return alpha;
}

static void ContactWrenchConeConstraintGenerate(double friction_coeff, double cop_x_b, double cop_y_b, double pushing_force_threshold, Eigen::MatrixXd &A, Eigen::VectorXd &lb, Eigen::VectorXd &ub) //A: constraint matrix
{
    //This function is for contact wrench cone constraint
    //based on Caron, Stéphane, Quang-Cuong Pham, and Yoshihiko Nakamura. "Stability of surface contacts for humanoid robots: Closed-form formulae of the contact wrench cone for rectangular support areas." 2015 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2015.
    // * z direction denotes normal direction of contact plane (negative direction = toward object or ground)
    // * pushing_force_thrreshold should be negative number
    // * coordinate has to locate center of the contact plane (cop_x_b = l_x/2, cop_y_b = l_y/2)
    // * friction_coeff, cop_x_b and cop_y_b should be positive number

    A.setZero(17, 6);
    lb.setZero(17);
    ub.setZero(17);

    double inf_val = 100000000.0; //large number to describe infinite

    // A matrix
    //CoP x
    A(0, 2) = -cop_x_b;
    A(0, 4) = -1.0;
    A(1, 2) = cop_x_b;
    A(1, 4) = -1.0;
    //CoP y
    A(2, 2) = -cop_y_b;
    A(2, 3) = 1.0;
    A(3, 2) = cop_y_b;
    A(3, 3) = 1.0;
    //Fx friction
    A(4, 0) = 1.0;
    A(4, 2) = -friction_coeff;
    A(5, 0) = 1.0;
    A(5, 2) = friction_coeff;
    //Fy friction
    A(6, 1) = 1.0;
    A(6, 2) = -friction_coeff;
    A(7, 1) = 1.0;
    A(7, 2) = friction_coeff;
    //Mz friction
    A(8, 0) = -cop_y_b;
    A(8, 1) = -cop_x_b;
    A(8, 2) = -friction_coeff * (cop_x_b + cop_y_b) / 2.0;
    A(8, 3) = friction_coeff;
    A(8, 4) = friction_coeff;
    A(8, 5) = 1.0;
    A(9, 0) = -cop_y_b;
    A(9, 1) = cop_x_b;
    A(9, 2) = -friction_coeff * (cop_x_b + cop_y_b) / 2.0;
    A(9, 3) = friction_coeff;
    A(9, 4) = -friction_coeff;
    A(9, 5) = 1.0;
    A(10, 0) = cop_y_b;
    A(10, 1) = -cop_x_b;
    A(10, 2) = -friction_coeff * (cop_x_b + cop_y_b) / 2.0;
    A(10, 3) = -friction_coeff;
    A(10, 4) = friction_coeff;
    A(10, 5) = 1.0;
    A(11, 0) = cop_y_b;
    A(11, 1) = cop_x_b;
    A(11, 2) = -friction_coeff * (cop_x_b + cop_y_b) / 2.0;
    A(11, 3) = -friction_coeff;
    A(11, 4) = -friction_coeff;
    A(11, 5) = 1.0;
    A(12, 0) = -cop_y_b;
    A(12, 1) = -cop_x_b;
    A(12, 2) = -friction_coeff * (cop_x_b + cop_y_b) / 2.0;
    A(12, 3) = -friction_coeff;
    A(12, 4) = -friction_coeff;
    A(12, 5) = -1.0;
    A(13, 0) = -cop_y_b;
    A(13, 1) = cop_x_b;
    A(13, 2) = -friction_coeff * (cop_x_b + cop_y_b) / 2.0;
    A(13, 3) = -friction_coeff;
    A(13, 4) = friction_coeff;
    A(13, 5) = -1.0;
    A(14, 0) = cop_y_b;
    A(14, 1) = -cop_x_b;
    A(14, 2) = -friction_coeff * (cop_x_b + cop_y_b) / 2.0;
    A(14, 3) = friction_coeff;
    A(14, 4) = -friction_coeff;
    A(14, 5) = -1.0;
    A(15, 0) = cop_y_b;
    A(15, 1) = cop_x_b;
    A(15, 2) = -friction_coeff * (cop_x_b + cop_y_b) / 2.0;
    A(15, 3) = friction_coeff;
    A(15, 4) = friction_coeff;
    A(15, 5) = -1.0;
    //Fz (pushing force)
    A(16, 2) = 1.0;

    // lb vector
    lb(1) = -inf_val;
    lb(3) = -inf_val;
    lb(5) = -inf_val;
    lb(7) = -inf_val;
    lb(16) = -inf_val;

    //ub vector
    ub(0) = inf_val;
    ub(2) = inf_val;
    ub(4) = inf_val;
    ub(6) = inf_val;
    ub(8) = inf_val;
    ub(9) = inf_val;
    ub(10) = inf_val;
    ub(11) = inf_val;
    ub(12) = inf_val;
    ub(13) = inf_val;
    ub(14) = inf_val;
    ub(15) = inf_val;
    ub(16) = pushing_force_threshold;
}

static Eigen::Quaterniond CalcVectorToQuaternion(Eigen::Vector3d Vec) //calculate quaternion from euler angle
{
    /*Eigen::Matrix3d Mat = GetBodyRotationMatrix(Vec(0), Vec(1), Vec(2));

    Eigen::AngleAxisd AA;
    AA = Mat;

    Eigen::Quaterniond Q;
	Q = AA;*/
    Eigen::Quaterniond Q;
    Q = AngleAxisd(Vec(2), Eigen::Vector3d::UnitZ())
        * AngleAxisd(Vec(1), Eigen::Vector3d::UnitY())
        * AngleAxisd(Vec(0), Eigen::Vector3d::UnitX());
    return Q;
}

static Eigen::Quaterniond CalcMatrixToQuaternion(Eigen::Matrix3d Mat)
{
    Eigen::AngleAxisd AA;
    AA = Mat;
    
    Eigen::Quaterniond Q;
    Q = AA;

    return Q;
}

static Eigen::Matrix3d CalcMatrixFromQuaternion(Eigen::Vector4d Quat) //calculate matrix form quaternion. If you want to use this method, you may change the type of quaternion from Quaterniond to Vector4d
{
    Vector4d x = Quat;
	Matrix3d theta_;
	theta_ << (x(3)*x(3) + x(0)*x(0) - x(1)*x(1) - x(2)*x(2)) , 2.0 * (x(0)*x(1) - x(2)*x(3))                   , 2.0 * (x(0)*x(2) + x(1)*x(3))
         	, 2.0 * (x(0)*x(1) + x(2)*x(3))                   , (x(3)*x(3) - x(0)*x(0) + x(1)*x(1) - x(2)*x(2)) , 2.0 * (x(1)*x(2) - x(0)*x(3))
         	, 2.0 * (x(0)*x(2) - x(1)*x(3))                   , 2.0 * (x(1)*x(2) + x(0)*x(3))                   , (x(3)*x(3) - x(0)*x(0) - x(1)*x(1) + x(2)*x(2));
	return theta_;
}

static Eigen::Vector3d CalcQuaternionToVector(Eigen::Quaterniond Quat) //calculate vector from quaternion
{
    Vector3d Vec;
    Vec = Quat.toRotationMatrix().eulerAngles(0, 1, 2);
    return Vec;
}

static Eigen::Matrix3d CalcQuaternionToMatrix(Eigen::Quaterniond Quat) //calculate matrix from quaternion
{
    Eigen::Matrix3d Mat;
    Mat = Quat.normalized().toRotationMatrix();
    return Mat;
}

static Eigen::Quaterniond MultiplicationTwoQuaternions(Eigen::Quaterniond lhs, Eigen::Quaterniond rhs)
{
    Eigen::Vector3d vec_lhs, vec_rhs;
    vec_lhs = lhs.vec();
    vec_rhs = rhs.vec();

    double w_lhs, w_rhs;
    w_lhs = lhs.w();
    w_rhs = rhs.w();

    Eigen::Quaterniond result;
    result.w() = w_lhs * w_rhs - vec_lhs.dot(vec_rhs);
    result.vec() = vec_lhs.cross(vec_rhs) + w_lhs * vec_rhs + w_rhs * vec_lhs;

    return result;
}

static Eigen::Matrix4d MapFromQuaternionTo4DMatrix(Quaterniond Quat)
{
    double x_0 = Quat.w();
    double x_1 = Quat.x();
    double x_2 = Quat.y();
    double x_3 = Quat.z();

    Eigen::Matrix4d quat_to_mat;
    quat_to_mat << x_0, x_1, x_2, x_3,
                  -x_1, x_0, x_3,-x_2,
                  -x_2,-x_3, x_0, x_1,
                  -x_3, x_2,-x_1, x_0;

    return quat_to_mat;
}

static Eigen::Matrix3d ConverseFromQuaternionToOGMatrix(Quaterniond Quat)
{
    double x_0 = Quat.w();
    double x_1 = Quat.x();
    double x_2 = Quat.y();
    double x_3 = Quat.z();

    Eigen::Matrix3d quat_to_mat;
    quat_to_mat(0, 0) = x_0 * x_0 + x_1 * x_1 - x_2 * x_2 - x_3 * x_3;
    quat_to_mat(0, 1) = 2.0 * (x_1 * x_2 - x_3 * x_0);
    quat_to_mat(0, 2) = 2.0 * (x_1 * x_3 + x_2 * x_0);
    quat_to_mat(1, 0) = 2.0 * (x_1 * x_2 + x_3 * x_0);
    quat_to_mat(1, 1) = x_0 * x_0 - x_1 * x_1 + x_2 * x_2 - x_3 * x_3;
    quat_to_mat(1, 2) = 2.0 * (x_2 * x_3 - x_1 * x_0);
    quat_to_mat(2, 0) = 2.0 * (x_1 * x_3 - x_2 * x_0);
    quat_to_mat(2, 1) = 2.0 * (x_2 * x_3 + x_1 * x_0);
    quat_to_mat(2, 2) = x_0 * x_0 - x_1 * x_1 - x_2 * x_2 + x_3 * x_3;

    return quat_to_mat;
}

static Eigen::MatrixXd CalcAngularVelFromBodyFixedQuaternion(Quaterniond Quat)
{
    double q_0 = Quat.w();
    double q_1 = Quat.x();
    double q_2 = Quat.y();
    double q_3 = Quat.z();

    Eigen::MatrixXd quat_to_mat;
    quat_to_mat.setZero(3, 4);
    quat_to_mat <<-q_1, q_0, q_3,-q_2,
                  -q_2,-q_3, q_0, q_1,
                  -q_3, q_2,-q_1, q_0;

    return quat_to_mat;
}

static Eigen::Quaterniond LogQuaternion(Eigen::Quaterniond Quat)
{
    Eigen::Vector3d quat_vec = Quat.vec();
    double quat_w = Quat.w();
    double abs_quat = Quat.norm();
    double abs_quat_vec = quat_vec.norm();

    Eigen::Vector3d u = quat_vec / abs_quat_vec;
    double theta = acos(quat_w / abs_quat);
    Eigen::Quaterniond quat_res;
    quat_res.vec() = u * theta;
    quat_res.w() = log(abs_quat);
    return quat_res;
}

static Eigen::Quaterniond ExpQuaternion(Eigen::Quaterniond Quat)
{
    Eigen::Vector3d quat_vec = Quat.vec();
    double quat_w = Quat.w();
    double norm_vec = quat_vec.norm();

    Eigen::Quaterniond quat_res;
    if (fabs(sin(norm_vec) > 0.0000001))
    {
        quat_res.vec() = exp(quat_w) * ((quat_vec / norm_vec) * sin(norm_vec));
    }
    else
    {
        quat_res.vec() = exp(quat_w) * quat_vec;
    }
    quat_res.w() = exp(quat_w) * cos(norm_vec);
    return quat_res;
}

static Eigen::Matrix3d rotationLog(Eigen::Matrix3d RotMat)
{
    Eigen::Matrix3d RotMatLog;
    double phi = acos((RotMat.trace() - 1.0) / 2.0);
    // double div_phi_with_pi = std::fmod(phi, (2.0 * PI));
    if (RotMat.trace() == -1)
    {
        // if (div_phi_with_pi == 0.0)
        // {
        //     RotMatLog = (1 / 2) * (RotMat - RotMat.transpose());
        // }
        // else
        // {
        RotMat(0, 0) = RotMat(0, 0) - 1.0;
        RotMat(1, 1) = RotMat(1, 1) - 1.0;
        RotMat(2, 2) = RotMat(2, 2) - 1.0;
        Vector3d zero_vec;
        zero_vec.setZero();
        Eigen::ComplexEigenSolver<Matrix3d> ces(RotMat);
        int idx = 0;
        for (int i = 0; i < 3; i++)
        {
            std::cout << "eigenvalue [" << i << "] : " << ces.eigenvalues().col(i) << std::endl;
            /*if (ces.eigenvalues().row(i).real().isConstant(1.0))
            {
                std::cout << "find eigenvalue of 1!!" << std::endl;
                idx = i;
            }*/
        }
        Vector3d v_hat;
        v_hat(0) = ces.eigenvectors()(0, idx).real();
        v_hat(1) = ces.eigenvectors()(1, idx).real();
        v_hat(2) = ces.eigenvectors()(2, idx).real();
        RotMatLog = PI * skew(v_hat);
        // }
    }
    else
    {
        RotMatLog = phi / (2 * sin(phi)) * (RotMat - RotMat.transpose());
        //std::cout << "RotMatLog : \n" << RotMatLog << std::endl;
    }
    if (phi == 0.0)
    {
        RotMatLog.setZero();
    }
    return RotMatLog;
}

static Eigen::Matrix3d rotationExp(Eigen::Vector3d RotVec)
{
    if (RotVec.norm() == 0.0)
    {
        return Eigen::Matrix3d::Identity(3, 3);
    }
    else
    {
        Eigen::Matrix3d RotMatExp, Id;
        Id.setIdentity(3, 3);
        double norm = RotVec.norm();
        RotMatExp = Id + (sin(norm) / norm) * skew(RotVec) + ((1 - cos(norm)) / (norm * norm)) * skew(RotVec).cwiseAbs2();
        //std::cout << "RotMatExp : \n" << RotMatExp << std::endl;
        return RotMatExp;
    }
}

// Kang, I. G., and F. C. Park.
// "Cubic spline algorithms for orientation interpolation."
// const static Eigen::Matrix3d rotationCubic(double time,
//                                            double time_0,
//                                            double time_f,
//                                            const Eigen::Matrix3d &rotation_0,
//                                            const Eigen::Matrix3d &rotation_f)
// {
//     if (time >= time_f)
//     {
//         return rotation_f;
//     }
//     else if (time < time_0)
//     {
//         return rotation_0;
//     }
//     double tau = Cubic(time, time_0, time_f, 0, 1, 0, 0);
//     Eigen::Matrix3d rot_scaler_skew;
//     rot_scaler_skew = rotationLog(rotation_0.transpose() * rotation_f);
//     std::cout << "rot_scaler_skew :" << std::endl << rot_scaler_skew << std::endl;
//     Eigen::Matrix3d result = rotation_0 * rotationExp((rot_scaler_skew * tau));
//     std::cout << "result :" << std::endl << result << std::endl;
//     return result;
// }
// const static Eigen::Matrix3d rotationCubic(double time, double time_0, double time_f,
//                                            const Eigen::Vector3d &w_0, const Eigen::Vector3d &a_0,
//                                            const Eigen::Matrix3d &rotation_0, const Eigen::Matrix3d &rotation_f)
// {
//     Eigen::Matrix3d r_skew;
//     r_skew = rotationLog(rotation_0.transpose() * rotation_f);
//     Eigen::Vector3d a, b, c, r;
//     double tau = (time - time_0) / (time_f - time_0);
//     r(0) = r_skew(2, 1);
//     r(1) = r_skew(0, 2);
//     r(2) = r_skew(1, 0);
//     c = w_0;
//     b = a_0 / 2;
//     a = r - b - c;
//     Eigen::Vector3d t, u, c_i, b_i, a_i;
//     Eigen::Matrix3d Id_3, rd;
//     Id_3.setIdentity();
//     double norm_r;
//     t = 3 * a + 2 * b + c;
//     u = 6 * a + 2 * b;
//     norm_r = r.norm();
//     c_i = (Id_3 - (1 - cos(norm_r)) / (norm_r * norm_r) * r_skew + (norm_r - sin(norm_r)) / (norm_r * norm_r * norm_r) * r_skew.cwiseAbs2()) * t;
//     b_i = (u - (r.transpose() * t) * r.cross(t) / (norm_r * norm_r * norm_r * norm_r) * (2.0 * cos(norm_r) + norm_r * sin(norm_r) - 2.0)
//                 - (1.0 - cos(norm_r)) / (norm_r * norm_r) * r.cross(u)
//                 + (r.transpose() * t) * r.cross(r.cross(t))  / (norm_r * norm_r * norm_r * norm_r * norm_r) * (3.0 * sin(norm_r) - norm_r * cos(norm_r) - 2.0 * norm_r)
//                 + (norm_r - sin(norm_r)) / (norm_r * norm_r * norm_r) * (t.cross(r.cross(t)) + r.cross(r.cross(u)))) / 2.0;
//     a_i = r - b_i - c_i;
//     rd = rotation_0 * rotationExp(a_i * (tau * tau * tau) + b_i * (tau * tau) + c_i * tau);
//     if (tau < 0)
//         return rotation_0;
//     if (tau > 1)
//         return rotation_f;
//     return rd; //3 * a * pow(tau, 2) + 2 * b * tau + c;
// }

const static Eigen::Vector3d rotationCubicDot(double time, double time_0, double time_f,
                                              const Eigen::Vector3d &w_0, const Eigen::Vector3d &a_0,
                                              const Eigen::Matrix3d &rotation_0, const Eigen::Matrix3d &rotation_f)
{
    Eigen::Matrix3d r_skew;
    r_skew = rotationLog(rotation_0.transpose() * rotation_f);
    Eigen::Vector3d a, b, c, r;
    double tau = (time - time_0) / (time_f - time_0);
    r(0) = r_skew(2, 1);
    r(1) = r_skew(0, 2);
    r(2) = r_skew(1, 0);
    c = w_0;
    b = a_0 / 2;
    a = r - b - c;
    Eigen::Vector3d rd;
    for (int i = 0; i < 3; i++)
    {
        rd(i) = CubicDot(time, time_0, time_f, 0, r(i), 0, 0);
    }
    rd = rotation_0 * rd;
    if (tau < 0)
        return w_0;
    if (tau > 1)
        return Eigen::Vector3d::Zero();
    return rd; //3 * a * pow(tau, 2) + 2 * b * tau + c;
}
}
#endif

