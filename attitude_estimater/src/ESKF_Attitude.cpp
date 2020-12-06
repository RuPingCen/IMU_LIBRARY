#include <iostream>
#include "math.h"
#include "ESKF_Attitude.h"
#include "Convert.h"


using namespace std;

namespace IMU
{

ESKF_Attitude::ESKF_Attitude(Vector_12 Covar_Mat, double dt)
{
    deltaT = dt;

    DetAng_noise = Covar_Mat.block<3, 1>(0, 0);
    DetAngVel_noise = Covar_Mat.block<3, 1>(3, 0);
    Acc_noise = Covar_Mat.block<3, 1>(6, 0);
    Mag_noise = Covar_Mat.block<3, 1>(9, 0);

    CovarMat_Q = Eigen::Matrix<double, 6, 6>::Zero();
    CovarMat_R = Eigen::Matrix<double, 6, 6>::Zero();
}

void ESKF_Attitude::Param_Change(Vector_12 Covar_Mat)
{
    if (isStopped())
    {
        DetAng_noise = Covar_Mat.block<3, 1>(0, 0);
        DetAngVel_noise = Covar_Mat.block<3, 1>(0, 0);
        Acc_noise = Covar_Mat.block<3, 1>(0, 0);
        Mag_noise = Covar_Mat.block<3, 1>(0, 0);

        // Initialize the covariances matrices Q and R

        CovarMat_Q.block<3, 3>(0, 0) = DetAng_noise.asDiagonal();
        CovarMat_Q.block<3, 3>(3, 3) = DetAngVel_noise.asDiagonal();
        CovarMat_R.block<3, 3>(0, 0) = Acc_noise.asDiagonal();
        CovarMat_R.block<3, 3>(3, 3) = Mag_noise.asDiagonal();
    }
}

void ESKF_Attitude::Init_Estimator()
{
    // Initialize the covariances matrices Q and R

    CovarMat_Q.block<3, 3>(0, 0) = DetAng_noise.asDiagonal();
    CovarMat_Q.block<3, 3>(3, 3) = DetAngVel_noise.asDiagonal();
    CovarMat_R.block<3, 3>(0, 0) = Acc_noise.asDiagonal();
    CovarMat_R.block<3, 3>(3, 3) = Mag_noise.asDiagonal();

    // Initialize the nominal state

    //Vector_3 x_state, y_state, z_state;
    //z_state = Cur_Measurement.block<3, 1>(3, 0);
    //y_state = z_state.cross(Cur_Measurement.block<3, 1>(6, 0));
    //x_state = y_state.cross(z_state);
    Eigen::Vector3d Acc0, Gyro0, Mag0;
    Gyro0 = Cur_Measurement.block<3, 1>(0, 0);
    Acc0 = Cur_Measurement.block<3, 1>(3, 0);
    Mag0 = Cur_Measurement.block<3, 1>(6, 0);

    double Pitch0 = asin(Acc0[0]/Acc0.norm());
    double Roll0 = atan2(-Acc0[1], -Acc0[2]);
    double Yaw0 = atan2(-Mag0[1] * cos(Roll0) + Mag0[2] * sin(Roll0), Mag0[0] * cos(Pitch0) + Mag0[1] * sin(Pitch0)*sin(Roll0) + Mag0[2] * sin(Pitch0)*cos(Roll0)) - 8.3*3.14166 / 180;

    Eigen::Quaterniond quat_temp = Euler_to_Quaternion(Eigen::Vector3d(Roll0, Pitch0, Yaw0));
    quat_temp.normalize();

    Vector_3 AngVel_temp = DetAngVel_noise;

    IMU_State state;
    state.Nominal_quat = quat_temp;
    state.Nominal_AngVel = AngVel_temp;

    // Initialize the error state

    Vector_3 detla_theta, detla_angVel;
    state.Error_theta = Vector_3::Zero();
    state.Error_AngVel = Vector_3::Zero();
    state.Error_Convar = Eigen::Matrix<double, 6, 6>::Zero();
    state.Error_Convar.block<3, 3>(0, 0) = 1e-5*Eigen::Matrix<double, 3, 3>::Identity();
    state.Error_Convar.block<3, 3>(3, 3) = 1e-7*Eigen::Matrix<double, 3, 3>::Identity();

    State_Vector.push_back(state);
    quaternion.push_back(quat_temp);

    Last_Measurement = Cur_Measurement;
}

void ESKF_Attitude::NominaState_Predict()
{
    Vector_3 delta_theta;
    Eigen::Quaterniond quat_temp;
    IMU_State Piror_State = State_Vector.back();
    delta_theta = (0.5*(Cur_Measurement.block<3, 1>(0, 0) + Last_Measurement.block<3, 1>(0, 0)) - Piror_State.Nominal_AngVel)*deltaT;

    quat_temp.w() = 1;
    quat_temp.vec() = 0.5*delta_theta;
    quat_temp = Piror_State.Nominal_quat*quat_temp;
    quat_temp.normalize();

    IMU_State Post_State;
    Post_State.Nominal_quat = quat_temp;
    Post_State.Nominal_AngVel = Piror_State.Nominal_AngVel;
    State_Vector.push_back(Post_State);
}

void ESKF_Attitude::ErrorState_Predict()
{
    IMU_State Post_State = State_Vector.back();
    State_Vector.pop_back();
    IMU_State Piror_State = State_Vector.back();

    // calculate the transition matrix A
    Eigen::Matrix<double, 6, 6> Trasition_A;
    Vector_3 delta_theta;
    delta_theta = (Last_Measurement.block<3, 1>(0, 0) - Piror_State.Nominal_AngVel)*deltaT;
    Matrix_3 Rotation_Mat = Euler_to_RoatMat(delta_theta);

    Trasition_A.block<3, 3>(0, 0) = Rotation_Mat.transpose();
    Trasition_A.block<3, 3>(0, 3) = Matrix_3::Identity()*(-deltaT);
    Trasition_A.block<3, 3>(3, 0) = Matrix_3::Zero();
    Trasition_A.block<3, 3>(3, 3) = Matrix_3::Identity();

    // priori prediction

    Vector_6 Errstate_temp;
    Errstate_temp.block<3, 1>(0, 0) = Piror_State.Error_theta;
    Errstate_temp.block<3, 1>(3, 0) = Piror_State.Error_AngVel;
    Errstate_temp = Trasition_A*Errstate_temp;

    Post_State.Error_theta = Errstate_temp.block<3, 1>(0, 0);
    Post_State.Error_AngVel = Errstate_temp.block<3, 1>(3, 0);

    Eigen::Matrix<double, 6, 6> CovarMat_Qi = CovarMat_Q*deltaT;
    Eigen::Matrix<double, 6, 6> NoiseMat_Fi = Eigen::Matrix<double, 6, 6>::Identity();

    Post_State.Error_Convar = Trasition_A*Piror_State.Error_Convar*Trasition_A.transpose() +
                              NoiseMat_Fi*CovarMat_Qi*NoiseMat_Fi.transpose();

    State_Vector.push_back(Post_State);
}

Eigen::Matrix<double, 6, 6> ESKF_Attitude::IMU_State::Cal_ObserveMat(Vector_9 measurenment, Vector_6 &residual)
{
    Vector_3 True_AccMea, True_MagMre, Mea_Mag;
    Eigen::Matrix<double, 4, 1> Mag_global;

    Eigen::Quaterniond q = Nominal_quat;

    Mea_Mag = measurenment.block<3, 1>(6, 0);
    Eigen::Quaterniond Mag_quat(0.0, Mea_Mag(0), Mea_Mag(1), Mea_Mag(2));
    Mag_quat = QuatMult(q, QuatMult(Mag_quat, q.conjugate()));

    Mag_global <<   0, sqrt(Mag_quat.x()*Mag_quat.x() + Mag_quat.y()*Mag_quat.y()),
            0, Mag_quat.z();

    // calculate the observe Matrix H

    Eigen::Matrix<double, 3, 4> acc_H, mag_H;
    acc_H << 2 * q.y(), -2 * q.z(), 2 * q.w(), -2 * q.x(),
            -2 * q.x(), -2 * q.w(), -2 * q.z(), -2 * q.y(),
            0, 4 * q.x(), 4 * q.y(), 0;

    mag_H << -2 * Mag_global(3)*q.y(), 2 * Mag_global(3)*q.z(),
            -4 * Mag_global(1)*q.y() - 2 * Mag_global(3)*q.w(), -4 * Mag_global(1)*q.z() + 2 * Mag_global(3)*q.x(),

            -2 * Mag_global(1)*q.z() + 2 * Mag_global(3)*q.x(), 2 * Mag_global(1)*q.y() + 2 * Mag_global(3)*q.w(),
            2 * Mag_global(1)*q.x() + 2 * Mag_global(3)*q.z(), -2 * Mag_global(1)*q.w() + 2 * Mag_global(3)*q.y(),

            2 * Mag_global(1)*q.y(), 2 * Mag_global(1)*q.z() - 4 * Mag_global(3)*q.x(),
            2 * Mag_global(1)*q.w() - 4 * Mag_global(3)*q.y(), 2 * Mag_global(1)*q.x();

    Eigen::Matrix<double, 6, 7> Observe_Hx;
    Observe_Hx.block<3, 4>(0, 0) = acc_H;
    Observe_Hx.block<3, 4>(3, 0) = mag_H;
    Observe_Hx.block<3, 3>(0, 4) = Matrix_3::Zero();
    Observe_Hx.block<3, 3>(3, 4) = Matrix_3::Zero();

    Eigen::Matrix<double, 7, 6> Observe_Xx;
    Eigen::Matrix<double, 4, 3> Matrix_Q;
    Matrix_Q << -q.x(), -q.y(), -q.z(),
            q.w(), -q.z(), q.y(),
            q.z(), q.w(), -q.x(),
            -q.y(), q.x(), q.w();
    Observe_Xx.block<4, 3>(0, 0) = 0.5*Matrix_Q;
    Observe_Xx.block<4, 3>(0, 3) = Eigen::Matrix<double, 4, 3>::Zero();
    Observe_Xx.block<3, 3>(4, 0) = Matrix_3::Zero();
    Observe_Xx.block<3, 3>(4, 3) = Matrix_3::Identity();

    Eigen::Matrix<double, 6, 6> Observe_Matrix;
    Observe_Matrix = Observe_Hx*Observe_Xx;

    // Calculate the true measurement
    // the true accelerometer measurement
    True_AccMea <<  -2 * (q.x()*q.z() - q.w()*q.y()),
            -2 * (q.w()*q.x() + q.y()*q.z()),
            -2 * (0.5 - q.x()*q.x() - q.y()*q.y());

    // the true magnetometer measurement
    True_MagMre <<  -(2 * Mag_global(1)*(0.5 - q.y()*q.y() - q.z()*q.z()) + 2 * Mag_global(3)*(q.x()*q.z() - q.w()*q.y())),
            -(2 * Mag_global(1)*(q.x()*q.y() - q.w()*q.z()) + 2 * Mag_global(3)*(q.w()*q.x() + q.y()*q.z())),
            -(2 * Mag_global(1)*(q.w()*q.y() + q.x()*q.z()) + 2 * Mag_global(3)*(0.5 - q.x()*q.x() - q.y()*q.y()));

    // calculate the residual
    residual.block<3, 1>(0, 0) = measurenment.block<3, 1>(3, 0) + True_AccMea;
    residual.block<3, 1>(3, 0) = measurenment.block<3, 1>(6, 0) + True_MagMre;

    return Observe_Matrix;
}

void ESKF_Attitude::Update_Filter()
{
    Vector_6 Residual;
    Eigen::Matrix<double, 6, 6> Observe_Matrix;
    IMU_State Post_State = State_Vector.back();
    State_Vector.pop_back();

    // calculate the observe matrix and the correction residual
    Observe_Matrix = Post_State.Cal_ObserveMat(Cur_Measurement, Residual);

    //Calculate the kalman gain
    Eigen::Matrix<double, 6, 6> Poste_Cov = Post_State.Error_Convar;
    Eigen::Matrix<double, 6, 6> Kalman_Gain;
    Kalman_Gain = Observe_Matrix*Poste_Cov*Observe_Matrix.transpose() + CovarMat_R;
    Kalman_Gain = Poste_Cov*Observe_Matrix.transpose()*Kalman_Gain.inverse();

    // update error state
    Vector_6 Post_ErrState = Kalman_Gain*Residual;
    Post_State.Error_theta = Post_ErrState.block<3, 1>(0, 0);
    Post_State.Error_AngVel = Post_ErrState.block<3, 1>(3, 0);

    Post_State.Error_Convar = Poste_Cov - Kalman_Gain*(Observe_Matrix*Poste_Cov*Observe_Matrix.transpose() +
                                                       CovarMat_R)*Kalman_Gain.transpose();

    State_Vector.push_back(Post_State);
}

void ESKF_Attitude::Update_NomianState()
{
    IMU_State Post_State = State_Vector.back();
    State_Vector.pop_back();

    Eigen::Quaterniond delta_q = BuildUpdateQuat(Post_State.Error_theta);
    Post_State.Nominal_quat = Post_State.Nominal_quat*delta_q;
    Post_State.Nominal_quat.normalize();

    Post_State.Nominal_AngVel = Post_State.Nominal_AngVel + Post_State.Error_AngVel;

    State_Vector.push_back(Post_State);
    quaternion.push_back(Post_State.Nominal_quat);
}

void ESKF_Attitude::Reset_ErrorState()
{
    IMU_State Post_State = State_Vector.back();
    State_Vector.pop_back();

    Eigen::Matrix<double, 6, 6> Matrix_G = Eigen::Matrix<double, 6, 6>::Identity();
    //Matrix_3 Ang_Mat = Post_State.Error_theta.asDiagonal();

    //Matrix_G.block<3, 3>(0, 0) = Matrix_3::Identity() - Ang_Mat;


    Post_State.Error_theta = Vector_3::Zero();
    Post_State.Error_AngVel = Vector_3::Zero();

    Post_State.Error_Convar = Matrix_G*Post_State.Error_Convar*Matrix_G.transpose();

    State_Vector.push_back(Post_State);

    Last_Measurement = Cur_Measurement;
}

void ESKF_Attitude::Read_SensorData(Vector_9 measurement)
{
    Vector_3 gyro_mea, acc_mea, mag_mea;
    acc_mea = measurement.block<3, 1>(0, 0);
    gyro_mea = measurement.block<3, 1>(3, 0);
    mag_mea = measurement.block<3, 1>(6, 0);

    acc_mea.normalize();
    mag_mea.normalize();

    Cur_Measurement.block<3, 1>(0, 0) = gyro_mea;
    Cur_Measurement.block<3, 1>(3, 0) = acc_mea;
    Cur_Measurement.block<3, 1>(6, 0) = mag_mea;
}

Eigen::Quaterniond ESKF_Attitude::Run(Vector_9 measurement)
{
    while (true)
    {

        Read_SensorData(measurement);

        if (State_Vector.size() == 0 || quaternion.size() == 0)
        {
            // Initialize the true state of the estimator

            Init_Estimator();
        }
        else
        {
            // Predict the nomial and error state

            NominaState_Predict();
            ErrorState_Predict();

            // Update the filter parametres

            Update_Filter();

            // Uptate the nominal state

            Update_NomianState();

            // Reset the error state

            Reset_ErrorState();
        }

        if (Stop())
        {
            while (isStopped())
            {
#ifdef _WIN32

                Sleep(3);
#else

                usleep(3000);
#endif

            }
        }
        return quaternion.back();
    }

}

void ESKF_Attitude::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
}

void ESKF_Attitude::RequestStart()
{
    unique_lock<mutex> lock(mMutexStop);
    if (mbStopped)
    {
        mbStopped = false;
        mbStopRequested = false;
    }

}

bool ESKF_Attitude::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if (mbStopRequested)
    {
        mbStopped = true;
        return true;
    }
    return false;
}

bool ESKF_Attitude::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

void ESKF_Attitude::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
    mbStopRequested = false;

    State_Vector.clear();
    quaternion.clear();

    cout << "EKF attitude release " << endl;
}

}// namespace IMU