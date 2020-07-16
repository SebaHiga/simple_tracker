#pragma once

#include <Eigen/Dense>
#include <geometry.hpp>

// reference: https://www.kalmanfilter.net/multiSummary.html fuck http://ros-developer.com/2019/04/10/kalman-filter-explained-with-python-code-from-scratch/
// Everything is fitted for 2d tracking

class Kalman{
public:
    static const int    nx = 4,
                        nu = 2,
                        nz = 2, NUM_VARS = 1;

    //  Actual state
    Eigen::Matrix<double, nx, 1> Xn;
    // Output state
    Eigen::Matrix<double, nz, 1> Zn;
    // State transition matrix
    Eigen::Matrix<double, nx, nx> F;
    // Control matrix
    Eigen::Matrix<double, nx, nu> G;
    // Covariance matrix
    Eigen::Matrix<double, nx, nx> P;
    // Process covariance matrix
    Eigen::Matrix<double, nx, nx> Q;
    // Uncertainty matrix
    Eigen::Matrix<double, nz, nz> R;
    // Observation matrix
    Eigen::Matrix<double, nz, nx> H;
    // Kalman gain matrix
    Eigen::Matrix<double, nx, nz> K;
    // Identity matrix
    Eigen::Matrix<double, nx, nx> I;

    float dt;

    // Variable indexes
    static const int iX  = 0; // X position
    static const int iY  = 1; // Y position
    static const int idX = 2; // X velocity
    static const int idY = 3; // Y velocity

    Kalman(){
        // Prediction matrices
        F.setIdentity();
        G.setZero();
        Q.setIdentity();
        P.setIdentity();

        H <<    1, 0, 0, 0,
                0, 1, 0, 0;

        R <<    1, 0,
                0, 1;
    }

    Kalman(double iniX, double inidX, double iniY, double inidY){
        Xn(iX)  = iniX;
        Xn(idX) = inidX;
        Xn(iY)  = iniY;
        Xn(idY) = inidY;

        // Prediction matrices
        F = F.setIdentity();
        G = G.setZero();
        Q = Q.setIdentity();

        R = R.setIdentity();
        R *= 5;

        P = P.setIdentity();

        H = H.setZero();
        H(iX, iX) = 1;
        H(iY, iY) = 1;

        I = I.Identity();
    }

    void init(double iniX, double iniY, double dt){
        Xn(iX)  = iniX;
        Xn(idX) = 0;
        Xn(iY)  = iniY;
        Xn(idY) = 0;
        this->dt = dt;

        F(iX, idX) = dt;
        F(iY, idY) = dt;
        G(iX, iX) = 0.5 * dt * dt;
        G(iY, iY) = 0.5 * dt * dt;
        G(idX, iX) = dt;
        G(idY, iY) = dt;

        Q = G * 1 * G.transpose();
    }

    // // Prediction method for object tracking
    // void predict(){
    //     // Predict:   Xn+1 = F * Xn + G * Un
    //     Xn = F * Xn;

    // }

    // Prediction method for object tracking
    void predict(){
        // Predict:   Xn+1 = F * Xn + G * Un
        Xn = F * Xn;

        P  = F * P * F.transpose() + Q; 
    }

    void update(int x, int y){
        // Zn
        Zn <<   x,
                y;

        // K
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

        // Xn
        Xn = Xn + K * (Zn - H * Xn);

        // P
        Eigen::Matrix<double, nx, nx> I;
        I = I.Identity();
        // P = P * (I.Identity() - K * H);
        // P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();
        P = P * (I - K * H);
    }

    std::pair<double, double> getPosition(){
        return {Xn(iX), Xn(iY)};
    }
};