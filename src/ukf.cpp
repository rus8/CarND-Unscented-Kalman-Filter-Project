#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    // initially set to false, set to true in first call of ProcessMeasurement
    is_initialized_ = false;

    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // size of the state vector
    n_x_ = 5;

    // size of the augumented state vector
    n_aug_ = n_x_ + 2;

    // number of sigma points
    n_sig_ = 2 * n_aug_ + 1;

    // initial state vector
    x_ = VectorXd(n_x_);

    // initial covariance matrix
    P_ = MatrixXd(n_x_, n_x_);

    // sigma points matrix
    Xsig_pred_ = MatrixXd(n_aug_, n_sig_);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 30;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 30;

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;

    lambda_ = 3 - n_aug_;

    R_lidar_ = MatrixXd(2, 2);
    R_lidar_ << std_laspx_ * std_laspx_, 0,
                0, std_laspy_ * std_laspy_;

    R_radar_ = MatrixXd(3, 3);
    R_radar_ << std_radr_ * std_radr_, 0, 0,
                0, std_radphi_ * std_radphi_, 0,
                0, 0, std_radrd_ * std_radrd_;

    weights_ = VectorXd(n_sig_);


}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
    TODO:
    Complete this function! Make sure you switch between lidar and radar
    measurements.
    */
    if(is_initialized_ != true){
        x_ << 0, 0, 0, 0, 0;
        P_.fill(0.0); // will be finally initialized according to the sensor_type_
        if(meas_package.sensor_type_ == MeasurementPackage::LASER){
            x_(0) = meas_package.raw_measurements_(0);
            x_(1) = meas_package.raw_measurements_(1);

            P_(0, 0) = std_laspx_;
            P_(1, 1) = std_laspy_;
            P_(2, 2) = 1;
            P_(3, 3) = 1;
            P_(4, 4) = 1;
        }
        else if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
            double ro = meas_package.raw_measurements_(0);
            double theta = meas_package.raw_measurements_(1);
            double dro = meas_package.raw_measurements_(2);
            x_(0) = ro * cos(theta);
            x_(1) = ro * sin(theta);
            double vx, vy;
            vx = dro * cos(theta);
            vy = dro * sin(theta);
            x_(2) = sqrt(vx * vx + vy * vy);

            P_(0, 0) = 1;
            P_(1, 1) = 1;
            P_(2, 2) = 1;
            P_(3, 3) = std_radphi_;
            P_(4, 4) = 1;
        }

        time_us_ = meas_package.timestamp_;

        weights_(0) = lambda_ / (lambda_ + n_aug_);
        for (int i = 1; i < n_sig_; i++) {
            weights_(i) = 0.5 / (n_aug_ + lambda_);
        }

        is_initialized_ = true;
    }

    double dt;
    dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;

    Prediction(dt);

    if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR){
        UpdateRadar(meas_package);
    }
    if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER){
        UpdateLidar(meas_package);
    }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    /**
    TODO:

    Complete this function! Estimate the object's location. Modify the state
    vector, x_. Predict sigma points, the state, and the state covariance matrix.
    */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Use lidar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the lidar NIS.
    */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Use radar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the radar NIS.
    */
}
