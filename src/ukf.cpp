#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  is_initialized_ = false;

  // L8:17 Augumentation Assignment
  n_x_ = 5; // state dimension
  n_aug_ = 7; // augumented state dimension
  lambda_ = 3 - n_aug_; // sigma point spreading parameter
  n_z = 3;
  aug_size = 2 * n_aug_ + 1;
  // NIS
  NIS_radar_ = 0;
  NIS_laser_ = 0;  

  // 5x5 Initial covariance matrix
  P_ = MatrixXd::Identity(n_x_, n_x_);
  P_ << 1, 0, 0, 0, 0,
      0, 1, 0, 0, 0,
      0, 0, 10, 0, 0,
      0, 0, 0, 50, 0,
      0, 0, 0, 0, 3; // Better results than Identity matrix

  Xsig_pred_ = MatrixXd::Zero(n_x_, aug_size);
  time_us_ = 0;
  weights_ = VectorXd::Zero(aug_size); // Sigma point weights

  H_ = MatrixXd(2, 5);
  H_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;
  R_lidar = MatrixXd(n_z-1, n_z-1);
  R_lidar << std_laspx_ * std_laspx_, 0,
      0, std_laspy_ * std_laspy_;
  R_radar = MatrixXd(n_z, n_z);
  R_radar << std_radr_ * std_radr_, 0, 0,
          0, std_radphi_ * std_radphi_, 0,
          0, 0, std_radrd_ * std_radrd_;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_)
  {
    /**
      * Initialize the state x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    x_ = VectorXd::Zero(5);

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float x = meas_package.raw_measurements_(0);
      float y = meas_package.raw_measurements_(1);
      float rho_dot = meas_package.raw_measurements_(2);
      float rho = sqrt(x*x + y*y);
      float theta = atan2(y, x);

      x_(0) = rho*cos(theta);
      x_(1) = rho*sin(theta);
      x_(2) = rho_dot;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Control structure (similar to EKF)
   ****************************************************************************/

  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0; // dt in seconds
  time_us_ = meas_package.timestamp_;
  Prediction(dt);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    UpdateRadar(meas_package);
  }
  else
  {
    UpdateLidar(meas_package);
  }

  // print the output
  /*std::cout << "x_ = " << x_ << std::endl;
  std::cout << "P_ = " << P_ << std::endl;
  std::cout << "NIS_radar_ = " << NIS_radar_ << " NIS_laser_ = " << NIS_laser_ << std::endl;*/
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  /*****************************************************************************
   *  Create Augmented Sigma Points
   ****************************************************************************/
  // from L7:17 Augmentation Assignment 1

  VectorXd x_aug = VectorXd::Zero(n_aug_);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
 
  //create augmented covariance matrix
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, aug_size);
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  /*****************************************************************************
   *  Predic Sigma Points
   ****************************************************************************/
  // from L7:20 Sigma Point Prediction Assignment 1

  for (int i = 0; i < aug_size; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    if (fabs(yawd) > 0.001) //avoid division by zero
    {
      px_p = p_x + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t));
    }
    else
    {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  /*****************************************************************************
   *  Predic mean and covariance
   ****************************************************************************/
  // from L7:24 Prediction Mean and Covariance Assignment 1

  // set weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;

  for (int i=1; i < aug_size; i++) //2n+1 weights
  {
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < aug_size; i++)
  {
    //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < aug_size; i++) //iterate over sigma points
  {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    NormalizeAngle(&x_diff(3));

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  /*****************************************************************************
   *  Update Lidar Measurements
   ****************************************************************************/
  //Lesson L6:12 Laser Measurements for regular Kalman filter

  VectorXd z = meas_package.raw_measurements_;
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_lidar;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  NIS_laser_ = y.transpose() * Si * y;  
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  /*****************************************************************************
   *  Predict Radar Sigma Points
   ****************************************************************************/
  // L7:26 Predict Radar Measurement

  MatrixXd Zsig = MatrixXd::Zero(n_z, aug_size);
  //transform sigma points into measurement space
  for (int i = 0; i < aug_size; i++) // 2n+1 simga points
  {
    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model r, phi, r_dot
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);
    Zsig(1,i) = atan2(p_y,p_x);
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);
  }

  //calculate mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z);
  for (int i=0; i < aug_size; i++)
  {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //calculate innovation covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z,n_z);
  for (int i = 0; i < aug_size; i++) // 2n+1 simga points
  {
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    NormalizeAngle(&z_diff(1));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix

  S = S + R_radar;

  /*****************************************************************************
   *  Update Radar
   ****************************************************************************/
  // L7:29 UKF Update

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);
  //calculate cross correlation matrix
  for (int i = 0; i < aug_size; i++) //2n+1 simga points
  {
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    NormalizeAngle(&z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    NormalizeAngle(&x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  VectorXd z = meas_package.raw_measurements_;
  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  NormalizeAngle(&z_diff(1));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  MatrixXd Si = S.inverse();

  NIS_radar_ = z_diff.transpose() * Si * z_diff;
}

/**
 * Centralize the Angle normalization.
 * @param {pValue} points to the angle
 */
void UKF::NormalizeAngle(double *pValue)
{
  while (*pValue > M_PI) *pValue -= 2. * M_PI;
  while (*pValue <-M_PI) *pValue += 2. * M_PI;
}