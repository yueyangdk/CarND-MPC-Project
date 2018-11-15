#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

#define NUMBER_OF_STEPS 20 //time step length N 
#define dt 0.1 // time step duration dt in s 

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
#define Lf 2.67

// The reference velocity is set to 70 mph.

#define v_ref 70

// Set weights parameters(penalty factor) for the cost function
#define w_cte 8.4
#define w_epsi 0.32
#define w_velocity 0.261
#define w_delta 600000
#define w_accelerate 17.1
#define w_ddelta 0.01
#define w_da 0.00001

// Set lower and upper limits for variables.
#define Steer_bound 0.436332 // 25 deg in rad, used as delta bound
#define Accel_bound 1.0 // Maximum acceleration value
#define Other_bound 1.0e3 // Bound value for other variables

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
