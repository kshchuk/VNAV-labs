/**
 * @file    deliverable_4.cpp
 * @brief   Rotation averaging using nonlinear optimization.
 */

#include <deliverable_5.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <random>
#include <vector>

using namespace std;
using namespace gtsam;

// This is just to generate random numbers.
std::random_device rd_{};
std::mt19937 gen_{rd_()};

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  // Note: this is not a ROS node.

  // Create the ground-truth rotation matrix.
  // Here a unit rotation.
  Rot3 gt_rot;

  // Simulate rotation measurements by corrupting the original matrix
  // with noise.
  static constexpr size_t N = 10;
  vector<Rot3> measurements(N);
  for (size_t i = 0; i < N; ++i) {
    static Eigen::Matrix<double, 3, 1> e;
    static constexpr double rot_std_dev = 1e-2;
    static std::normal_distribution<> d_rotation{0, rot_std_dev};
    e << d_rotation(gen_), d_rotation(gen_), d_rotation(gen_);
    measurements.at(i) = gt_rot.expmap(e);
    std::cout << measurements.at(i) << std::endl;
  }

  // TODO: Create a factor graph.
  // Insert code below:

  // Define the rotation observation model.
  Vector9 sigmas;
  sigmas << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2;
  noiseModel::Diagonal::shared_ptr noise = noiseModel::Diagonal::Sigmas(sigmas);

  // Matrix3 rot_sum = measurements.at(0).matrix();
  // for (int i = 1; i < measurements.size(); ++i) {
  //   rot_sum += measurements.at(i).matrix();
  // }

  // TODO: Now, first create your own Frobenius Norm Factor, either in this
  // file or in the deliverable_4.h header file. And use this new factor you
  // created to build the corresponding factor graph to find the actual rotation
  // matrix.
  // Insert code below:

  // TODO: Print your graph.
  // Insert code below:
    NonlinearFactorGraph graph;

  // Create the data structure to hold the initial estimate to the
  // solution Intentionally initialize the variables off from the ground truth.
  Values initial;
  Matrix3 perturbation_matrix;
  // Rotation along y of 0.785398 rads
  perturbation_matrix << 0.7071069, 0.0, 0.7071066, 0.0, 1.0, 0.0, -0.7071066, 0.0,
      0.7071069;
  initial.insert(1, measurements.at(0) * Rot3(perturbation_matrix));

  // TODO: You can also print your initial estimate, useful to compare against
  // the actual result.
  // Insert code below:

  // TODO: Optimize the graph and print results.
  // Insert code below:

  // TODO: Print your final result, the initial and final error.
  // Insert code below:

    for (const Rot3& R_i : measurements) {
        graph.add(FrobeniusNormFactor(1, R_i, noise));
    }
    graph.print("Factor graph:\n");

  initial.print("Initial estimate:\n");
    LevenbergMarquardtParams params;
    params.setVerbosity("ERROR");
    const Values result = LevenbergMarquardtOptimizer(graph, initial, params).optimize();
    result.print("Final result:\n");
    std::cout << "initial error = " << graph.error(initial) << std::endl;
    std::cout << "final error = " << graph.error(result) << std::endl;
    // Closed form for comparison: projection of the mean onto SO(3).
    Matrix3 M = Matrix3::Zero();
    for (const Rot3& R_i : measurements) M += R_i.matrix();
    Eigen::JacobiSVD<Matrix3> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Matrix3 S = Matrix3::Identity();
    S(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant();
    const Rot3 closed_form(svd.matrixU() * S * svd.matrixV().transpose());
    std::cout << "angle(GTSAM, closed form) = "
    << result.at<Rot3>(1).between(closed_form).axisAngle().second << " rad\n";

  return 0;
}
/* ************************************************************************* */
