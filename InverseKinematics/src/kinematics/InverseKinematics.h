#include "ForwardKinematics.h"

#include <ObjectiveFunction.h>
#include <NewtonFunctionMinimizer.h>

class InverseKinematics : public ObjectiveFunction
{
public:
    const Linkage *linkage; // pointer to linkae. (const Linkage &linkage_ = *linkage;)
    const Vector2d *target; // the end-effector target position
public:

    InverseKinematics(const Linkage &linkage, const Vector2d &target)
        : linkage(&linkage), target(&target){

    }

    // Return the objective function value f(x) corresponding to the IK problem
    // Given `x` which are the two angles.
    // This will be the scalar function that we want to minimize to solve IK.
    double evaluate(const VectorXd& x) const override {
        // 3 - Inverse Kinematics, Task 1
        return (endEffectorPosition(*linkage, x) - *target).squaredNorm()/2;
    }

    // Compute the gradient of the objective function, df/dx.
    // Given `x` which are the two angles.
    VectorXd gradient(const VectorXd& x) const override {
        // 3 - Inverse Kinematics, Task 2
        return dendEffector_dangles(*linkage, x).transpose() * (endEffectorPosition(*linkage, x) - *target);
    }

    // Compute the Hessian of the objective function, d^2f/dx^2
    // Given `x` which are the two angles.
    /*Matrix2d hessian(const VectorXd &x) const {
        Matrix2d hess = Matrix2d::Zero();
        // 3 - Inverse Kinematics, Task 3
        Tensor2x2x2 T = ddendEffector_ddangles(*linkage, x);
        Vector2d v = endEffectorPosition(*linkage, x) - *target;
        Vector2d d0 = dendEffector_dangles(*linkage, x).col(0);
        Vector2d d1 = dendEffector_dangles(*linkage, x).col(1);
        hess(0, 0) = T[0].col(0).dot(v) + d0.dot(d0);
        hess(1, 1) = T[1].col(1).dot(v) + d1.dot(d1);
        hess(0, 1) = hess(1, 0) = T[0].col(1).dot(v) + d0.dot(d1);
        return hess;
    }*/

    // Compute the Hessian of the objective function, d^2f/dx^2
    // Given `x` which are the two angles.
    Matrix2d hessian(const VectorXd &x) const {
        Matrix2d hess = Matrix2d::Zero();
        // 3 - Inverse Kinematics, Task 3
        Tensor2x2x2 T = ddendEffector_ddangles(*linkage, x);
        Matrix2d A = dendEffector_dangles(*linkage, x);
        Vector2d v = endEffectorPosition(*linkage, x) - *target;
        hess.row(0) = T[0].transpose() * v;
        hess.row(1) = T[1].transpose() * v;
        //TODO: This is all great and all but can't we just multiply the matrices directly?
        /*return A.transpose() * A + T * v;*/
        return hess + A.transpose() * A;
    }

    // prepares the dense matrix from `hessian(...)` to be added to a sparse matrix.
    // You can ignore this piece of code.
    void addHessianEntriesTo(const VectorXd& x, std::vector<Triplet<double>>& hessianEntries) const override {
        auto hess = hessian(x);
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                hessianEntries.push_back(Triplet<double>(i, j, hess(i,j)));
            }
        }
    }

};

// Return the angles computed with IK for the end-effector to reach the target.
// Given the `linkage`, the end-effector target position `target`,
//       the current angles `anglesCurrent` and a minimization method `method`.
Vector2d inverseKinematics(const Linkage &linkage, const Vector2d &target,
                           const Vector2d &anglesCurrent, MinimizationMethod *method) {

    InverseKinematics objective(linkage, target);
    VectorXd angles = anglesCurrent;
    method->minimize(&objective, angles);

    return angles;

}
