//
// Created by Dongho Kang on 25.04.22.
//

#ifndef A5_RBSYMPLECTICENGINE_H
#define A5_RBSYMPLECTICENGINE_H

#include "sim/RBEngine.h"

namespace crl {

/**
 * simulation world implemented with symplectic integration scheme.
 */
class RBSymplecticEngine : public RBEngine {
public:
    RBSymplecticEngine() : RBEngine() {}

    ~RBSymplecticEngine() override = default;

    void step(double dt) override {
        // external force and torque
        updateForceForGravity();
        updateForceAndTorqueForSprings();

        // update states of rbs (integration)
        for (uint i = 0; i < rbs.size(); i++) {
            RB *rb = rbs[i];
            // retrieve saved force and tau
            V3D f = f_ext[i];
            V3D tau = tau_ext[i];

            // TODO: Ex.3 Stable Simulation
            // why our simulation is blown up? let's make it more stable using
            // symplectic Euler integration!

            // Note: Strictly the same as Explicit Euler
            V3D v_i = rb->state.velocity;
            V3D w_i = rb->state.angularVelocity;

            double m = rb->rbProps.mass;
            Matrix3x3 local_I = rb->rbProps.MOI_local;
            Matrix3x3 R = rb->state.orientation.matrix();
            Matrix3x3 I = R * local_I * R.transpose();
            //Could also define as R * local_I.inverse() * R^T, and only compute the inverse once...
            //but performance does not seem to be an issue so far
            Matrix3x3 I_inv = I.inverse();
            V3D I_times_w_i = (I * w_i).eval();

            rb->state.velocity = v_i + dt * f / m;
            rb->state.angularVelocity = w_i + dt * I_inv * (tau - w_i.cross(I_times_w_i));

            if (simulateCollisions && rb->rbProps.collision)
                updateVelocityAfterCollision(rb);

            // TODO: Ex.3 Stable Simulation
            // implement forward (explicit) Euler integration scheme for computing pose.
            // Note: Updating the position and orientation is different in symplectic Euler
            rb->state.pos = rb->state.pos + rb->state.velocity * dt; //Note: Use updated velocity
            rb->state.orientation = updateRotationGivenAngularVelocity(
                rb->state.orientation, rb->state.angularVelocity, dt); //Note: Use updated angular velocity
        }

        // clean up
        for (uint i = 0; i < f_ext.size(); i++) {
            f_ext[i] = V3D();
            tau_ext[i] = V3D();
        }
    }
};

}  // namespace crl

#endif  //A5_RBSYMPLECTICENGINE_H
