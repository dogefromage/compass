#include "ahrs.h"

#include "config.h"
#include "logging.h"

// https://ahrs.readthedocs.io/en/latest/filters/madgwick.html#id5
Eigen::Vector3f madgwick_cost(const Eigen::Quaternionf& q, const Eigen::Vector3f& local_d,
                              const Eigen::Vector3f& body_s) {
    Eigen::Vector3f result = q.conjugate() * local_d - body_s;
    return result;
}

Eigen::Matrix<float, 3, 4> madgwick_cost_jac(const Eigen::Quaternionf& q,
                                             const Eigen::Vector3f& local_d) {
    const float qw = q.w();
    const float qx = q.x();
    const float qy = q.y();
    const float qz = q.z();

    const float dx = local_d.x();
    const float dy = local_d.y();
    const float dz = local_d.z();

    Eigen::Matrix<float, 3, 4> jacobian;

    jacobian << 2.0f * dy * qz - 2.0f * dz * qy, 2.0f * dy * qy + 2.0f * dz * qz,
        -4.0f * dx * qy + 2.0f * dy * qx - 2.0f * dz * qw,
        -4.0f * dx * qz + 2.0f * dy * qw + 2.0f * dz * qx,

        -2.0f * dx * qz + 2.0f * dz * qx, 2.0f * dx * qy - 4.0f * dy * qx + 2.0f * dz * qw,
        2.0f * dx * qx + 2.0f * dz * qz, -2.0f * dx * qw - 4.0f * dy * qz + 2.0f * dz * qy,

        2.0f * dx * qy - 2.0f * dy * qx, 2.0f * dx * qz - 2.0f * dy * qw - 4.0f * dz * qx,
        2.0f * dx * qw + 2.0f * dy * qz - 4.0f * dz * qy, 2.0f * dx * qx + 2.0f * dy * qy;

    return jacobian;
}

Eigen::Vector3f project_by_vector(const Eigen::Vector3f& v, const Eigen::Vector3f& n) {
    Eigen::Vector3f n_hat = n.normalized();
    return v - (v.dot(n_hat)) * n_hat;
}

void AHRS::update_madgwick(float dt, const Eigen::Vector3f& angular_vel,
                           const Eigen::Vector3f& body_acc, const Eigen::Vector3f& local_acc_goal,
                           const Eigen::Vector3f& body_field,
                           const Eigen::Vector3f& local_field_goal) {
    // integration step
    Eigen::Quaternionf omega(0.0, angular_vel.x(), angular_vel.y(), angular_vel.z());
    Eigen::Quaternionf att_omega = attitude * omega;
    Eigen::Vector4f q_dot =
        0.5 * Eigen::Vector4f(att_omega.w(), att_omega.x(), att_omega.y(), att_omega.z());

    float mag_weight = 4;
    float acc_weight = 1;
    float beta = 0.5;

    // gradient descent using accelerometer
    Eigen::Matrix<float, 3, 4> acc_jac = madgwick_cost_jac(attitude, local_acc_goal);
    Eigen::Vector3f acc_cost = madgwick_cost(attitude, local_acc_goal, body_acc);

    Eigen::Matrix<float, 3, 4> mag_jac = madgwick_cost_jac(attitude, local_field_goal);
    Eigen::Vector3f mag_cost = madgwick_cost(attitude, local_field_goal, body_field);

    Eigen::Matrix<float, 6, 4> unified_jac;
    unified_jac << acc_weight * acc_jac, mag_weight * mag_jac;
    Eigen::Matrix<float, 6, 1> unified_cost;
    unified_cost << acc_weight * acc_cost, mag_weight * mag_cost;

    Eigen::Vector4f grad = unified_jac.transpose() * unified_cost;
    float grad_norm = grad.norm();
    if (grad_norm == 0) {
        Serial.println("ERROR grad_norm is 0 at gradient descent");
        return;
    }
    grad /= grad_norm;

    Eigen::Vector4f q_last(attitude.w(), attitude.x(), attitude.y(), attitude.z());

    Eigen::Vector4f q_next = q_last + (q_dot - beta * grad) * dt;

    attitude = Eigen::Quaternionf(q_next[0], q_next[1], q_next[2], q_next[3]);
    attitude.normalize();

    last_update_micros = micros();
}

void AHRS::finalize_warmup(const Eigen::Vector3f& local_acc_goal,
                           const Eigen::Vector3f& local_field_goal) {
    Eigen::Vector3f local_acc_goal_hat = local_acc_goal.normalized();
    body_warmup_acc.normalize();
    body_warmup_field.normalize();

    Eigen::Quaternionf q1 = Eigen::Quaternionf::FromTwoVectors(body_warmup_acc, local_acc_goal_hat);

    Eigen::Vector3f field_prime = q1 * body_warmup_field;
    field_prime = project_by_vector(field_prime, local_acc_goal_hat);
    field_prime.normalize();

    Eigen::Vector3f local_field_goal_proj = project_by_vector(local_field_goal, local_acc_goal_hat);
    local_field_goal_proj.normalize();

    Eigen::Quaternionf q2 = Eigen::Quaternionf::FromTwoVectors(field_prime, local_field_goal);

    attitude = q2 * q1;
    state = AHRSState::AHRS_Running;
}

void AHRS::update_warmup(float dt, const Eigen::Vector3f& angular_vel,
                         const Eigen::Vector3f& body_acc, const Eigen::Vector3f& local_acc_goal,
                         const Eigen::Vector3f& body_field,
                         const Eigen::Vector3f& local_field_goal) {
    Eigen::Vector3f deltaTheta = angular_vel * dt;
    Eigen::Quaternionf delta_q(1, 0.5f * deltaTheta.x(), 0.5f * deltaTheta.y(),
                               0.5f * deltaTheta.z());
    delta_q.normalize();

    body_warmup_acc = delta_q.conjugate() * body_warmup_acc;
    body_warmup_field = delta_q.conjugate() * body_warmup_field;

    body_warmup_acc += body_acc;
    body_warmup_field += body_field;

    Serial.print("AHRS warmup ");
    Serial.println(warmup_cycles);

    warmup_cycles++;
    if (warmup_cycles >= AHRS_WARMUP_CYCLES) {
        finalize_warmup(local_acc_goal, local_field_goal);
    }
}

void AHRS::update(float dt, const Eigen::Vector3f& angular_vel, const Eigen::Vector3f& body_acc,
                  const Eigen::Vector3f& local_acc_goal, const Eigen::Vector3f& body_field,
                  const Eigen::Vector3f& local_field_goal) {
    switch (state) {
        case AHRSState::AHRS_None:
            warmup_cycles = 0;
            body_warmup_acc = Eigen::Vector3f::Zero();
            body_warmup_field = Eigen::Vector3f::Zero();
            state = AHRSState::AHRS_Warmup;
            break;
        case AHRSState::AHRS_Warmup:
            update_warmup(dt, angular_vel, body_acc, local_acc_goal, body_field, local_field_goal);
            break;
        case AHRSState::AHRS_Running:
            update_madgwick(dt, angular_vel, body_acc, local_acc_goal, body_field,
                            local_field_goal);
            break;
    }
}

void AHRS::randomize() {
    attitude *= Eigen::Quaternionf(Eigen::AngleAxisf(5 * attitude.x(), Eigen::Vector3f::UnitX()));
    attitude *= Eigen::Quaternionf(Eigen::AngleAxisf(-3 * attitude.y(), Eigen::Vector3f::UnitZ()));
    attitude.normalize();
}

Eigen::Vector3f AHRS::vector_body_to_local(Eigen::Vector3f& body_vec) {
    return attitude * body_vec;
}

Eigen::Vector3f AHRS::vector_local_to_body(Eigen::Vector3f& local_vec) {
    return attitude.conjugate() * local_vec;
}
