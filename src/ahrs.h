#pragma once

#include <ArduinoEigenDense.h>

Eigen::Vector3f project_by_vector(const Eigen::Vector3f& v, const Eigen::Vector3f& n);

enum class AHRSState {
    AHRS_None,
    AHRS_Warmup,
    AHRS_Running,
};

class AHRS {
   public:
    Eigen::Quaternionf attitude = Eigen::Quaternionf::Identity();
    uint64_t last_update_micros = 0;
    uint64_t warmup_cycles = 0;
    Eigen::Vector3f body_warmup_field, body_warmup_acc;
    AHRSState state = AHRSState::AHRS_None;

    void update_madgwick(float dt, const Eigen::Vector3f& angular_vel,
                         const Eigen::Vector3f& body_acc, const Eigen::Vector3f& local_acc_goal,
                         const Eigen::Vector3f& body_field,
                         const Eigen::Vector3f& local_field_goal);

    void finalize_warmup(const Eigen::Vector3f& local_acc_goal,
                         const Eigen::Vector3f& local_field_goal);

    void update_warmup(float dt, const Eigen::Vector3f& angular_vel,
                       const Eigen::Vector3f& body_acc, const Eigen::Vector3f& local_acc_goal,
                       const Eigen::Vector3f& body_field, const Eigen::Vector3f& local_field_goal);

    void update(float dt, const Eigen::Vector3f& angular_vel, const Eigen::Vector3f& body_acc,
                const Eigen::Vector3f& local_acc_goal, const Eigen::Vector3f& body_field,
                const Eigen::Vector3f& local_field_goal);

    void randomize();

    Eigen::Vector3f vector_body_to_local(Eigen::Vector3f& body_vec);
    Eigen::Vector3f vector_local_to_body(Eigen::Vector3f& local_vec);
};
