#include "agent.h"
#include <algorithm>

ARS::ARS(float stsize, float exnoise, int top2use, int samsize,
        double* pkp, double* pki, double* pkd)
    : step_size(stsize),
      explore_noise(exnoise),
      top_per2use(top2use),
      sample_size(samsize),
      kp(pkp),
      ki(pki),
      kd(pkd),
      gen(std::random_device{}()),
      dist(0, 1) {}

void ARS::perturb_parameters() {
    // Tạo nhiễu loạn có hướng
    float noise_kp = dist(gen) * explore_noise;
    float noise_ki = dist(gen) * explore_noise;
    float noise_kd = dist(gen) * explore_noise;
    
    // Cập nhật tham số với nhiễu loạn
    *kp += noise_kp * step_size;
    *ki += noise_ki * step_size;
    *kd += noise_kd * step_size;
    
    // Giới hạn giá trị tham số
    *kp = std::clamp(*kp, 0.0, 0.01);
    *ki = std::clamp(*ki, 0.0, 0.001);
    *kd = std::clamp(*kd, 0.0, 0.1);
}

void ARS::reset_parameters(double original_kp, double original_ki, double original_kd) {
    *kp = original_kp;
    *ki = original_ki;
    *kd = original_kd;
}