#ifndef _AGENT_H_
#define _AGENT_H_

#include <random>

class ARS {
private:
    // Hyperparameters
    float step_size;
    float explore_noise;
    int top_per2use;
    int sample_size;
    
    // PID parameters pointers
    double* kp;
    double* ki;
    double* kd;

    // Random number generator
    std::mt19937 gen;
    std::normal_distribution<> dist;

public:
    ARS(float stsize, float exnoise, int top2use, int samsize, 
       double* pkp, double* pki, double* pkd);
    
    void perturb_parameters();
    void reset_parameters(double original_kp, double original_ki, double original_kd);
};

#endif