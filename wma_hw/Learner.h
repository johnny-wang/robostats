#ifndef __LEARNER_HEADER__
#define __LEARNER_HEADER__

#include <iostream>
#include <vector>
#include <map>
#include <random>

#include "Expert.h"
#include "DieHardExpert.h"
#include "PessimistExpert.h"
#include "OddEvenExpert.h"

/*
 * Nature class of Prediction with Expert Advice model (PWEA).
 * This is part of the online learning algorithm for Weighted Majority Algorithm.
 */

class Learner {
    public:
        Learner();
        Learner(int num_experts);
        ~Learner();

        int add_diehard_expert();
        int add_pessimist_expert();
        int add_oddeven_expert();
        int calculate_label(int learner_type);
        void calculate_loss(int nature_label);
        std::vector<int> get_e_predictions();
        std::vector<int> get_expert_loss();
        int get_label();
        int get_learner_loss();
        int get_num_experts();
        int get_num_weights();
        int get_regret();
        std::vector<double> get_weights();
        void make_predictions();
        void print_experts();
        void print_weights();
        void set_eta(float eta);
        void set_observations(std::vector<int> observations);
        void update_weights(int nature_label);

    private:
        void get_wma_label(std::map<int, double> wt_sum);
        void get_rwma_label();

        // variables
        int learner_label;      // learner's label
        float eta;              // penalty weight reduction
        std::vector<int> vec_observations;
        std::vector<double> vec_weights;
        std::vector<int> vec_e_predictions;
        std::vector<std::shared_ptr<Expert> > vec_experts;
        int learner_loss;    // cumulative loss of learner
        std::vector<int> expert_loss;     // cumulative loss of experts
        int regret;
};

#endif // __LEARNER_HEADER__
