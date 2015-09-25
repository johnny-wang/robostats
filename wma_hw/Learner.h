#ifndef __LEARNER_HEADER__
#define __LEARNER_HEADER__

#include <iostream>
#include <vector>

#include "Expert.h"

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
        void calculate_loss();
        int get_num_experts();
        int get_num_weights();
        void make_prediction();
        void print_experts();
        void print_weights();
        void set_observations(std::vector<int> observations);
        void update_weights();

    private:
        std::vector<int> vec_observations;
        std::vector<float> vec_weights;
        std::vector<std::shared_ptr<Expert> > vec_experts;
};

#endif // __LEARNER_HEADER__
