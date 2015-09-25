#ifndef __NATURE_HEADER__
#define __NATURE_HEADER__

#include <iostream>
#include <vector>

/*
 * Nature class of Prediction with Expert Advice model (PWEA).
 * This is part of the online learning algorithm for Weighted Majority Algorithm.
 */

class Nature {
    public:
        Nature();
        ~Nature();

        int get_label();
        std::vector<int> get_observations();
        void set_expert_predictions(std::vector<int> e_predicitons);
        void set_final_prediction(int prediction);
        void set_weights(std::vector<float> weights);

    private:
        int label;          // label to return to learner
        int prediction;     // prediction made by the learner
        std::vector<int> vec_observations;  // observation(s) to send to learner
        /*
         * In adversarial mode, Nature can see the weights and predictions of
         * each experts of the Learner.
         */
        std::vector<float> vec_weights;
        std::vector<int> vec_e_predictions;
};

#endif // __NATURE_HEADER
