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
        Nature(int rounds);
        virtual ~Nature();

        virtual int get_label(int round);
        virtual int get_label(int round, std::vector<int> e_pred, 
            std::vector<double> wts);
        std::vector<int> get_observations(int round);
        void set_expert_predictions(std::vector<int> e_predicitons);
        void set_final_prediction(int prediction);
        void set_weights(std::vector<double> weights);

    protected:
        // calculate label based on type of Nature
        virtual void calculate_label(int round) {}; 

        int label;          // label to return to learner
        int prediction;     // prediction made by the learner
        /* 
         * Observation(s) to send to learner. 
         * Vector of vector b/c we store vector of observations for each round.
         * First vector index is round. e.g. vec_observations[1][2] is round 0,
         * second observation element.
         */
        std::vector<std::vector<int> > vec_observations;
  
        /*
         * In adversarial mode, Nature can see the weights and predictions of
         * each experts of the Learner.
         */
        std::vector<double> vec_weights;
        std::vector<int> vec_e_predictions;
};

#endif // __NATURE_HEADER
