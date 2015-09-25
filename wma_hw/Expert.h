#ifndef __EXPERT_HEADER__
#define __EXPERT_HEADER__

#include <iostream>
#include <vector>

/*
 * Expert class of Prediction with Expert Advice model (PWEA).
 * This is part of the online learning algorithm for Weighted Majority Algorithm.
 * It should be extended to create unique prediction algorithms.
 */

class Expert {
    public:
        Expert();
        virtual ~Expert();

        void set_observations(std::vector<int> observations);
        virtual int make_prediction() { return 0; };
        virtual void print() { printf("I'm an Expert\n"); };

    protected:
        int prediction;
        std::vector<int> vec_observations;
};

#endif // __EXPERT_HEADER__
