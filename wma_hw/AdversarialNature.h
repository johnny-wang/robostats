#ifndef __ADVERSARIALNATURE_HEADER__
#define __ADVERSARIALNATURE_HEADER__

#include "Nature.h"
#include <map>

class AdversarialNature : public Nature {
    public:
        AdversarialNature() : Nature() {};
        AdversarialNature(int round) : Nature(round) {};
        int get_label(int round, std::vector<int> e_pred, std::vector<float> wts);

    private:
        // Make sure the parent get_labe() can't be called with this class.
        // This is because we will only calculate label with predictions and weights
        using Nature::get_label;
        void calculate_label(int round);
        void get_true_label(std::map<int, float> wt_sum);
        void print_wt_sum(std::map<int, float> wt_sum);
};

#endif // __ADVERSARIALNATURE_HEADER__
