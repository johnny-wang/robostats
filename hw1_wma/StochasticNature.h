#ifndef __STOCHASTICNATURE_HEADER__
#define __STOCHASTICNATURE_HEADER__

#include "Nature.h"

/*
 * Stochastic Nature provides 'randomly' selected labels.
 * We use a random number generator in this case.
 */

class StochasticNature : public Nature {
    public:
        StochasticNature();
        StochasticNature(int round) : Nature(round) { srand(time(NULL)); };
        int get_label(int round);

    private:
        void calculate_label(int round);
};

#endif // __STOCHASTICNATURE_HEADER__
