#ifndef __DETERMINISTICNATURE_HEADER__
#define __DETERMINISTICNATURE_HEADER__

#include "Nature.h"

class DeterministicNature : public Nature {
    public:
        DeterministicNature() : Nature() {};
        DeterministicNature(int round) : Nature(round) {};
        int get_label(int round);
        bool is_prime(int num);

    private:
        void calculate_label(int round);
};

#endif // __DETERMINISTICNATURE_HEADER__
