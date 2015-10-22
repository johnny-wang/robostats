#ifndef __SUPEREXPERT_HEADER__
#define __SUPEREXPERT_HEADER__

#include "Expert.h"

class SuperExpert : public Expert {
    public:
        int make_prediction();
        void print();
};

#endif // __SUPEREXPERT_HEADER__
