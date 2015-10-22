#ifndef __DIEHARDEXPERT_HEADER__
#define __DIEHARDEXPERT_HEADER__

#include "Expert.h"

class DieHardExpert : public Expert {
    public:
        int make_prediction();
        void print();
};

#endif // __DIEHARDEXPERT_HEADER__
