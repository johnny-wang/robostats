#ifndef __LOCATIONEXPERT_HEADER__
#define __LOCATIONEXPERT_HEADER__

#include "Expert.h"

class LocationExpert : public Expert {
    public:
        int make_prediction();
        void print();
};

#endif // __LOCATIONEXPERT_HEADER__
