#ifndef __PESSIMISTEXPERT_HEADER__
#define __PESSIMISTEXPERT_HEADER__

#include "Expert.h"

class PessimistExpert : public Expert {
    public:
        int make_prediction();
        void print();
};

#endif // __PESSIMISTEXPERT_HEADER__
