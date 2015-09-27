#ifndef __FIELDEXPERT_HEADER__
#define __FIELDEXPERT_HEADER__

#include "Expert.h"

class FieldExpert : public Expert {
    public:
        int make_prediction();
        void print();
};

#endif // __FIELDEXPERT_HEADER__
