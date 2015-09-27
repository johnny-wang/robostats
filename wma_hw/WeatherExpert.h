#ifndef __WEATHEREXPERT_HEADER__
#define __WEATHEREXPERT_HEADER__

#include "Expert.h"

class WeatherExpert : public Expert {
    public:
        int make_prediction();
        void print();
};

#endif // __WEATHEREXPERT_HEADER__
