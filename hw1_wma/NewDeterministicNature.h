#ifndef __NEWDETERMINISTICNATURE_HEADER__
#define __NEWDETERMINISTICNATURE_HEADER__

#include "Nature.h"

class NewDeterministicNature : public Nature {
    public:
        NewDeterministicNature() : Nature() {};
        NewDeterministicNature(int round) : Nature(round) {};
        int get_label(int round);
        
    private:
        void calculate_label(int round);
};

#endif // __NEWDETERMINISTICNATURE_HEADER__
