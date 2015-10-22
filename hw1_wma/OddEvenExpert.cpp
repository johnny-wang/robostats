#include "OddEvenExpert.h"

int OddEvenExpert::make_prediction() {
    // predicts loss if it's odd number games
    if (vec_observations[0] % 2) {
        prediction = 0;
    } else {
    // predicts win if it's even number games
        prediction = 1;
    }

    return prediction;
}

void OddEvenExpert::print() {
    printf("I'm a OddEven Expert\n");
}
