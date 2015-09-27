#include "StochasticNature.h"

StochasticNature::StochasticNature() {
    // set the seed for random number generator
    srand(time(NULL));
}

void StochasticNature::calculate_label(int round) {
    label = rand() % 2;
}

/*
 * Get the label.
 */
int StochasticNature::get_label(int round) {
    // Calculate the label
    calculate_label(round);

    return label;
}

