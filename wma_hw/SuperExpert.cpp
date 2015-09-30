#include "SuperExpert.h"

/*
 * Observation vector:
 *  0 - game id
 *  1 - weather
 *  2 - field
 *  3 - location
 *
 * Super Expert predicts 1 if is_sunny && (grass || home)
 */
int SuperExpert::make_prediction() {
    // predicts loss if it's astroturf
    if ((vec_observations[1]) && (vec_observations[1] || vec_observations[2])) {
        prediction = 1;
    } else {
        prediction = 0;
    }
    return prediction;
}

void SuperExpert::print() {
    printf("I'm a Super Expert.\n");
}
