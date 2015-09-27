#include "FieldExpert.h"

/*
 * Observation vector:
 *  0 - game id
 *  1 - weather
 *  2 - field
 *  3 - location
 *
 * Field can be grass (0) or astroturf (1).
 * If it's grass, then the expert predicts win, else it predicts loss.
 */
int FieldExpert::make_prediction() {
    // predicts loss if it's astroturf
    if (vec_observations[2]) {
        prediction = 0;
    } else {
    // predicts win if it's grass
        prediction = 1;
    }
    return prediction;
}

void FieldExpert::print() {
    printf("I'm a Field Expert.\n");
}
