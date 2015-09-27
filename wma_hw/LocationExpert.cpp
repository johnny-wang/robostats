#include "LocationExpert.h"

/*
 * Observation vector:
 *  0 - game id
 *  1 - weather
 *  2 - field
 *  3 - location
 *
 * Location can be home (0) or away(1).
 * If it's home, then the expert predicts win, else it predicts loss.
 */
int LocationExpert::make_prediction() {
    // predicts loss if it's away game.
    if (vec_observations[3]) {
        prediction = 0;
    } else {
    // predicts win if it's home game.
        prediction = 1;
    }
    return prediction;
}

void LocationExpert::print() {
    printf("I'm a Location Expert.\n");
}
