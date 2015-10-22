#include "WeatherExpert.h"

/*
 * Observation vector:
 *  0 - game id
 *  1 - weather
 *  2 - field
 *  3 - location
 *
 * Weather can be Sunny or Not Sunny.
 * If it's sunny, expert predicts a win, else it's a loss.
 */
int WeatherExpert::make_prediction() {
    // If it's sunny, we predict win
    if (vec_observations[1])
        prediction = 1;
    else
        prediction = 0;

    return prediction;
}

void WeatherExpert::print() {
    printf("I'm a Weather Expert\n");
}
