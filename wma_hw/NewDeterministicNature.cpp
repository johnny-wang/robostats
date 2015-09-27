#include "definitions.h"
#include "NewDeterministicNature.h"

/* 
 * Return the label (after calculating it).
 */
int NewDeterministicNature::get_label(int round) {
    // calculate the label
    calculate_label(round);

    return label;
}

void NewDeterministicNature::calculate_label(int round) {
    int game_num = vec_observations[round][OBS_GAME_ID];
    int is_sunny = vec_observations[round][OBS_WEATHER];
    int is_grass = vec_observations[round][OBS_FIELD];
    int is_home = vec_observations[round][OBS_LOCATION];

    if ((game_num % 3) && (is_sunny || is_grass || is_home)) {
        label = 1;
    } else {
        label = 0;
    }
}
