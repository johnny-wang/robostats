#include "DeterministicNature.h"

/***********************************************************************
 * Get the label.
 */
int DeterministicNature::get_label(int round) {
    // Calculate the label
    calculate_label(round);

    return label;
}

/*
 * Only "win" if game number is prime.
 */
void DeterministicNature::calculate_label(int round) {
    int game_num = vec_observations[round][0];

    //int isHome = game_num % 4 == 0 ? 1:0;
    //int weather = game_num % 7;
    //int fatigue = game_num % 5;
    //label = ( (isHome == 1 || weather>3) && fatigue<3) == 0? 0: 1;

    if (is_prime(game_num))
        label = 1;
    else
        label = 0;
}

bool DeterministicNature::is_prime(int num) {
    // Base case: check if less than or equal to 3
    if (num <= 3) {
        return num > 1;     // True for 2 and 3
    } else if ( ((num % 2) == 0) || ((num % 3) == 0) ) {
        return false;       // Check if multiple of 2 or 3
    } else {
        // Everything else starting from 5.
        // Only check numbers less than sqrt(num)
        // Increment by 6 because we already checked multiples of 2 and 3
        for (int i=5; i*i <= num; i+=6) {
            if ((num % i == 0) || (num % (i+2) == 0)) {
                return false;
            } 
        }
        return true;
    }
}
