#include "Nature.h"

Nature::Nature() {
}

Nature::Nature(int rounds) {
    for(int i=0; i<rounds; i++) {
        std::vector<int> temp;
        temp.push_back(i);
        vec_observations.push_back(temp);
    }
}

Nature::~Nature() {
}

/***********************************************************************
 * Get the label.
 */
int Nature::get_label(int round) {
    // Calculate the label
    calculate_label(round);

    return label;
}

/***********************************************************************
 * Get the label.
 * In the adversarial case, we'll need to look at the weights and expert
 * predictions of the Learner.
 */
int Nature::get_label(int round, std::vector<int> e_pred, std::vector<double> wts) {
    vec_e_predictions = e_pred;
    vec_weights = wts;

    // Calculate the label
    calculate_label(round);

    return label;
}

/***********************************************************************
 * Return the observations of the given round.
 */
std::vector<int> Nature::get_observations(int round) {
    return vec_observations[round];
}

/***********************************************************************
 *
 */
void Nature::set_expert_predictions(std::vector<int> e_pred) {
    vec_e_predictions = e_pred;
}

/***********************************************************************
 *
 */
void Nature::set_final_prediction(int pred) {
    prediction = pred;
}

/***********************************************************************
 *
 */
void Nature::set_weights(std::vector<double> wts) {
    vec_weights = wts;
}
