#include "Nature.h"

Nature::Nature() {
}

Nature::~Nature() {
}

int Nature::get_label() {
    return label;
}

std::vector<int> Nature::get_observations() {
    return vec_observations;
}

void Nature::set_expert_predictions(std::vector<int> e_pred) {
    vec_e_predictions = e_pred;
}

void Nature::set_final_prediction(int pred) {
    prediction = pred;
}

void Nature::set_weights(std::vector<float> wts) {
    vec_weights = wts;
}
