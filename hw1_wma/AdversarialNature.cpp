#include "AdversarialNature.h"

/*
 * Calculate the label to return.
 * Since this is an Adversarial Nature, we will peek at the Learner's Expert
 * predictions and weights before calculating the label.
 *
 * RETURN:
 *  label - 0 or 1 (opposite of what we think Learner will predict).
 */
int AdversarialNature::get_label(int round, std::vector<int> e_pred,
    std::vector<double> wts) {

    //printf("weights size: %lu\n", wts.size());
    //printf("pred    size: %lu\n", e_pred.size());

    vec_e_predictions = e_pred;
    vec_weights = wts;

    calculate_label(round);

    return label;
}

/*
 * Calculate a label based on weighted majority algorithm.
 * 
 */
void AdversarialNature::calculate_label(int round) {
    std::map<int, double> wt_sum; // weighted sum of expert prediction and weights. 

    //printf("weights size: %lu\n", vec_weights.size());
    //printf("pred    size: %lu\n", vec_e_predictions.size());

    // error checking
    if (vec_weights.size() != vec_e_predictions.size()) {
        printf("ERROR: weight and expert prediction vectors are not same size!\n");
        return;
    }

    for (int i=0; i<vec_weights.size(); i++) {
        if (wt_sum.find(vec_e_predictions[i]) == wt_sum.end()) {
            // this expert prediction doesn't exist, add it
            wt_sum[vec_e_predictions[i]] = vec_weights[i];
        } else {
            // increment count by weighted value
            wt_sum[vec_e_predictions[i]] += vec_weights[i];
        }
    }

    // show map contents
    //print_wt_sum(wt_sum);
    get_true_label(wt_sum);     // true for adversary of WMA
}

/*
 * Print the weight sum map contents.
 */
void AdversarialNature::print_wt_sum(std::map<int, double> wt_sum) {
    std::map<int, double>::iterator it;
    for (it=wt_sum.begin(); it!=wt_sum.end(); it++) {
        printf("%d: %f\n", it->first, it->second);
    }
}

/*
 * Find the _smallest_ weighted sum value to use as the label. This is because
 * we are assuming the Learner is using WMA so it will pick the _largest_ weighted
 * sum to use as the label.
 */
void AdversarialNature::get_true_label(std::map<int, double> wt_sum) {
    std::map<int, double>::iterator it;
    double count = 999999999.0f;

    for (it=wt_sum.begin(); it!=wt_sum.end(); it++) {
        printf("%d: %f\n", it->first, it->second);
        // Adversarial gets the _smallest_ weighted value as label
        if (it->second < count) {
            count = it->second;
            label = it->first;
        }
    }
}
