#include "Learner.h"
#include "definitions.h"

/*********************************************************************
 * Basic constructor that creates the 3 experts we need.
 */
Learner::Learner() {
    //add_diehard_expert();
    //add_pessimist_expert();
    //add_oddeven_expert();

    //vec_e_predictions.reserve(vec_experts.size());
    //for (int i=0; i<vec_experts.size(); i++) {
    //    // Fill expert prediction vector with -1s.
    //    vec_e_predictions.push_back(-1);
    //    // Fill expert losses with 0s.
    //    expert_loss.push_back(0);
    //}

    add_all_experts();

    // initialize learner loss
    learner_loss = 0;
}

/*********************************************************************
 * Destructor
 */
Learner::~Learner() {
}

/*********************************************************************
 *
 */
void Learner::add_all_experts() {
    for (int expert = DIEHARD; expert <= LOCATION; expert++) {
        add_expert(expert);
    }
}

/*********************************************************************
 *
 */
void Learner::add_expert(int exp_type) {
    std::shared_ptr<Expert> p1;
    switch(exp_type) {
        case DIEHARD: {
            std::shared_ptr<DieHardExpert> p2(new DieHardExpert());
            p1=p2;
        }
        break;
        case PESSIMIST: {
            std::shared_ptr<PessimistExpert> p2(new PessimistExpert());
            p1=p2;
        }
        break;
        case ODDEVEN: {
            std::shared_ptr<OddEvenExpert> p2(new OddEvenExpert());
            p1=p2;
        }
        break;
        case WEATHER: {
            std::shared_ptr<WeatherExpert> p2(new WeatherExpert());
            p1=p2;
        }
        break;
        case FIELD: {
            std::shared_ptr<FieldExpert> p2(new FieldExpert());
            p1=p2;
        }
        break;
        case LOCATION: {
            std::shared_ptr<LocationExpert> p2(new LocationExpert());
            p1=p2;
        }
        break;
    }
    vec_experts.push_back(p1);
    // add weight for the expert
    vec_weights.push_back(1);
    // add expert prediction vector with -1.
    vec_e_predictions.push_back(-1);
    // add expert losses with 0.
    expert_loss.push_back(0);

}

/*********************************************************************
 * Add new Diehard Expert (and associated weight) to our vector.
 * RETURN:
 *  0 - success
 *  1 - error (vector lengths are different)
 */
int Learner::add_diehard_expert() {
    // add to weight vector and expert vector
    vec_weights.push_back(1);
    std::shared_ptr<DieHardExpert> p1(new DieHardExpert());
    vec_experts.push_back(p1);

    // check vector lengths are the same
    if (vec_experts.size() == vec_weights.size())
        return 0;
    else 
        return 1;
}

/*********************************************************************
 * Add new Pessimist Expert (and associated weight) to our vector.
 * RETURN:
 *  0 - success
 *  1 - error (vector lengths are different)
 */
int Learner::add_pessimist_expert() {
    // add to weight vector and expert vector
    vec_weights.push_back(1);
    std::shared_ptr<PessimistExpert> p1(new PessimistExpert());
    vec_experts.push_back(p1);

    // check vector lengths are the same
    if (vec_experts.size() == vec_weights.size())
        return 0;
    else 
        return 1;
}

/*********************************************************************
 * Add new OddEven Expert (and associated weight) to our vector.
 * RETURN:
 *  0 - success
 *  1 - error (vector lengths are different)
 */
int Learner::add_oddeven_expert() {
    // add to weight vector and expert vector
    vec_weights.push_back(1);
    std::shared_ptr<OddEvenExpert> p1(new OddEvenExpert());
    vec_experts.push_back(p1);

    // check vector lengths are the same
    if (vec_experts.size() == vec_weights.size())
        return 0;
    else 
        return 1;
}

/*********************************************************************
 * Calculate the label based on Experts' predictions and weights.
 *
 * RETURN: 
 *  0 - success
 *  1 - failure
 */
int Learner::calculate_label(int type) {
    std::map<int, double> wt_sum;    // weighted sum of expert prediction & weights.

    //printf("weights size: %lu\n", vec_weights.size());
    //printf("pred    size: %lu\n", vec_e_predictions.size());

    // error checking
    if (vec_weights.size() != vec_e_predictions.size()) {
        printf("ERROR: weight and expert prediction vectors are not same size!\n");
        return 1;
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

    if (type == WMA) {
        // calculate wma 
        get_wma_label(wt_sum);
    } else {
        // calculate rwma 
        get_rwma_label();
    }

    return 0;
}

/*********************************************************************
 * Calculate the loss after receiving final label from Nature.
 */
void Learner::calculate_loss(int nature_label) {
    // update learner's loss
    if (learner_label != nature_label) {
        learner_loss++;
    }

    // update experts' losses
    for (int i=0; i<vec_e_predictions.size(); i++) {
        if (vec_e_predictions[i] != nature_label) {
            expert_loss[i] += 1;
        }
    }

    // calculate regret
    // max(loss of learner - expert_loss)
    regret = -999999999;  // in case we get negative regret
    for (int i=0; i<expert_loss.size(); i++) {
        int temp = learner_loss - expert_loss[i];
        printf("Regret single: %d\n", temp);
        if (temp > regret) {
            regret = temp;
        }
    }
    printf("regret: %d\n", regret);
}

/*********************************************************************
 * Return Expert prediction vector.
 */
std::vector<int> Learner::get_e_predictions() {
    return vec_e_predictions;
}

/*********************************************************************
 * Return expert loss vector.
 */
std::vector<int> Learner::get_expert_loss() {
    return expert_loss;
}

/*********************************************************************
 * Return calculated label.
 */
int Learner::get_label() {
    return learner_label;
}

/*********************************************************************
 * Return learner loss.
 */
int Learner::get_learner_loss() {
    return learner_loss;
}

/*********************************************************************
 * Return the number of experts this learner has.
 */
int Learner::get_num_experts() {
    return vec_experts.size();
}

/*********************************************************************
 * Return the number weights this learner has.
 */
int Learner::get_num_weights() {
    return vec_weights.size();
}

/*********************************************************************
 * Return the regret value.
 */
int Learner::get_regret() {
    return regret;
}

/*********************************************************************
 * Return the weight vector.
 */
std::vector<double> Learner::get_weights() {
    return vec_weights;
}

/*********************************************************************
 * Call Experts to make prediction based on given observation.
 */
void Learner::make_predictions() {
    for(int i=0; i<vec_experts.size(); i++) {
        vec_e_predictions[i] = vec_experts[i]->make_prediction();
        printf("e_pred: %d", vec_e_predictions[i]);
        printf(" wt: %.30lf\n", vec_weights[i]);
    }
}

/*********************************************************************
 * Call each expert to identify/print what type of expert they are.
 */
void Learner::print_experts() {
    for(int i=0; i<vec_experts.size(); i++) {
        printf("%d: ", i);
        vec_experts[i]->print();
    }
}

/*********************************************************************
 * Print all the weight values to screen.
 */
void Learner::print_weights() {
    for(int i=0; i<vec_weights.size(); i++) {
        printf("%d: %f\n", i, vec_weights[i]);
    }
}

/*********************************************************************
 * Set the eta penalty value.
 */
void Learner::set_eta(float tmp_eta) {
    eta = tmp_eta;
}

/*********************************************************************
 * Save the observations as given from Nature. Also update these observations
 * to the Experts.
 */
void Learner::set_observations(std::vector<int> observations) {
    vec_observations = observations;
    // set observations for all Experts
    for (int i=0; i<vec_experts.size(); i++) {
        vec_experts[i]->set_observations(observations);
    }
}

/*********************************************************************
 * Update weights based on Nature's label.
 * All the experts who got the wrong label will have their weights lowered.
 */
void Learner::update_weights(int nature_label) {
    for (int i=0; i<vec_experts.size(); i++) {
        if (vec_e_predictions[i] != nature_label) {
            vec_weights[i] = vec_weights[i] * (1-eta);
        }
    }

    // normalize weights?
}

/*********************************************************************
 * Find the largest weighted sum value to use as the label. This is because
 * in the Weighted Majority Algorithm, we sum up the weights and predictions
 * and choose the largest as the true label.
 */
void Learner::get_wma_label(std::map<int, double> wt_sum) {
    std::map<int, double>::iterator it;
    double count = 0.0f;

    for (it=wt_sum.begin(); it!=wt_sum.end(); it++) {
        printf("%d: %f\n", it->first, it->second);
        // WMA wants the _largest_ sum
        if (it->second > count) {
            count = it->second;
            learner_label = it->first;
        }
    }
}

/*********************************************************************
 * Use Randomized Weighted Majority Algorithm to find the best label.
 * For this algorithm we find the probability of each weight against the sum
 * of the weights. We then randomly choose one of the weights based on
 * the probabilities.
 */
void Learner::get_rwma_label() {
    double sum_wts = 0;
    int rand_e_index;

    // Use c++11's discrete_distribution
    std::random_device rd;
    std::mt19937 gen(rd()); // random generator
    std::discrete_distribution<double> dist(vec_weights.begin(), vec_weights.end());

    rand_e_index = dist(gen);
    printf("dist gen: %d ", rand_e_index);
    printf("label: %d\n", vec_e_predictions[rand_e_index]);

    learner_label = vec_e_predictions[rand_e_index];
}
