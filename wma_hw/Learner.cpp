#include "Learner.h"
#include "DieHardExpert.h"
#include "PessimistExpert.h"
#include "OddEvenExpert.h"

/* 
 * Basic constructor that creates 1 weight, 1 expert.
 */
Learner::Learner() {
}

/* 
 * Constructor that creates specified number of experts, and the corresponding
 * number of weights.
 */
Learner::Learner(int num_experts) {

    for (int i=0; i<num_experts; i++) {
        // initialize weights
        vec_weights.push_back(1);
        // create Experts
        std::shared_ptr<DieHardExpert> p1(new DieHardExpert());
        vec_experts.push_back(p1);
    }
}

/*
 * Destructor
 */
Learner::~Learner() {
}

/*
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

/*
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

/*
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

void Learner::calculate_loss() {
}

/*
 * Return the number of experts this learner has.
 */
int Learner::get_num_experts() {
    return vec_experts.size();
}

/*
 * Return the number weights this learner has.
 */
int Learner::get_num_weights() {
    return vec_weights.size();
}

void Learner::make_prediction() {
}

/*
 * Call each expert to identify/print what type of expert they are.
 */
void Learner::print_experts() {
    for(int i=0; i<vec_experts.size(); i++) {
        printf("%d: ", i);
        vec_experts[i]->print();
    }
}

/*
 * Print all the weight values.
 */
void Learner::print_weights() {
    for(int i=0; i<vec_weights.size(); i++) {
        printf("%d: %f\n", i, vec_weights[i]);
    }
}

void Learner::set_observations(std::vector<int> observations) {
    vec_observations = observations;
}

void Learner::update_weights() {
}
