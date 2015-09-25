#include <iostream>

#include "Nature.h"
#include "Learner.h"

using namespace std;

/*
 * This is an online learning Prediction with Expert Advice Model (PWEA).
 * It used to simulate the Weighted Majority Algorithm.
 *
 * Outline of how the code should run:
 * 1. Nature send observation
 * 2. Learner receives observation, asks each Expert to make a prediction.
 * 3. Nature determines the label.
 *    For adversarial case, the nature gets to see the weight vector and prediction
 *    for each Expert (but not directly the final prediction made by the learner).
 * 4. Learner makes final prediction and the true label is revealed.
 * 5. Learner suffers loss and updates Expert weight.
 * 6. Repeat.
 * 
 * Total Rounds = 100.
 */

int main() {

    Nature my_nature = Nature();
    Learner my_learner = Learner();
    // add the 3 experts
    my_learner.add_diehard_expert();
    my_learner.add_pessimist_expert();
    my_learner.add_oddeven_expert();

    cout << my_learner.get_num_experts() << endl;
    cout << my_learner.get_num_weights() << endl;

    my_learner.print_experts();
    my_learner.print_weights();
   
    return 0; 
} 
