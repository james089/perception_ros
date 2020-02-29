#include <mUtils/mFunctions.h>

namespace mUtils
{   
    // drop evenly each round
    double drop_even(const double &current_val, const double &init_val, const int &total_round)
    {
        return (current_val - init_val / total_round);
    }

    // drop in same ratio (d is the same for each round's weight)
    double drop_acc_even(const double &current_val, const double &init_val, const int &current_round, const int &total_round)
    {
        return (current_val - (total_round - current_round) * 2 * init_val / (total_round * (total_round - 1)));
    }
}