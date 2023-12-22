#include "regulators_lib/regulators.hpp"

class AckermanRegulator : public Regulator
{
    void logic(geometry_msgs::msg::PoseStamped pose_msg)
    {
        return;
    }
};

int main ()
{
    return 0;
}