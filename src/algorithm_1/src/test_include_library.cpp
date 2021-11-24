#include "mixed_reality_library/mixed_reality_library.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_include_library");
    ros::NodeHandle nh;
    sayHello();
}
