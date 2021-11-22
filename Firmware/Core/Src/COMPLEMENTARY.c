#include "COMPLEMENTARY.h"

void COMPLEMENTARY_Initialise(COMPLEMENTARY *filter, float first_parameter, float second_parameter)
{
    filter->first_parameter = first_parameter;
    filter->second_parameter = second_parameter;
}

float COMPLEMENTARY_Fuse(COMPLEMENTARY *filter, float first_variable, float second_variable)
{
    return filter->first_parameter * first_variable + filter->second_parameter * second_variable;
}
