/*
Title: Complementary Filter Implementation

Authors: Chris Efthalidis, Vangelis Epanomitis
*/

#ifndef COMPLEMENTARY_H
#define COMPLEMENTARY_H

typedef struct
{

    float first_parameter, second_parameter;

} COMPLEMENTARY;

void COMPLEMENTARY_Initialise(COMPLEMENTARY *filter, float first_parameter, float second_parameter);
float COMPLEMENTARY_Fuse(COMPLEMENTARY *filter, float first_variable, float second_variable);

#endif
