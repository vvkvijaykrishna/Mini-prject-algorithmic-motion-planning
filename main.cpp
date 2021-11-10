#include <iostream>
#include <vector>
#include <time.h>
#include <fstream>
#include <random>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <chrono>

using namespace std;

int xmin{}, xmax{}, ymin{}, ymax{};

struct state{
    float x;
    float y;
    float theta0;
    float v;
    float phi;
    float theta1;
    float theta2;
    float theta3;
    
    state(){
        x=0.0;
        y=0.0;
        theta0=theta1=theta2=theta3=0.0;
        v=0.0;
        phi=0.0;
    }
};

struct addstate : state {
    int state_number;
    int previous_state;
    addstate(){
        state_number=0;
        previous_state=0;
    }
};

vector <state> graph {};
vector <vector <vector <float>>> obstacle_vector{};

int main()
{
    cout<<"Hihello"<<endl;
    xmin=0;xmax=40;ymin=0;ymax=40;
    obstacle_vector={ {{0,6},{12,6},{12,10},{0,10}} , {{28,13},{40,13},{40,17},{28,17}} , 
    {{0,20},{12,20},{12,24},{0,24}} , {{28,27},{40,27},{40,31},{28,31}} };
    
    return 0;
}
