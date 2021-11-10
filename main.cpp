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

struct state{
    float x;
    float y;
    float theta0;
    float v;
    float phi;
    float theta1;
    float theta2;
    float theta3;
    
    state(float mx, float my, float mtheta0, float mv, float mphi, float mtheta1, float mtheta2, float mtheta3){
        x=my;
        y=mx;
        theta0=mtheta0;
        theta1=mtheta1;
        theta2=mtheta2;
        theta3=mtheta3;
        v=mv;
        phi=mphi;   
    } 
    
    state(){
        x=0.0;
        y=0.0;
        theta0=theta1=theta2=theta3=0.0;
        v=0.0;
        phi=0.0;
    }
    
    void displaystate(){
     cout<<"\nStates are:\nXpos: "<<x<<"\tYpos: "<<y<<"\tTheta0: "<<theta0<<"\tVelocity: "<<v<<"\tSteer angle: "<<phi<<"\nTheta1: "<<theta1<<"\tTheta2: "<<theta2<<"\tTheta3: "<<theta3<<endl;
    }
    
    int check_state(){
     if( (x>=0)&&(x<=6) && (y>=34)&&(y<=38) && 
     (theta0>=2.791)&&(theta0<=3.488) && (theta1>=2.791)&&(theta1<=3.488) && (theta2>=2.791)&&(theta2<=3.488) && 
     (theta3>=2.791)&&(theta3<=3.488) && (v>=-0.025)&&(v<=0.025) )
        return 1;
     else
        return 0;
    }
    
};

struct addstate : state {
    int state_number;
    int previous_state;
    addstate(int sn, int ps){
        state_number=sn;
        previous_state=ps;
    }
};

vector <addstate> graph {};
vector <vector <vector <float>>> obstacle_vector{};

int xmin{}, xmax{}, ymin{}, ymax{};
state goal_state;
state start_state(11,2,0,0,0,0,0,0);

//function definitions

int main()
{
    cout<<"Hihello"<<endl;
    
    xmin=0;xmax=40;ymin=0;ymax=40;
    obstacle_vector={ {{0,6},{12,6},{12,10},{0,10}} , {{28,13},{40,13},{40,17},{28,17}} , 
    {{0,20},{12,20},{12,24},{0,24}} , {{28,27},{40,27},{40,31},{28,31}} };
    //start_state.x=11;start_state.y=2;start_state.theta0=0;start_state.v=0;start_state.phi=0;start_state.theta1=0;start_state.theta2=0;start_state.theta3=0;
    
    addstate initial_state (1,0);
    //defining initial_states
    initial_state.x=11.0;
    initial_state.y=2.0;
    
    graph.push_back(initial_state);
    
    state rand_state;
    srand(time(0));
    int count=0,point_number=0,previous_point=0;
    state newstate;
    //newstate.x=100;
    newstate.displaystate();
    start_state.displaystate();
    
    //cout<<"\nCheck state function  1 2 3 :\t"<<start_state.check_state();
    
    while( newstate.check_state() == 0 ){
        
    }
    
    
    return 0;
}

//int check_state(const state &a){
//    if( (a.x>=0)&&(a.x<=6) && (a.y>=34)&&(a.y<=38) && 
//    (a.theta0>=2.791)&&(a.theta0<=3.488) && (a.theta1>=2.791)&&(a.theta1<=3.488) && (a.theta2>=2.791)&&(a.theta2<=3.488) && 
//    (a.theta3>=2.791)&&(a.theta3<=3.488) && (a.v>=-0.025)&&(a.v<=0.025) )
//        return 1;
//    else
//        return 0;
//}
