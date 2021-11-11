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

int xmin{}; int xmax=40; int ymin {}; int ymax= 40;
float L=2;

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
    
    void randomize_state(){
       x = rand() % ((xmax+1-xmin)*100) + xmin*100; x/=100;
       y = rand() % ((ymax+1-ymin)*100) + ymin*100; y/=100;
       theta0 = rand() % ((628+1-0)*10) + 0*10; theta0/=1000;
       theta1 = rand() % ((628+1-0)*10) + 0*10; theta1/=1000;
       theta2 = rand() % ((628+1-0)*10) + 0*10; theta2/=1000;
       theta3 = rand() % ((628+1-0)*10) + 0*10; theta3/=1000;
       v = rand() % ((5000+1+1666)*1)  -1666; v/=10000;
       phi = rand() % ((5266+1+5266)*1) - 5266*1; phi/=10000;
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

state goal_state;
state start_state(11,2,0,0,0,0,0,0);
int g=1;

//function definitions
state getnearstate(const state &s);
float dist_states(const state &a, const state&s);
vector <vector <float>> generate_input (const state &near_state);
vector <vector <float>> generate_input (const state &near_state);
state find_new_state(const vector <vector <float>> &urand, const state &near_state, const state &rand_state);
state runge_kutta( const state &near_state, const vector <float> &urandom);

int main()
{
    cout<<"Hihello"<<endl;
    
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
    
    while( (newstate.check_state() == 0) && (count<=1) ){
        
        rand_state.randomize_state();
        cout<<"\nDisplaying random states\n";
        rand_state.displaystate();
        
        state near_state = getnearstate(rand_state);
        cout<<"\nDisplaying near states\n";
        near_state.displaystate();
        
        //set u rand
        vector < vector <float> > urand = generate_input( near_state );
        state newstate = find_new_state(urand,near_state,rand_state);
        
        
//        state b (12, 38 , 0, 0, 0, 0, 0, 0);
//        addstate c (178,341);
//        c.theta1=0.89;
//        b=c;
//        b.displaystate();
//        cout<<endl<<endl;
        
        count++;
    }
    
    
    return 0;
}


float dist_states(const state &a, const state&s){
    float dist{};
       dist+= ( ( a.x-s.x )*( a.x-s.x ) ) ;
       dist+= ( ( a.y-s.y )*( a.y-s.y ) ) ;
       dist+= ( ( a.theta0-s.theta0 )*( a.theta0-s.theta0 ) * (6.369*6.369) ) ;
       dist+= ( ( a.v-s.v )*( a.v-s.v ) * (60*60) ) ;
       dist+= ( ( a.phi-s.phi )*( a.phi-s.phi ) * (38.216*38.216) ) ;
       dist+= ( ( a.theta1-s.theta1 )*( a.theta1-s.theta1 ) * (6.369*6.369) ) ;
       dist+= ( ( a.theta2-s.theta2 )*( a.theta2-s.theta2 ) * (6.369*6.369) ) ;
       dist+= ( ( a.theta3-s.theta3 )*( a.theta3-s.theta3 ) * (6.369*6.369) ) ;
    return ( dist );
}

state getnearstate(const state &s){
    float least_distance=INT_MAX;
    state nearest_state;
    for(int i=0; i<graph.size(); ++i){
       float dist = dist_states( graph.at(i) ,s );
    
     if( least_distance>dist ){
        least_distance=dist;
        nearest_state=graph.at(i);
     }    
    }
    return nearest_state;
}

vector <vector <float>> generate_input (const state &near_state){
    vector < vector <float> > us{};
    for(int i=0; i<20; i++){
     vector <float> u{0,0};
     u.at(0) = rand() % (( (g/6) + 1 - (1/6) )*100 ) - (1/6)*100; u.at(0)/=100;
     u.at(1) = rand() % ((5266+1+5266)*1) - 5266*1; u.at(1)/=10000;
     us.push_back(u);
    }
    return us;
}

state find_new_state(const vector <vector <float>> &urand, const state &near_state, const state &rand_state){
    state new_state;
    state this_state;
    
    float least_distance = INT_MAX;
    for(int i=0; i<urand.size(); ++i){
     this_state = runge_kutta( near_state, urand.at(i) );
     if( least_distance > dist_states(this_state,rand_state) ){
        new_state = this_state;
     }
    }
    
    return new_state;
}

state runge_kutta( const state &near_state, const vector <float> &urandom){
    state this_state;
    float delt=0.5;
    float w1{}, w2{}, w3{}, w4{};
    //velocity
    //w1 = urandom.at(0);
    //w2 =
    this_state.v = near_state.v + delt * urandom.at(0);
    this_state.phi = near_state.phi + delt * urandom.at(1);
    
    this_state.theta0 = near_state.theta0 + delt * (near_state.v/L) * tan(near_state.phi);
    this_state.x = near_state.x + delt * near_state.v * cos(near_state.theta0) ;
    this_state.y = near_state.y + delt * near_state.v * sin(near_state.theta0) ;
    this_state.theta1 = near_state.theta1 + delt * (near_state.v/L) * sin(near_state.theta0 - near_state.theta1);
    this_state.theta2 = near_state.theta2 + delt * (near_state.v/L) * cos(near_state.theta0 - near_state.theta1) * sin(near_state.theta0 - near_state.theta1);
    this_state.theta3 = near_state.theta3 + delt * (near_state.v/L) * cos(near_state.theta1 - near_state.theta2) * cos(near_state.theta0 - near_state.theta1) * sin(near_state.theta0 - near_state.theta1);
}

