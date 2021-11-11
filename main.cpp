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
float L=2, l=0.5, w=1;

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
        
        if ( ifisstatevalid(new_state) == 1 ){
            
        }
        
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
    this_state.theta1 = near_state.theta1 + delt * (near_state.v/(L+l)) * sin(near_state.theta0 - near_state.theta1);
    this_state.theta2 = near_state.theta2 + delt * (near_state.v/(L+l)) * cos(near_state.theta0 - near_state.theta1) * sin(near_state.theta0 - near_state.theta1);
    this_state.theta3 = near_state.theta3 + delt * (near_state.v/(L+l)) * cos(near_state.theta1 - near_state.theta2) * cos(near_state.theta0 - near_state.theta1) * sin(near_state.theta0 - near_state.theta1);
    
    return this_state;
}

int ifisstatevalid(const state &s){
    if( (s.v>=-0.1667)&&(s.v<=0.5) && (s.phi>=-0.5233)&&(s.phi<=0.5233) ){
        if( (s.x>=xmin)&&(s.x<=xmax) && (s.y>=ymin)&&(s.y<=ymax) && (s.theta0>=0)&&(s.theta0<=6.28) ){
            if( check_state_collision(s) == 0 ){
                return 1;
            }
            else return 0;
        }
        else return 0;
    }
    else return 0;
}

int check_state_collision(const state &s){
    //tractors and all trailers
    vector <vector<float> xytheta = getvehiclecoordinates(s.x, s.y, s.theta0)
}

vector <vector<float> getvehiclecoordinates(const float &x, const float &y, const float &theta){
    x1 = x + (w/2)*sin(theta);
    y1 = y - (w/2)*cos(theta);
    
    x2 = x + (w/2)*sin(theta) + L*cos(theta);
    y2 = y - (w/2)*cos(theta) + L*sin(theta);
    
    x3 = x - (w/2)*sin(theta) + L*cos(theta);
    y3 = y + (w/2)*cos(theta) + L*sin(theta);
    
    x4 = x - (w/2)*sin(theta);
    y4 = y + (w/2)*cos(theta);
    
    
}

float theta_vector(float p1x, float p1y, float p2x, float p2y){
    if(((p2y-p1y)>=0)&&((p2x-p1x)>=0))             //1st quadrant
        return (atan((p2y-p1y)/(p2x-p1x)));
    else if(((p2y-p1y)>=0)&&((p2x-p1x)<0))
        return (3.14159+atan((p2y-p1y)/(p2x-p1x)));
    else if(((p2y-p1y)<0)&&((p2x-p1x)<0))
        return (3.14159+atan((p2y-p1y)/(p2x-p1x)));
    else if(((p2y-p1y)<0)&&((p2x-p1x)>=0))
        return ((2*3.14159)+atan((p2y-p1y)/(p2x-p1x)));
    else
        return (0);
}

int ifispointobstacle(float px, float py){
    int count=0;
    int inside_obstalce=0;
    float total_theta {};
    for(int i=0; i<obstacle_vector.size(); i++){total_theta=0;count=0;
        for(int j=0; j<obstacle_vector.at(i).size(); j++){
          int m=j+1;
          float theta1 {}, theta2 {};
          if((j+1)==obstacle_vector.at(i).size())
              m=0;
          theta2=theta_vector(px,py,obstacle_vector.at(i).at(m).at(0),obstacle_vector.at(i).at(m).at(1));
          theta1=theta_vector(px,py,obstacle_vector.at(i).at(j).at(0),obstacle_vector.at(i).at(j).at(1));
          if(theta1<=theta2)
             total_theta=(theta2-theta1);
          else
             total_theta=(theta2+3.14159*2-theta1);
             
          if(total_theta>3.14159)
            count++;
        }

                                                      
    if(count==0) {
    inside_obstalce++;}
             
    }
    if(inside_obstalce!=0)
        return (1);
    else
        return (0);
}
