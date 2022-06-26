#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>


#define TIME_STEP 32
#define MAX_SPEED 5
#define MIN_SPEED 0
using namespace webots;
using namespace std;


Robot* robot = new Robot();
Motor* wheels[2];
PositionSensor* encoders[2];
DistanceSensor* F_S[8];
double kp = 0.01;
double ki = 0.0000;
double kd = 0.00;
double P, I, D;
float leftSpeed, rightSpeed;
float  midSpeed = 5;
float previous_error = 0;
double feedback;
void rotateWheels(int, int);
void PID();
void getLineSensors();
void transferCoff();
void pidUpdate();
short gainCalc(short errorX);
int S[8] = { 0,0,0,0,0,0,0,0 };
int TH = 400;
double a0, a1, a2, b0, b1, b2;
double Kp = 15;
double Ki = 0.000;
double Kd = 0.5;
double Ts = TIME_STEP;
double N = 20;
short e[3] = { 0,0,0 };
float c[3] = { 0,0,0 };
unsigned long previousTime = 0;
char LineSensors[8][5] = { "F_S1", "F_S2", "F_S3", "F_S4", "F_S5","F_S6","F_S7","F_S8" };
char Encoder_names[2][10] = { "L_encoder" , "R_encoder" };
char wheels_names[2][8] = { "L_wheel", "R_wheel" };

double L_Offset = 0;
double R_Offset = 0;
double L_Pos = 0;
double R_Pos = 0;

//________________________________achintha 
#include <bits/stdc++.h>
#define ROW 13 
#define COL 17
typedef pair<int, int> Pair;
typedef pair<double, pair<int, int> > pPair;
struct cell {
    // Row and Column index of its parent
    // Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    int parent_i, parent_j;
    // f = g + h
    double f, g, h;
};

int route[60][2];
int current_node[2]={0,0};
int target[2]={0,2};
int destination[2]={0,2};
int middle_node[2]={0,1};
int last_visited[2]={0,0};
int stage=0;//500
int substage=0;
int node=2;

DistanceSensor*object[6];
char objectSensors[6][5] = { "O_1", "O_2", "O_3", "O_4", "O_5","O_6"};
int O[6]={0,0,0,0,0,0};
/*float matrix[13][17]={
  {1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000},
  {1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000},
  {1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000},
  {1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000},
  {1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000},
  {1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000},
  {1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000},
  {1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000},
  {1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000},
  {1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000},
  {1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000},
  {1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000,INFINITY,1000},
  {1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000,    1000,1000} 
};*/

float matrix[13][17]={
  /*00*/{     100,     100,     100,     100,     100,INFINITY,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100},
  /*01*/{     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,INFINITY,INFINITY,     100},
  /*02*/{     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100},
  /*03*/{     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,INFINITY,INFINITY,     100},
  /*04*/{     100,     100,     100,     100,     100,INFINITY,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100},
  /*05*/{     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,     100},
  /*06*/{     100,     100,     100,INFINITY,     100,     100,     100,     100,     100,INFINITY,     100,INFINITY,     100,     100,     100,     100,     100},
  /*07*/{     100,INFINITY,     100,INFINITY,     100,INFINITY,INFINITY,INFINITY,     100,INFINITY,INFINITY,INFINITY,     100,INFINITY,     100,INFINITY,INFINITY},
  /*08*/{     100,INFINITY,     100,     100,     100,     100,     100,INFINITY,     100,     100,     100,INFINITY,     100,     100,     100,     100,     100},
  /*09*/{     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,     100},
  /*10*/{     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100},
  /*11*/{     100,INFINITY,     100,INFINITY,     100,INFINITY,INFINITY,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,     100,INFINITY,     100},
  /*13*/{     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,     100,INFINITY,     100,     100,     100} 
};

bool isValid(int row, int col);
bool isUnBlocked(int grid[][COL], int row, int col);
bool isDestination(int row, int col, Pair dest);
double calculateHValue(int row, int col, Pair dest);
void tracePath(cell cellDetails[][COL], Pair dest);
void aStarSearch(int grid[][COL], Pair src, Pair dest);


double turn = 0;
void left_turn();
void right_turn();
void go_forward();
void stop();
int orientation=4;
int delay=0;
bool pass=true;
bool box=false;

DistanceSensor*junc_s[2];
char junc_s_names[2][5]={"L_J","R_J"};
int J[2]={0,0};

DistanceSensor*wb_s[2];
char wb_s_names[2][6]={"L_WB","R_WB"};
int WB[2]={0,0};

Camera *camera[3];
char camera_names[3][5]={"F_C","L_C","R_C"};
int rgb[3][3]={{0,0,0},{0,0,0},{0,0,0}};
bool barrier[3]={false,false,false};
void color_test();

Camera *box_cam;
void get_box_color();
int box_color=-1;
bool red_box=false;
bool green_box=false;
bool blue_box=false;
int box_node[2]={0,0};

//________________________________achintha 


int main(int argc, char** argv) {

    Motor*stage_motor;
    stage_motor = robot->getMotor("stage_motor");
    stage_motor->setPosition(INFINITY);
    stage_motor->setVelocity(1);
    stage_motor->setPosition(3.1416);
    
    Motor*linear_motor;
    linear_motor = robot->getMotor("linear_motor");
    linear_motor->setPosition(0.0);
    
    //////////////////front falp
    
    Motor*front_servo;
    front_servo = robot->getMotor("front_servo");
    front_servo->setPosition(0.0);
    //front_servo->setVelocity(0.1);
    
    Motor*front_flap;
    front_flap = robot->getMotor("front_flap");
    //front_flap->setVelocity(0.1);
    front_flap->setPosition(0.00);
    
    Motor*front_servo_1;
    front_servo_1 = robot->getMotor("front_servo_1");
    front_servo_1->setPosition(0.0);
    //front_servo->setVelocity(0.1);
    
    Motor*front_flap_1;
    front_flap_1 = robot->getMotor("front_flap_1");
    front_flap_1->setVelocity(0.1);
    front_flap_1->setPosition(0.00);

    //////////////////left falp
    
    Motor*left_servo;
    left_servo = robot->getMotor("left_servo");
    left_servo->setPosition(0.0);
    //front_servo->setVelocity(0.1);
    
    Motor*left_flap;
    left_flap = robot->getMotor("left_flap");
    left_flap->setPosition(0.0);
    
    Motor*left_servo_1;
    left_servo_1 = robot->getMotor("left_servo_1");
    left_servo_1->setPosition(0.0);
    //front_servo->setVelocity(0.1);
    
    Motor*left_flap_1;
    left_flap_1 = robot->getMotor("left_flap_1");
    left_flap_1->setPosition(0.0);
    
    //////////////////right falp
    
    Motor*right_servo;
    right_servo = robot->getMotor("right_servo");
    right_servo->setPosition(0.0);
    //front_servo->setVelocity(0.1);
    
    Motor*right_flap;
    right_flap = robot->getMotor("right_flap");
    right_flap->setPosition(0.0);
    
    Motor*right_servo_1;
    right_servo_1 = robot->getMotor("right_servo_1");
    right_servo_1->setPosition(0.0);
    //front_servo->setVelocity(0.1);
    
    Motor*right_flap_1;
    right_flap_1 = robot->getMotor("right_flap_1");
    right_flap_1->setPosition(0.0);
    
    //////////////////rear falp
    
    Motor*rear_flap;
    rear_flap = robot->getMotor("rear_flap");
    rear_flap->setPosition(0.0);
    
    Motor*rear_flap_1;
    rear_flap_1 = robot->getMotor("rear_flap_1");
    rear_flap_1->setPosition(0.0);
    
    for (int i = 0; i < 2; i++) {
        wheels[i] = robot->getMotor(wheels_names[i]);
        wheels[i]->setPosition(INFINITY);
        wheels[i]->setVelocity(0);
    }
    for (int i = 0; i < 6; i++) {
        object[i] = robot->getDistanceSensor(objectSensors[i]);
        object[i]->enable(TIME_STEP);
    }
    for (int i = 0; i < 8; i++) {
        F_S[i] = robot->getDistanceSensor(LineSensors[i]);
        F_S[i]->enable(TIME_STEP);   
    }
    for (int i = 0; i < 2; i++) {
        junc_s[i] = robot->getDistanceSensor(junc_s_names[i]);
        junc_s[i]->enable(TIME_STEP);
        
    }
    for (int i = 0; i < 2; i++) {
        wb_s[i] = robot->getDistanceSensor(wb_s_names[i]);
        wb_s[i]->enable(TIME_STEP);
        
    }
    
    
    encoders[0] = robot->getPositionSensor(Encoder_names[0]);
    encoders[0]->enable(TIME_STEP);
    encoders[1] = robot->getPositionSensor(Encoder_names[1]);
    encoders[1]->enable(TIME_STEP);
    transferCoff();

    for(short i=0; i<3; i++){
    camera[i] = robot->getCamera(camera_names[i]);
    camera[i]->enable(TIME_STEP);
    }
    
    box_cam = robot->getCamera("box_camera");
    box_cam->enable(TIME_STEP);
    
    int grid[ROW][COL]
        = { /*00*/ { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
            /*01*/ { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1 },
            /*02*/ { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
            /*03*/ { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1 },
            /*04*/ { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
            /*05*/ { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1 },
            /*06*/ { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
            /*07*/ { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1 },
            /*08*/ { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
            /*09*/ { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1 },
            /*10*/ { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
            /*11*/ { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1 },
            /*12*/ { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } };
            /*       0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16*/
    // Source is the left-most bottom-most corner
    //Pair src = make_pair(8, 7);
 
    // Destination is the left-most top-most corner
    //Pair dest = make_pair(2, 14);
 
    //aStarSearch(grid, src, dest);
    
    //for (int  i=0; i<60; i++){
    //  cout<<"("<<route[i][0]<<","<<route[i][1]<<") , ";
    //}
    //cout<<endl;
    
    while (robot->step(TIME_STEP) != -1) {
        getLineSensors();
        L_Pos = encoders[0]->getValue();
        R_Pos = encoders[1]->getValue();
        cout<<"stage="<<stage<<"   substage="<<substage<<"   orientation="<<orientation<<"  R_Pos="<<R_Pos<<" L_Pos="<<L_Pos;
        cout<<"  current node - ("<<current_node[0]<<","<<current_node[1]<<")";
        cout<<"  middle node - ("<<middle_node[0]<<","<<middle_node[1]<<")";
        cout<<"  target node - ("<<target[0]<<","<<target[1]<<")";
        cout<<endl;
        //PID();
        if (stage==0){
          if(S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
            PID();
          }else{
            stage+=1;
          }
        }else if (stage==1){
          if (substage==0) {
            if (S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1){
              substage = 1;
              //turn=R_Pos+8.9759;
              wheels[0]->setVelocity(0);
              wheels[1]->setVelocity(0);
            }else{
              PID();
            }
          }else if (substage==1) {
            if (J[0]==1 or J[1]==1){
              stop();
              substage=2;
            }else{
              wheels[0]->setVelocity(3);
              wheels[1]->setVelocity(3);
            }
          }else if (substage==2){
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=3; 
                    turn=0;
                    //orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
          }else if (substage==3){
            if (S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
              substage = 4;
              //turn=R_Pos+8.9759;
              wheels[0]->setVelocity(0);
              wheels[1]->setVelocity(0);
            }else{
              PID();
            }
          }else if (substage==4){
            front_servo_1->setPosition(1.6580);//1.57
            left_servo_1->setPosition(1.6580);
            right_servo_1->setPosition(1.6580);
            substage=5;
          }else if (substage==5){
            front_flap_1->setPosition(0.02);
            left_flap_1->setPosition(0.07);
            right_flap_1->setPosition(0.07);
            rear_flap_1->setPosition(0.02);
            substage=6;
          }else if (substage==6){
            if (delay==50){
              delay=0;
              substage=7;
            }else{
             delay+=1;
            }
          }else if (substage==7){
            linear_motor->setPosition(0.04);
            substage=8;
          }else if (substage==8){
            stop();
            if (delay==50){
              delay=0;
              substage=9;
            }else{
             delay+=1;
            }
          }else if (substage==9){
            stop();
            stage_motor->setPosition(0.0);
            substage=10;
          }else if (substage==10){
            stop();
            if (delay==50){
              delay=0;
              substage=11;
            }else{
             delay+=1;
            }
          }else if (substage==11){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=12; 
                      turn=0;
                      //orientation=((orientation-1)%4)+4;
                      //orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
          }else if(substage==12){
            if (S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
              substage = 13;
              //turn=R_Pos+8.9759;
              wheels[0]->setVelocity(0);
              wheels[1]->setVelocity(0);
            }else{
              PID();
            }
          }else if (substage==13){
            if (J[0]==1 or J[1]==1){
              stop();
              substage=14;
            }else{
              wheels[0]->setVelocity(3);
              wheels[1]->setVelocity(3);
            }
          }else if (substage==14){
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=0;
                    stage=3; 
                    turn=0;
                    //orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
          }
        }else if (stage==2){
        
          cout<<"picking white box"<<endl;
          
        }else if(stage==3){
          //if(S[1]==1 and S[2]==1 and S[3]==1 and S[4]){
          //if(J[0]==1 or J[1]==1){
            //wheels[0]->setVelocity(0);
            //wheels[1]->setVelocity(0);
            //stage=4;
          //}else{
            //PID();
          //}
          if (substage==0){
            if(S[1]==1 and S[2]==1 and S[3]==1 and S[4]){
              wheels[0]->setVelocity(0);
              wheels[1]->setVelocity(0);
              substage=1;
            }else{
              PID();
            }
          }else if(substage==1){
            if(J[0]==1 or J[1]==1){
              wheels[0]->setVelocity(0);
              wheels[1]->setVelocity(0);
              stage=4;
              substage=0;
            }else{
              wheels[0]->setVelocity(3);
              wheels[1]->setVelocity(3);
            
            }
          }
        }else if(stage==4){///maze solving
          stage=500;
        
        }
        else if(stage==500){
          
          if (current_node[0]==12 and current_node[1]==16){
          //if (current_node[0]==6 and current_node[1]==10){
            cout<<"ended"<<endl;
            stage+=1; substage=0;
          }else{
      
            if (substage==0){
              
              if ( current_node[0]%4==0 ){
                //_________________________________forward_raw_________
                if (current_node[1]<16){
                  target[0]=current_node[0];
                  target[1]=current_node[1]+2;
                  middle_node[0]=current_node[0];
                  middle_node[1]=current_node[1]+1;
                }else{
                  target[0]=current_node[0]+2;
                  target[1]=current_node[1];
                  middle_node[0]=current_node[0]+1;
                  middle_node[1]=current_node[1];
                }
                //substage=1;
              
              }else if (current_node[0]%4==2){
                //___________________________________riverse row_________
                if (current_node[1]>0){
                  target[0]=current_node[0];
                  target[1]=current_node[1]-2;
                  middle_node[0]=current_node[0];
                  middle_node[1]=current_node[1]-1;
                }else{
                  target[0]=current_node[0]+2;
                  target[1]=current_node[1];
                  middle_node[0]=current_node[0]+1;
                  middle_node[1]=current_node[1];
                }
                //substage=1;
                
                
              }else{
                cout<<"error"<<endl;
              }
              destination[0]=target[0]; destination[1]=target[1];
              cout<<"current node - ("<<current_node[0]<<","<<current_node[1]<<")";
              cout<<"  middle node - ("<<middle_node[0]<<","<<middle_node[1]<<")";
              cout<<"  target node - ("<<target[0]<<","<<target[1]<<")";
              cout<<endl;
              
              if (grid[target[0]][target[1]]==-5){
                stop();
                if ( current_node[0]%4==0 ){
                //_________________________________forward_raw_________
                if (current_node[1]<14){
                  target[0]=target[0];
                  target[1]=target[1]+2;
                  //middle_node[0]=current_node[0];
                  //middle_node[1]=current_node[1]+1;
                }else{
                  target[0]=target[0]+2;
                  target[1]=target[1];
                  //middle_node[0]=current_node[0]+1;
                  //middle_node[1]=current_node[1];
                }
                //substage=1;
              
                }else if (current_node[0]%4==2){
                  //___________________________________riverse row_________
                  if (current_node[1]>2){
                    target[0]=target[0];
                    target[1]=target[1]-2;
                    //middle_node[0]=current_node[0];
                    //middle_node[1]=current_node[1]-1;
                  }else{
                    target[0]=target[0]+2;
                    target[1]=target[1];
                    //middle_node[0]=current_node[0]+1;
                    //middle_node[1]=current_node[1];
                  }
                  //substage=1;
                               
                }
                substage=10;
                destination[0]=target[0];
                destination[1]=target[1];
                node=2;
              }else{
                substage=1;
              }
            }//substage 0 ends here
            else if (substage==1){
              //current_node[0]=target[0];
              //current_node[1]=target[1];
              //substage=0;
              color_test();
              //updating neighbours
              if ( current_node[0]%4==0 ){
                //_________________________________forward_raw_________
                if (current_node[1]<16){
                  if (barrier[0]){//front neighbour
                    grid[current_node[0]][current_node[1]+1]=0;
                  }
                  if (current_node[0]>0 and barrier[2]){//down neighbour
                    grid[current_node[0]-1][current_node[1]]=0;
                  }
                  if (current_node[0]<12 and barrier[1]){//up neighbour
                    grid[current_node[0]+1][current_node[1]]=0;
                  }
                }else{
                  if (current_node[0]>0 and barrier[2]){//down neighbour
                    grid[current_node[0]-1][current_node[1]]=0;
                  }
                  if (current_node[0]<12 and barrier[1]){//up neighbour
                    grid[current_node[0]+1][current_node[1]]=0;
                  }
                }
              }else if (current_node[0]%4==2){
                //___________________________________riverse row_________
                if (current_node[1]>0){
                  if (barrier[0]){//front neighbour
                    grid[current_node[0]][current_node[1]-1]=0;
                  }
                  if (current_node[0]>0 and barrier[1]){//down neighbour
                    grid[current_node[0]-1][current_node[1]]=0;
                  }
                  if (current_node[0]<12 and barrier[2]){//up neighbour
                    grid[current_node[0]+1][current_node[1]]=0;
                  }
                  
                }else{
                  if (current_node[0]>0 and barrier[1]){//down neighbour
                    grid[current_node[0]-1][current_node[1]]=0;
                  }
                  if (current_node[0]<12 and barrier[2]){//up neighbour
                    grid[current_node[0]+1][current_node[1]]=0;
                  }
                }
                
              }
              
              //going to next node
              if(orientation==4){
                if (current_node[0]==target[0]){
                //check front cam
                  if (barrier[0]){ substage=10;}
                  else {substage=2;}
                }else if (current_node[0]<target[0]){
                //check left cam
                  if (barrier[1]){ substage=10;}
                  else {substage=2;}
                }else if (current_node[0]>target[0]){
                //check right cam
                  if (barrier[2]){ substage=10;}
                  else {substage=2;}
                }
                
              }else if (orientation==5){
                if (current_node[1]==target[1]){
                //check front cam
                  if (barrier[0]){ substage=10;}
                  else {substage=2;}
                }else if (current_node[1]<target[1]){
                //check left cam
                  if (barrier[1]){ substage=10;}
                  else {substage=2;}
                }else if (current_node[1]>target[1]){
                //check right cam
                  if (barrier[2]){ substage=10;}
                  else {substage=2;}
                }
                
              }else if (orientation==6){
                if (current_node[0]==target[0]){
                //check front cam
                  if (barrier[0]){ substage=10;}
                  else {substage=2;}
                }else if (current_node[0]<target[0]){
                //check right cam
                  if (barrier[2]){ substage=10;}
                  else {substage=2;}
                }else if (current_node[0]>target[0]){
                //check left cam
                  if (barrier[1]){ substage=10;}
                  else {substage=2;}
                }
              }else if (orientation==7){
                if (current_node[0]==target[0]){
                //check front cam
                  if (barrier[0]){ substage=10;}
                  else {substage=2;}
                }else if (current_node[0]<target[0]){
                //check right cam
                  if (barrier[2]){ substage=10;}
                  else {substage=2;}
                }else if (current_node[0]>target[0]){
                //check left cam
                  if (barrier[1]){ substage=10;}
                  else {substage=2;}
                }
              }
              
            
            }//substage 1 ends here
            else if (substage==2){
              if(orientation==4){
                if (current_node[0]==target[0]){
                //check front cam
                //cout<<"go forward"<<endl;
                  if(current_node[1]<target[1]){
                    if (J[0]==1 or J[1]==1){
                      PID();
                    }else{
                      stop();
                      substage=3;
                    }
                  }else if (current_node[1]>target[1]){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=3; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
                  }
                }else if (current_node[0]<target[0]){
                //check left cam
                //cout<<"turn left"<<endl;
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=3; 
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
                  
                }else if (current_node[0]>target[0]){
                //check right cam
                //cout<<"turn right"<<endl;
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=3; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                  right_turn();
                  }
                  
                }
                
              }else if (orientation==5){
                if (current_node[1]==target[1]){
                //check front cam
                  if(current_node[0]>target[0]){
                    if (J[0]==1 or J[1]==1){
                      PID();
                    }else{
                      stop();
                      substage=3;
                    }
                  }else if (current_node[0]<target[0]){
                    if (turn==0){
                      turn=R_Pos+7.854;}//4.7124*2
                    if (turn==-5){
                      substage=3; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
                  }
                  
                }else if (current_node[1]<target[1]){
                //check left cam
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=3; 
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
                  
                }else if (current_node[1]>target[1]){
                //check right cam
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=3; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                  right_turn();
                  }
                }
                
              }else if (orientation==6){
                if (current_node[0]==target[0]){
                //check front cam
                  
                  if(current_node[1]>target[1]){
                    if (J[0]==1 or J[1]==1){
                      PID();
                    }else{
                      stop();
                      substage=3;
                    }
                  }else if (current_node[1]<target[1]){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=3; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
                  }
                  
                }else if (current_node[0]<target[0]){
                //check right cam
                //cout<<"turn right"<<endl;
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=3; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                  right_turn();
                  }
                  
                }else if (current_node[0]>target[0]){
                //check left cam
                //cout<<"turn left"<<endl;
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=3; 
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
                  
                }
              }else if (orientation==7){
                if (current_node[1]==target[1]){
                //check front cam
                  
                  if(current_node[0]<target[0]){
                    if (J[0]==1 or J[1]==1){
                      PID();
                    }else{
                      stop();
                      substage=3;
                    }
                  }else if (current_node[0]>target[0]){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=3; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
                  }
                  
                }else if (current_node[1]<target[1]){
                //check right cam
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=3; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                    right_turn();
                  }
                  
                }else if (current_node[1]>target[1]){
                //check left cam
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=3; 
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
                  
                }
              }
            
            }//substage 2 ends here
            else if (substage==3){
              //if(S[1]==1 and S[2]==1 and S[3]==1 and S[4]){
              if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
                wheels[0]->setVelocity(0);
                wheels[1]->setVelocity(0);
                substage=4;
              }else if((S[0]==1 and S[1]==1) and (S[6]==1 and S[7]==1)){
                stop();
                substage=40;
              }else{
                PID();
              }
            }else if (substage==4){
              if(J[0]==1 or J[1]==1){
                stop();
                if (S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
                  stop();
                  last_visited[0]=current_node[0];
                  last_visited[1]=current_node[0];
                  substage=150;
                }else{
                  current_node[0]=target[0];
                  current_node[1]=target[1];
                  substage=5;
                }
              }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
                //stop();
              }
            }//substage 4 ends here
            else if (substage==5){
              if (current_node[0]%4==0){//forward line
                if (orientation==7){
                // turn right
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=0; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                    right_turn();
                  }
                }else if (orientation==5){
                // turn left
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=0; 
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
                }else{
                  substage=0;
                }
              }else if (current_node[0]%4==2){//riverse line
                if (orientation==7){
                // turn left
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=0; 
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
                }else if (orientation==5){
                // turn right
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=0; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                    right_turn();
                  }
                }else{
                  substage=0;
                }
              }else{
                substage=0;
              }
            }//substage 5 ends here
            else if(substage==150){
              box=false;
              stop();
              for (int i=0; i<6; i++){
                //O[i] = (object[i]->getValue() < TH);
                cout<<object[i]->getValue()<<" , ";
                if(object[i]->getValue()<1000){
                  box=true;
                  
                }
              }
              cout<<endl;
              
              if (box){
                cout<<"treasure found"<<endl;
                substage=155;
              }else {
                linear_motor->setPosition(0.0);
                substage=151;
              }
                          
            }//substage 150 ends here
            else if(substage==151){
              if (delay==70){
                delay=0;
                substage=152;
              }else{
               delay+=1;
              }
            }
            else if(substage==152){
              stop();
              for (int i=0; i<6; i++){
                //O[i] = (object[i]->getValue() < TH);
                cout<<object[i]->getValue()<<" , ";
                if(object[i]->getValue()<1000){
                  box=true;
                  
                }
              }
              cout<<endl;
              if (box){
                substage=155;
              }else{
                substage=450;
                //just proceeding stages
              }
            }
            else if(substage==155){
              front_servo->setPosition(1.6580);//1.6580//1.57
              left_servo->setPosition(1.8);
              right_servo->setPosition(1.8);
              substage=156;
            }else if(substage==156){
              front_flap->setPosition(0.02);
              left_flap->setPosition(0.06);
              right_flap->setPosition(0.06);
              rear_flap->setPosition(0.02);
              substage=157;
            }else if(substage==157){
              if (delay==50){
                delay=0;
                substage=158;
              }else{
                 delay+=1;
              }
            }else if(substage==158){
              stop();
              //color test of box
              get_box_color();
              substage=159;
            }else if(substage==159){
              front_flap->setPosition(0.0);
              left_flap->setPosition(0.0);
              right_flap->setPosition(0.0);
              rear_flap->setPosition(0.0);
              substage=160;
            }else if(substage==160){
              if (delay==50){
                delay=0;
                //substage=substage+box_color;// 161 red // 162 green // 163 blue // 164 white
                if(red_box and box_color==1){/*mark as barrier and proceed*/}
                else if(green_box and box_color==2){/*mark as barrier and proceed*/}
                else if(blue_box and box_color==3){/*mark as barrier and proceed*/}
                else{substage=substage+box_color;/* 161 red // 162 green // 163 blue // 164 white*/}
              }else{
                 delay+=1;
              }
            }else if(substage==161){
              front_flap->setPosition(0.02);
              left_flap->setPosition(0.06);
              right_flap->setPosition(0.06);
              rear_flap->setPosition(0.02);
              substage=165;
            }else if(substage==162){
              front_flap->setPosition(0.02);
              left_flap->setPosition(0.07);
              right_flap->setPosition(0.07);
              rear_flap->setPosition(0.02);
              substage=165;
            }else if(substage==163){
              front_flap->setPosition(0.02);
              left_flap->setPosition(0.08);//0.07
              right_flap->setPosition(0.08);
              rear_flap->setPosition(0.02);
              substage=165;
            }else if(substage==164){
              if(pass){
                front_flap->setPosition(0.02);
                left_flap->setPosition(0.07);
                right_flap->setPosition(0.07);
                rear_flap->setPosition(0.02);
                substage=250;
                linear_motor->setPosition(0.04);
              }else{
 ///////////////////  //pick from other arm
                 front_servo->setPosition(0.0);//1.6580//1.57
                 left_servo->setPosition(0.0);
                 right_servo->setPosition(0.0);
                 linear_motor->setPosition(0.04);
                 substage=269;
                 stop();
              }
            }else if(substage==250){
              if (delay==80){
                delay=0;
                substage=251;
              }else{
                 delay+=1;
              }
            }else if(substage==251){
              if(J[0]==1 or J[1]==1){
              //if(S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
              }else{
                stop();
                substage=252;
                color_test();
              }
            }else if(substage==252){
              if(barrier[0]){
      //////////////////////////////barrier
                stop();
                substage=281;
              }else{
                substage=253;
              }
            }else if(substage==253){//no barrier
              stage_motor->setPosition(3.1416);
              substage=254;
            }else if(substage==254){
              if (delay==100){
                delay=0;
                substage=255;
              }else{
                delay+=1;
              }
            }else if(substage==255){
              //linear_motor->setPosition(0.0);
              front_flap->setPosition(0.0);
              left_flap->setPosition(0.0);
              right_flap->setPosition(0.0);
              rear_flap->setPosition(0.0);
              front_servo->setPosition(0);//1.57
              left_servo->setPosition(0);
              right_servo->setPosition(0);
              substage=256;
              stop();
              stage_motor->setPosition(0.0);
            }else if(substage==256){
              current_node[0]=target[0];
              current_node[1]=target[1];
              substage=0;
            }else if(substage==269){
              if (delay==100){
                delay=0;
                substage=270;
              }else{
               delay+=1;
              }
            }else if(substage==270){
              if(S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
                wheels[0]->setVelocity(-3);
                wheels[1]->setVelocity(-3);
              }else{
                stop();
                substage=271;
                linear_motor->setPosition(0.0);
                stage_motor->setPosition(3.1416);
              }
            }else if(substage==271){
              if (delay==100){
                delay=0;
                substage=272;
              }else{
                delay+=1;
              }
            }else if(substage==272){
              if(J[0]==1 and J[1]==1){
              //if(S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
              }else{
                stop();
                substage=273;
                linear_motor->setPosition(0.0);
              }
            }else if(substage==273){
              front_servo_1->setPosition(1.6580);//1.57
              left_servo_1->setPosition(1.6580);
              right_servo_1->setPosition(1.6580);
              substage=274;
            }else if(substage==274){
              stop();
              if (delay==10){
                delay=0;
                substage=275;
              }else{
               delay+=1;
              }
            }else if(substage==275){
              front_flap_1->setPosition(0.02);
              left_flap_1->setPosition(0.07);
              right_flap_1->setPosition(0.07);
              rear_flap_1->setPosition(0.02);
              substage=276;
              pass=true;
            }else if(substage==276){
              stop();
              if (delay==10){
                delay=0;
                substage=277;
                linear_motor->setPosition(0.04);
                stage_motor->setPosition(0.0);
              }else{
               delay+=1;
              }
            }else if(substage==277){
              if (delay==100){
                delay=0;
                substage=278;
              }else{
                delay+=1;
              }
            }else if(substage==278){
              if(J[0]==1 or J[1]==1) {
                stop();
                color_test();
                substage=279;
              }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
              }
            }else if(substage==279){
              if(J[0]==1 or J[1]==1) {
                
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
              }else{
                stop();
                color_test();
                substage=280;
              }
            }else if(substage==280){
              if(barrier[0]){
              ////////////////////barrier
              substage=281;
              stop();
              }else{
                current_node[0]=target[0];
                current_node[1]=target[1];
                substage=0;
              }
            }else if(substage==281){
              if(J[0]==1 or J[1]==1){
                stop();
                substage=282;
              }else{
                wheels[0]->setVelocity(-3);
                wheels[1]->setVelocity(-3);
              }
            }else if(substage==282){
              if(J[0]==1 or J[1]==1){
                
                wheels[0]->setVelocity(-3);
                wheels[1]->setVelocity(-3);
              }else{
                stop();
                substage=283;
              }
            }else if(substage==283){
              if(S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
                wheels[0]->setVelocity(-3);
                wheels[1]->setVelocity(-3);
              }else{
                stop();
                substage=284;
                stop();
                
                grid[target[0]][target[1]]=-8;
                front_flap->setPosition(0.0);
                left_flap->setPosition(0.0);
                right_flap->setPosition(0.0);
                rear_flap->setPosition(0.0);
                front_servo->setPosition(0.0);//1.57
                left_servo->setPosition(0.0);
                right_servo->setPosition(0.0);
              }
            }else if(substage==284){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=285; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
            }else if(substage==285){
              if(S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
                stop();
                substage=286;
              }else{
                PID();
              }
            }else if(substage==286){
              if(J[0]==1 or J[1]==1){
                stop();
                substage=287;
              }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
              }
            }else if(substage==287){
              ////calculate target
                stop();
                if ( current_node[0]%4==0 ){
                //_________________________________forward_raw_________
                if (current_node[1]<14){
                  target[0]=target[0];
                  target[1]=target[1]+2;
                  //middle_node[0]=current_node[0];
                  //middle_node[1]=current_node[1]+1;
                }else{
                  target[0]=target[0]+2;
                  target[1]=target[1];
                  //middle_node[0]=current_node[0]+1;
                  //middle_node[1]=current_node[1];
                }
                //substage=1;
              
                }else if (current_node[0]%4==2){
                  //___________________________________riverse row_________
                  if (current_node[1]>2){
                    target[0]=target[0];
                    target[1]=target[1]-2;
                    //middle_node[0]=current_node[0];
                    //middle_node[1]=current_node[1]-1;
                  }else{
                    target[0]=target[0]+2;
                    target[1]=target[1];
                    //middle_node[0]=current_node[0]+1;
                    //middle_node[1]=current_node[1];
                  }
                  //substage=1;
                               
                }
                substage=10;
                destination[0]=target[0];
                destination[1]=target[1];
                node=2;
            }else if(substage==165){
              
              if (delay==80){
                delay=0;
                substage=166;
              }else{
                 delay+=1;
              }
            
            }else if(substage==166){
              linear_motor->setPosition(0.08);
              substage=167;
            }else if(substage==167){
              if(S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
                wheels[0]->setVelocity(-2);
                wheels[1]->setVelocity(-2);
              }else{
                stop();
                substage=168;
              }
                 
            }else if(substage==168){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=169; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
            }else if(substage==169){
              //linear_motor->setPosition(0.04);
              substage=170;
            }else if (substage==170){
              stop();
              if (delay==75){
                delay=0;
                substage=171;
              }else{
               delay+=1;
              }
            }else if(substage==171){
              if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
                wheels[0]->setVelocity(0);
                wheels[1]->setVelocity(0);
                substage=172;
              }else if((S[0]==1 and S[1]==1) and (S[6]==1 and S[7]==1)){
                stop();
                substage=175;
              }else{
                PID();
              }
            }else if(substage==172){
              if(J[0]==1 or J[1]==1){
                stop();
                substage=180;
              }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
              }
            }else if(substage==175){
              if(J[0]==1 and J[1]==1){
                stop();
                substage=176;
              }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
              }
            }else if(substage==176){
              if(J[0]==0 or J[1]==0){
                stop();
                substage=180;
              }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
              }
            }else if(substage==180){
              last_visited[0]=current_node[0];
              last_visited[1]=current_node[1];
              box_node[0]=target[0];
              box_node[1]=target[1];
              //current_node[0]=target[0];
              //current_node[1]=target[1];
              destination[0]=0;
              destination[1]=0;
              grid[box_node[0]][box_node[1]]=-8;
              node=2;
              substage=10;
              
            }else if(substage==190){
              stop();
              grid[box_node[0]][box_node[1]]=8;
              if(orientation==5){
                //turn right
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=12; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                  right_turn();
                  }
              }else{
                //orientation=4;
                substage=191;
              }
            }else if(substage==191){
              if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
                wheels[0]->setVelocity(0);
                wheels[1]->setVelocity(0);
                substage=192;
              }else{
                PID();
              }
            }else if(substage==192){
              if(J[0]==1 or J[1]==1){
                stop();
                if(box_color==1){
                  substage=193;
                }else{
                  substage=210;
                }
              }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
              }
            }else if(substage==193){
              if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
                wheels[0]->setVelocity(0);
                wheels[1]->setVelocity(0);
                substage=194;
              }else{
                PID();
              }
            }else if(substage==194){
              if(J[0]==1 and J[1]==1){
                stop();
                substage=195;
              }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
              }
            }else if(substage==195){
              linear_motor->setPosition(0.0);
              front_flap->setPosition(0.0);
              left_flap->setPosition(0.0);
              right_flap->setPosition(0.0);
              rear_flap->setPosition(0.0);
              front_servo->setPosition(0);//1.57
              left_servo->setPosition(0);
              right_servo->setPosition(0);
              red_box=true;
              substage=196;
            }else if(substage==196){
              stop();
              if (delay==70){
                delay=0;
                substage=197;
                linear_motor->setPosition(0.04);
              }else{
               delay+=1;
              }
            }else if(substage==197){
              //if (J[0]==1 or J[1]==1){
              if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
                wheels[0]->setVelocity(-3);
                wheels[1]->setVelocity(-3);
              }else{
                stop();
                substage=198;
              }
            }else if(substage==198){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=199; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
            }else if(substage==199){
              if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
                wheels[0]->setVelocity(0);
                wheels[1]->setVelocity(0);
                substage=200;
              }else{
                PID();
              }
            }else if(substage==200){
              if(J[0]==1 or J[1]==1){
                stop();
                if(red_box and green_box and blue_box){
                  stage=600;
                  substage=0;
                }else{
                  substage=201;
                }
              }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
              }
         ///////////////////////////////goig back to maze
            }else if(substage==201){//////////////////
              if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
                wheels[0]->setVelocity(0);
                wheels[1]->setVelocity(0);
                substage=202;
              }else{
                PID();
              }
            }else if(substage==202){
              if(J[0]==1 or J[1]==1){
                stop();
                substage=203;
              }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
              }
            }else if(substage==203){
              box=false;
              current_node[0]=0;
              current_node[1]=0;
              node=2;
              destination[0]=last_visited[0];
              destination[1]=last_visited[1];
              substage=10;
            }else if(substage==210){
              //turn right
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=211; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                  right_turn();
                  }
            }else if(substage==211){
              if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
                wheels[0]->setVelocity(0);
                wheels[1]->setVelocity(0);
                substage=213;
              }else{
                PID();
              }
            }else if(substage==213){
              if(J[0]==1 and J[1]==1){
                stop();
                //substage=214;
                if (box_color==2){//if box is green
                  substage=214;
                }else{
                  substage=230;
                }
              }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
              }
            }else if(substage==214){
              linear_motor->setPosition(0.0);
              front_flap->setPosition(0.0);
              left_flap->setPosition(0.0);
              right_flap->setPosition(0.0);
              rear_flap->setPosition(0.0);
              front_servo->setPosition(0);//1.57
              left_servo->setPosition(0);
              right_servo->setPosition(0);
              green_box=true;
              substage=215;
            }else if(substage==215){
              stop();
              if (delay==70){
                delay=0;
                substage=216;
                linear_motor->setPosition(0.04);
              }else{
               delay+=1;
              }
            }else if(substage==216){
              if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
                wheels[0]->setVelocity(-3);
                wheels[1]->setVelocity(-3);
              }else{
                stop();
                substage=217;
              }
            }else if(substage==217){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=218; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
            }else if(substage==218){
              if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
                //wheels[0]->setVelocity(3);
                //wheels[1]->setVelocity(3);
                stop();
                substage=219;
              }else{
                PID();
              }
            }else if(substage==219){
              if(J[0]==1 or J[1]==1){
                stop();
                substage=220;
              }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
              }
            }else if(substage==220){
              //left turn
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    //substage=201;
                    if(red_box and green_box and blue_box){
                      stage=600;
                      substage=0;
                    }else{
                      substage=201;
                    } 
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
            }else if(substage==230){
              stage_motor->setPosition(1.5708);
              substage=231;
            }else if(substage==231){
              stop();
              if (delay==80){
                delay=0;
                substage=232;
              }else{
               delay+=1;
              }
            }else if(substage==232){
              linear_motor->setPosition(0.0);
              front_flap->setPosition(0.0);
              left_flap->setPosition(0.0);
              right_flap->setPosition(0.0);
              rear_flap->setPosition(0.0);
              front_servo->setPosition(0);//1.57
              left_servo->setPosition(0);
              right_servo->setPosition(0);
              blue_box=true;
              substage=233;
            }else if(substage==233){
              stop();
              if (delay==70){
                delay=0;
                substage=234;
                linear_motor->setPosition(0.04);
              }else{
               delay+=1;
              }
            }else if(substage==234){
              if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
                wheels[0]->setVelocity(-3);
                wheels[1]->setVelocity(-3);
              }else{
                stop();
                substage=217;
                stage_motor->setPosition(0);
              }
            }
            
            else if(substage==40){
              if (grid[target[0]][target[1]]>1){
                //go forward
                //stop();
                //cout<<"here"<<endl;
                if (turn==0){
                    turn=L_Pos+7.8540;}//4.7124
                  if (turn==-5){
                    substage=0; 
                    turn=0;
                    stop();
                    current_node[0]=target[0];
                    current_node[1]=target[1];
                    //orientation=((orientation+1)%4)+4;
                  }else {
                    go_forward();
                  }
              }else{
                if (pass){
                  if (current_node[0]==12){
                    stop();
                    stage_motor->setPosition(-1.5708);
                  }else{
                    stage_motor->setPosition(-1.5708);
                    stop();
                  }
                
                  substage=41;
                  linear_motor->setPosition(0.0);
                }else{
                  stop();
                  grid[target[0]][target[1]]=-5;
                  //map as an obstacle
                  substage=45;
                }
              }
            }else if(substage==41){
              if (delay==60){
                delay=0;
                substage=42;
              }else{
               delay+=1;
              }
            }else if(substage==42){
              front_flap_1->setPosition(0.0);
              left_flap_1->setPosition(0.0);
              right_flap_1->setPosition(0.0);
              rear_flap_1->setPosition(0.0);
              
              front_servo_1->setPosition(0);//1.57
              left_servo_1->setPosition(0);
              right_servo_1->setPosition(0);
              
              linear_motor->setPosition(0.04);
                            
              substage=43;
              grid[target[0]][target[1]]=5;
            }else if(substage==43){
              if (delay==40){
                delay=0;
                substage=44;
              }else{
               delay+=1;
              }
            }else if(substage==44){
              stage_motor->setPosition(0.0);
              substage=40;
              pass=false;
            }else if(substage==45){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=46; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
            }else if(substage==46){
              if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
                wheels[0]->setVelocity(0);
                wheels[1]->setVelocity(0);
                substage=47;
              }else{
                PID();
              }
            }else if(substage==47){
              if(J[0]==1 or J[1]==1){
                stop();
                //current_node[0]=target[0];
                //current_node[1]=target[1];
                substage=10;
                if(current_node[0]%4==0){
                  if(target[1]==12){
                    target[0]=target[0]+2;
                  }
                  target[1]=target[1]+2;
                }else if (current_node[0]%4==2){
                  if(target[1]==4){
                    target[0]=target[0]+2;
                  }
                  target[1]=target[1]-2;
                }
                /*cout<<"current node - ("<<current_node[0]<<","<<current_node[1]<<")";
                cout<<"  middle node - ("<<middle_node[0]<<","<<middle_node[1]<<")";
                cout<<"  target node - ("<<target[0]<<","<<target[1]<<")";
                cout<<endl;
                for (int i=0;i<13;i++){
                  cout<<i<<"  -  ";
                  for (int j=0;j<17;j++){
                    cout<<grid[i][j]<<",";
                  }
                  cout<<endl;
                }*/
                destination[0]=target[0];
                destination[1]=target[1];
              }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
                //stop();
              }
            }
            
            else if (substage==10){
              stop();
              cout<<"barrier"<<endl;
              Pair src = make_pair(current_node[0], current_node[1]);
              Pair dest = make_pair(destination[0], destination[1]);
              aStarSearch(grid, src, dest);

              for (int  i=0; i<60; i++){
                cout<<"("<<route[i][0]<<","<<route[i][1]<<") , ";
              }
              cout<<endl;
              substage=11;
              /*for (int i=0;i<13;i++){
                  cout<<i<<"  -  ";
                  for (int j=0;j<17;j++){
                    cout<<grid[i][j]<<",";
                  }
                  cout<<endl;
                }*/
            }
            else if (substage==11){
              target[0]=route[node][0]; target[1]=route[node][1];
              
              if(orientation==4){
                if (current_node[0]==target[0]){
                //check front cam
                //cout<<"go forward"<<endl;
                  if(current_node[1]<target[1]){
                    if (J[0]==1 or J[1]==1){
                      PID();
                    }else{
                      stop();
                      substage=12;
                    }
                  }else if (current_node[1]>target[1]){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=12; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
                  }
                }else if (current_node[0]<target[0]){
                //check left cam
                //cout<<"turn left"<<endl;
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=12; 
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
                  
                }else if (current_node[0]>target[0]){
                //check right cam
                //cout<<"turn right"<<endl;
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=12; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                  right_turn();
                  }
                  
                }
                
              }else if (orientation==5){
                if (current_node[1]==target[1]){
                //check front cam
                  if(current_node[0]>target[0]){
                    if (J[0]==1 or J[1]==1){
                      PID();
                    }else{
                      stop();
                      substage=12;
                    }
                  }else if (current_node[0]<target[0]){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=12; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
                  }
                  
                }else if (current_node[1]<target[1]){
                //check left cam
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=12; 
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
                  
                }else if (current_node[1]>target[1]){
                //check right cam
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=12; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                  right_turn();
                  }
                }
                
              }else if (orientation==6){
                if (current_node[0]==target[0]){
                //check front cam
                  
                  if(current_node[1]>target[1]){
                    if (J[0]==1 or J[1]==1){
                      PID();
                    }else{
                      stop();
                      substage=12;
                    }
                  }else if (current_node[1]<target[1]){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=12; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
                  }
                  
                }else if (current_node[0]<target[0]){
                //check right cam
                //cout<<"turn right"<<endl;
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=12; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                  right_turn();
                  }
                  
                }else if (current_node[0]>target[0]){
                //check left cam
                //cout<<"turn left"<<endl;
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=12; 
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
                  
                }
              }else if (orientation==7){
                if (current_node[1]==target[1]){
                //check front cam
                  
                  if(current_node[0]<target[0]){
                    if (J[0]==1 or J[1]==1){
                      PID();
                    }else{
                      stop();
                      substage=12;
                    }
                  }else if (current_node[0]>target[0]){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=12; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
                  }
                  
                }else if (current_node[1]<target[1]){
                //check right cam
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=12; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                    right_turn();
                  }
                  
                }else if (current_node[1]>target[1]){
                //check left cam
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=12; 
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
                  
                }
              }
            
            }//substage 11 ends here
            else if (substage==12){
              color_test();
              if (barrier[0]){
                grid[route[node-1][0]][route[node-1][1]]=0;
                substage=10;
                node=2;
              }else{
                substage=13;
              }
            }
            else if (substage==13){
                if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
                //if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6] and S[7])){
                  wheels[0]->setVelocity(0);
                  wheels[1]->setVelocity(0);
                  substage=14;
                }else if((S[0]==1 and S[1]==1) and (S[6]==1 and S[7]==1)){
                stop();
                substage=80;
                }else{
                  PID();
                }
              
            }//substage 13 ends here
            else if (substage==14){
              if(J[0]==1 or J[1]==1){
                stop();
                node+=2;
                current_node[0]=target[0];
                current_node[1]=target[1];
                //substage=0;
                if (current_node[0]==destination[0] and current_node[1]==destination[1]){
                  //substage=15;
                  if(box){
                    substage=190;
                  }else{
                    substage=15;
                  }
                  for(int i=0; i<node+2; i++){
                    route[i][0]=-1; route[i][1]=-1;  
                  }
                  node=2;
                }else if(S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
                  last_visited[0]=route[0][0];
                  last_visited[1]=route[0][1];
                  substage=150;
                }else{
                  substage=11;
                }
              }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
                //stop();
              }
            }//substage 14 ends here
            else if (substage==15){
              if (current_node[0]%4==0){//forward line
                if (orientation==7){
                // turn right
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=0; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                    right_turn();
                  }
                }else if (orientation==5){
                // turn left
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=0; 
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
                }else if (orientation==6){
                // turn 180
                  if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=0; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
                }else if(orientation==4){
                  substage=0;
                }
              }else if (current_node[0]%4==2){//riverse line
                if (orientation==7){
                // turn left
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=0; 
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
                }else if (orientation==5){
                // turn right
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=0; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                    right_turn();
                  }
                }else if (orientation==4){
                // turn 180
                  if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=0;
                      /*if (box){
                        substage=190;
                      } else{
                        substage=0;
                      }*/
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
                }else if(orientation==6){
                  substage=0;
                }
              }
            }//substage 15 ends
                       
            //current_node[0]=target[0];
            //current_node[1]=target[1];
            else if (substage==450){
              if(J[0]==1 or J[1]==1){
                wheels[0]->setVelocity(-3);
                wheels[1]->setVelocity(-3);
              }else{
                stop();
                
                substage=451;
              }
            }else if (substage==451){
              if(S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
                wheels[0]->setVelocity(-3);
                wheels[1]->setVelocity(-3);
              }else{
                stop();
                substage=452;
                grid[target[0]][target[1]]=-7;
              }
            }else if(substage==452){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=453; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
              
            }else if(substage==453){
              if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
                stop();
                substage=454;
              }else{
                PID();
              }
            }else if(substage==454){
              if(J[0]==1 or J[1]==1){
                stop();
                substage=455;
              }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
              }
            }else if(substage==455){
              stop();
                if ( current_node[0]%4==0 ){
                //_________________________________forward_raw_________
                if (current_node[1]<14){
                  target[0]=target[0];
                  target[1]=target[1]+2;
                  //middle_node[0]=current_node[0];
                  //middle_node[1]=current_node[1]+1;
                }else{
                  target[0]=target[0]+2;
                  target[1]=target[1];
                  //middle_node[0]=current_node[0]+1;
                  //middle_node[1]=current_node[1];
                }
                //substage=1;
              
                }else if (current_node[0]%4==2){
                  //___________________________________riverse row_________
                  if (current_node[1]>2){
                    target[0]=target[0];
                    target[1]=target[1]-2;
                    //middle_node[0]=current_node[0];
                    //middle_node[1]=current_node[1]-1;
                  }else{
                    target[0]=target[0]+2;
                    target[1]=target[1];
                    //middle_node[0]=current_node[0]+1;
                    //middle_node[1]=current_node[1];
                  }
                  //substage=1;
                               
                }
                substage=10;
                destination[0]=target[0];
                destination[1]=target[1];
                node=2;
              //}
            
            }
            
            /*else if(substage==451){
              if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) and (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
                stop();
                current_node[0]=target[0];
                current_node[1]=target[1];
                substage=0;
              }else{
                PID();
              }
            }*/
            else if (substage==80){
              if(grid[target[0]][target[1]]>1){
                substage=100;
              }else{
                stop();
                if(pass){
                  substage=81;
                  if (orientation==7){
                    if (target[1]==0){
                      stage_motor->setPosition(1.5708);
                    }else{
                      stage_motor->setPosition(-1.5708);
                    }
                  }else if (orientation==5){
                    if (target[1]==0){
                      stage_motor->setPosition(-1.5708);
                    }else{
                      stage_motor->setPosition(1.5708);
                    }
                  }else if (orientation==4){
                    if (target[0]==12){
                      stage_motor->setPosition(1.5708);
                    }else{
                      stage_motor->setPosition(-1.5708);
                    }
                  }else if (orientation==6){
                    if (target[0]==12){
                      stage_motor->setPosition(-1.5708);
                    }else{
                      stage_motor->setPosition(1.5708);
                    }
                  }
                  linear_motor->setPosition(0.0);
                }else{
                //map as an obstacle and go back
                  grid[target[0]][target[1]]=-5;
                  substage=85;
                }
              }
            
            }
            else if(substage==81){
              if (delay==60){
                delay=0;
                substage=82;
              }else{
               delay+=1;
              }
            }else if(substage==82){
              front_flap_1->setPosition(0.0);
              left_flap_1->setPosition(0.0);
              right_flap_1->setPosition(0.0);
              rear_flap_1->setPosition(0.0);
              
              front_servo_1->setPosition(0);//1.57
              left_servo_1->setPosition(0);
              right_servo_1->setPosition(0);
              
              linear_motor->setPosition(0.04);
                            
              substage=83;
              grid[target[0]][target[1]]=5;
            }else if(substage==83){
              if (delay==40){
                delay=0;
                substage=84;
              }else{
               delay+=1;
              }
            }else if(substage==84){
              stage_motor->setPosition(0.0);
              substage=80;//80
              pass=false;
            }
            
/////////////////////////////////////////////////////////////
            else if (substage==85){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=86; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
            }else if (substage==86){
              if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
                wheels[0]->setVelocity(0);
                wheels[1]->setVelocity(0);
                substage=87;
              }else{
                PID();
              }
            }else if (substage==87){
              if (J[0]==1 and J[1]==1){
                stop();
                substage=88;
              }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
              }
            }else if (substage==88){
              substage=10;
              stop();
              node=2;
              //target[0]=destination[0];
              //target[1]=destination[1];
              for (int i=0;i<13;i++){
                  cout<<i<<"  -  ";
                  for (int j=0;j<17;j++){
                    cout<<grid[i][j]<<",";
                  }
                  cout<<endl;
              }
              cout<<"  middle node - ("<<middle_node[0]<<","<<middle_node[1]<<")";
              cout<<"  target node - ("<<target[0]<<","<<target[1]<<")";
              cout<<endl;
            }
            
/////////////////////////////////////////////////////////////
            else if (substage==100){
              //cout<<"inverse"<<endl;
              if((S[0]==0 and S[1]==0 and S[2]==0 and S[3]==0) or (S[4]==0 and S[5]==0 and S[6]==0 and S[7]==0)){
                stop();
                substage=101;
              }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
              }
            }else if (substage==101){
              if(J[0]==0 or J[1]==0){ //and was previously used
                stop();
                substage=102;
                node+=2;
                current_node[0]=target[0];
                current_node[1]=target[1];
                target[0]=route[node][0]; target[1]=route[node][1];
              
              }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
              }
            }else if (substage==102){
              //stop();
              cout<<"inverse mid"<<endl;
              
              if(orientation==4){
                if (current_node[0]==target[0]){
                //check front cam
                //cout<<"go forward"<<endl;
                  if(current_node[1]<target[1]){
                    if (J[0]==1 and J[1]==1){
                      wheels[0]->setVelocity(3);
                      wheels[1]->setVelocity(3);
                    }else{
                      stop();
                      substage=103;
                    }
                  }else if (current_node[1]>target[1]){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=103; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
                  }
                }else if (current_node[0]<target[0]){
                //check left cam
                //cout<<"turn left"<<endl;
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=103; 
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
                  
                }else if (current_node[0]>target[0]){
                //check right cam
                //cout<<"turn right"<<endl;
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=103; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                  right_turn();
                  }
                  
                }
                
              }else if (orientation==5){
                if (current_node[1]==target[1]){
                //check front cam
                  if(current_node[0]>target[0]){
                    if (J[0]==1 and J[1]==1){
                      wheels[0]->setVelocity(3);
                      wheels[1]->setVelocity(3);
                    }else{
                      stop();
                      substage=103;
                    }
                  }else if (current_node[0]<target[0]){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=103; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
                  }
                  
                }else if (current_node[1]<target[1]){
                //check left cam
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=103; 
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
                  
                }else if (current_node[1]>target[1]){
                //check right cam
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=103; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                  right_turn();
                  }
                }
                
              }else if (orientation==6){
                if (current_node[0]==target[0]){
                //check front cam
                  
                  if(current_node[1]>target[1]){
                    if (J[0]==1 and J[1]==1){
                      wheels[0]->setVelocity(3);
                      wheels[1]->setVelocity(3);
                    }else{
                      stop();
                      substage=103;
                    }
                  }else if (current_node[1]<target[1]){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=103; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
                  }
                  
                }else if (current_node[0]<target[0]){
                //check right cam
                //cout<<"turn right"<<endl;
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=103; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                  right_turn();
                  }
                  
                }else if (current_node[0]>target[0]){
                //check left cam
                //cout<<"turn left"<<endl;
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=103; 
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
                  
                }
              }else if (orientation==7){
                if (current_node[1]==target[1]){
                //check front cam
                  
                  if(current_node[0]<target[0]){
                    if (J[0]==1 and J[1]==1){
                      wheels[0]->setVelocity(3);
                      wheels[1]->setVelocity(3);
                    }else{
                      stop();
                      substage=103;
                    }
                  }else if (current_node[0]>target[0]){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=103; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
                  }
                  
                }else if (current_node[1]<target[1]){
                //check right cam
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=103; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                    right_turn();
                  }
                  
                }else if (current_node[1]>target[1]){
                //check left cam
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=103; 
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
                  
                }
              }
            }else if (substage==103){
              if (S[0]==1 and S[7]==1){
                for (int i=0;i<8;i++){
                  S[i]=(S[i]+1)%2;
                }
                PID();
              }else{
                stop();
                substage=104;
              }
            }else if (substage==104){
              if(J[0]==1 and J[1]==1){
                PID();
              }else{
                stop();
                substage=13;
              }
            }else if (substage==105){
              stop();
            }
            
            
            
//////////////////////////////////////////////////////////        
         }
      }else if (stage==501){
        for(int i=0;i<13;i++){
          cout<<i<<" --- ";
          for(int j=0;j<17;j++){
            cout<<grid[i][j]<<" , ";
          }
          cout<<endl;
        }
        stage+=1; 
      }
      else if(stage==600){
        if(substage==0){
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=1;
                     
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
        }else if(substage==1){
          if(S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
            stop();
            substage=2;
          }else{
            PID();
          }
        }else if(substage==2){
          if(J[0]==1 and J[1]==1){
            stop();
            substage=3;
          }else{
            wheels[0]->setVelocity(3);
            wheels[1]->setVelocity(3);
          }
        }else if(substage==3){
          linear_motor->setPosition(0.0);
          substage=4;
        }else if(substage==4){
          stop();
            if (delay==50){
              delay=0;
              substage=5;
            }else{
             delay+=1;
            }
        }else if(substage==5){
              front_servo->setPosition(1.6580);//1.6580//1.57
              left_servo->setPosition(1.8);
              right_servo->setPosition(1.8);
              substage=6;
        }else if(substage==6){
            stop();
            if (delay==10){
              delay=0;
              substage=7;
            }else{
             delay+=1;
            }
        }else if(substage==7){
              front_flap->setPosition(0.02);
              left_flap->setPosition(0.07);
              right_flap->setPosition(0.07);
              rear_flap->setPosition(0.02);
              substage=8;
        }else if(substage==8){
          stop();
            if (delay==70){
              delay=0;
              substage=9;
            }else{
             delay+=1;
            }
        }else if(substage==9){
          linear_motor->setPosition(0.06);
          substage=10;
        }else if(substage==10){
          if(S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
            wheels[0]->setVelocity(-3);
            wheels[1]->setVelocity(-3);
          }else{
            stop();
            substage=11;
          }
        }else if(substage==11){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=12; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
        }else if(substage==12){
          if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
            stop();
            substage=13;
          }else{
            PID();
          }
        }else if(substage==13){
          if (J[0]==1 or J[1]==1){
            stop();
            substage=14;
          }else{
            wheels[0]->setVelocity(3);
            wheels[1]->setVelocity(3);
          }
        }else if(substage==14){
          //turn right
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=15; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                  right_turn();
                  }
        }else if(substage==15){
          if(S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
            stop();
            substage=16;
          }else{
            PID();
          }
        }else if(substage==16){
          if(J[0]==1 or J[1]==1){
            stop();
            substage=17;
            linear_motor->setPosition(0.05);
          }else{
            wheels[0]->setVelocity(3);
            wheels[1]->setVelocity(3);
          }
        }else if(substage==17){
          //grid[target[0]][target[1]]=-8;
                front_flap->setPosition(0.0);
                left_flap->setPosition(0.0);
                right_flap->setPosition(0.0);
                rear_flap->setPosition(0.0);
                front_servo->setPosition(0.0);//1.57
                left_servo->setPosition(0.0);
                right_servo->setPosition(0.0);
                substage=18;
        }else if(substage==18){
          stop();
            if (delay==50){
              delay=0;
              substage=29;
            }else{
             delay+=1;
            }
        }else if(substage==29){
          if(S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
            wheels[0]->setVelocity(-3);
            wheels[1]->setVelocity(-3);
          }else{
            stop();
            substage=30;
          }
        }else if(substage==30){
            wheels[0]->setVelocity(-3);
            wheels[1]->setVelocity(-3);
            if (delay==30){
              delay=0;
              substage=31;
              stop();
            }else{
             delay+=1;
            }
        
        }else if(substage==31){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=32; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
        }else if(substage==32){
          if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
            stop();
            substage=33;
          }else{
            PID();
          }
        }else if(substage==33){
          if(J[0]==1 or J[1]==1){
            stop();
            substage=34;
          }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
          }
        }else if(substage==34){
          //left turn
                  if (turn==0){
                    turn=R_Pos+3.9270;}
                  if (turn==-5){
                    substage=35;
                     
                    turn=0;
                    orientation=((orientation-1)%4)+4;
                  }else {
                  left_turn();
                  }
        }else if(substage==35){
          if(S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
            stop();
            substage=36;
            stage_motor->setPosition(1.5708);
          }else{
            PID();
          }
        }else if(substage==36){
          if(J[0]==1 and J[1]==1){
            stop();
            substage=37;
            linear_motor->setPosition(0.0);
          }else{
            wheels[0]->setVelocity(3);
            wheels[1]->setVelocity(3);
          }
        }else if(substage==37){
            stop();
            if (delay==50){
              delay=0;
              substage=38;
            }else{
             delay+=1;
            }
        }else if(substage==38){
              front_servo->setPosition(1.6580);//1.6580//1.57
              left_servo->setPosition(1.8);
              right_servo->setPosition(1.8);
              substage=39;
        }else if(substage==39){
            stop();
            if (delay==10){
              delay=0;
              substage=40;
            }else{
             delay+=1;
            }
        }else if(substage==40){
              front_flap->setPosition(0.02);
              left_flap->setPosition(0.08);//0.07
              right_flap->setPosition(0.08);
              rear_flap->setPosition(0.02);
              substage=41;
        }else if(substage==41){
            stop();
            if (delay==50){
              delay=0;
              substage=42;
              linear_motor->setPosition(0.14);
              stage_motor->setPosition(0.0);
            }else{
             delay+=1;
            }
        }else if(substage==42){
          if(S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
                wheels[0]->setVelocity(-3);
                wheels[1]->setVelocity(-3);
          }else{
            stop();
            substage=43;
          }
        }else if(substage==43){
                    if (turn==0){
                      turn=R_Pos+7.8540;}//4.7124*2
                    if (turn==-5){
                      substage=44; 
                      turn=0;
                      orientation=((orientation-1)%4)+4;
                      orientation=((orientation-1)%4)+4;
                    }else {
                    left_turn();
                    }
        }else if(substage==44){
          if((S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1) or (S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1)){
            stop();
            substage=45;
          }else{
            PID();
          }
        }else if(substage==45){
          if(J[0]==1 or J[1]==1){
            stop();
            substage=46;
          }else{
                wheels[0]->setVelocity(3);
                wheels[1]->setVelocity(3);
          }
        }else if(substage==46){
          //left turn
                  if (turn==0){
                    turn=L_Pos+3.9270;}
                  if (turn==-5){
                    substage=47; 
                    turn=0;
                    orientation=((orientation+1)%4)+4;
                  }else {
                  right_turn();
                  }
        }else if(substage==47){
          if(S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
            stop();
            substage=48;
          }else{
            PID();
          }
        }else if(substage==48){
          if(J[0]==1 and J[1]==1){
            stop();
            substage=49;
          }else{
            wheels[0]->setVelocity(3);
            wheels[1]->setVelocity(3);
          }
        }else if(substage==49){
                front_flap->setPosition(0.0);
                left_flap->setPosition(0.0);
                right_flap->setPosition(0.0);
                rear_flap->setPosition(0.0);
                front_servo->setPosition(0.0);//1.57
                left_servo->setPosition(0.0);
                right_servo->setPosition(0.0);
                substage=50;
        }else if(substage==50){
            stop();
            if (delay==50){
              delay=0;
              substage=51;
            }else{
             delay+=1;
            }
        }else if(substage==51){
          if(S[0]==1 and S[1]==1 and S[2]==1 and S[3]==1 and S[4]==1 and S[5]==1 and S[6]==1 and S[7]==1){
            wheels[0]->setVelocity(-3);
            wheels[1]->setVelocity(-3);
          }else{
            stop();
            //linear_motor->setPosition()
          }
        }
        
          
        
      }//stage 600 ends here
      
      
      
          

    } // while loop ends here
}// main function ends here

//////////get and print sensor data for line sensing////////
void getLineSensors() {
    for (int i = 0; i < 8; i++) {
        S[i] = (F_S[i]->getValue() < TH);
        //cout << S[i] << ",";
    }
    //cout << endl;
    J[0]=(junc_s[0]->getValue() < TH);
    J[1]=(junc_s[1]->getValue() < TH);
    WB[0]=(wb_s[0]->getValue() < TH);
    WB[1]=(wb_s[1]->getValue() < TH);
}


////////////moving manually///////////////////
void rotateWheels(int leftSpeed, int rightSpeed) {

    wheels[0]->setVelocity(leftSpeed);
    wheels[1]->setVelocity(rightSpeed);

}
short gainCalc(short errorX) {
    short gains[8] = { 0,1,3,6,15,30,55 };
    return gains[errorX];

}

//////////////////PID//////////////////
void transferCoff() {
    a0 = (1 + N * Ts);
    a1 = -(2 + N * Ts);
    a2 = 1;
    b0 = Kp * (1 + N * Ts) + Ki * Ts * (1 + N * Ts) + Kd * N;
    b1 = -(Kp * (2 + N * Ts) + Ki * Ts + 2 * Kd * N);
    b2 = Kp + Kd * N;

}

void pidUpdate() {
    
    c[2] = c[1];
    c[1] = c[0];
    e[2] = e[1];
    e[1] = e[0];
}


void PID() {
   // cout << a0 << "," << a1 << "," << a2 << "," << b0 << "," << b1 << "," << b2 << endl;
    short eSum = (-4 * S[0]) + (-3 * S[1]) + (-2 * S[2]) + (-1 * S[3]) + (1 * S[4]) + (2 * S[5]) + (3 * S[6]) + (4 * S[7]);
    short linerGains[10] = { 0,0,1,2,7,4,3,6,0,5 };
    short errorX = linerGains[abs(eSum)];
  
    e[0] = gainCalc(errorX);
    e[0] = e[0] * (1 - 2 * signbit(double(eSum)));
    c[0] = (-a1 * c[1] - a2 * c[2] + b0 * e[0] + b1 * e[1] + b2 * e[2]) / a0;
   // cout << a0<<","<< a1 << ","<< a2 << ","<< b0 << ","<<b1 << ","<< b2 << endl;
   // cout << e[0] <<","<<errorX<<endl;
    leftSpeed = midSpeed + c[0];
    rightSpeed = midSpeed - c[0];

    if (leftSpeed > MAX_SPEED) { leftSpeed = MAX_SPEED; }
    else if (leftSpeed < MIN_SPEED) { leftSpeed = MIN_SPEED; }
    if (rightSpeed > MAX_SPEED) { rightSpeed = MAX_SPEED; }
    else if (rightSpeed < MIN_SPEED) { rightSpeed = MIN_SPEED; }
    pidUpdate();
    rotateWheels(leftSpeed, rightSpeed);

}


//________________________________achintha 
void left_turn(){
  cout<<"left turning = "<<turn<<endl;
  if (R_Pos<turn){
    wheels[0]->setVelocity(-5);
    wheels[1]->setVelocity(5);
  }else{
    turn=-5;
    stop();
  }
  //stop();
}

void right_turn(){
  cout<<"right turning"<<endl;
  if (L_Pos<turn){
    wheels[0]->setVelocity(5);
    wheels[1]->setVelocity(-5);
  }else{
    turn=-5;
    stop();
  }
  
}

void go_forward(){
  cout<<"going forward"<<endl;
  if (L_Pos<turn){
    wheels[0]->setVelocity(3);
    wheels[1]->setVelocity(3);
  }else{
    turn=-5;
    stop();
  }
  
}

void stop() {

    wheels[0]->setVelocity(0);
    wheels[1]->setVelocity(0);

}

//*********************************************************
bool isValid(int row, int col){
    return (row >= 0) && (row < ROW) && (col >= 0)&& (col < COL);
}

bool isUnBlocked(int grid[][COL], int row, int col){
    if (grid[row][col] > 0)
        return (true);
    else
        return (false);
}

bool isDestination(int row, int col, Pair dest){
    if (row == dest.first && col == dest.second)
        return (true);
    else
        return (false);
}

double calculateHValue(int row, int col, Pair dest){
    //return ((double)sqrt((row-dest.first)*(row-dest.first)+(col-dest.second)*(col-dest.second)));
    return ((double)abs(abs(row-dest.first)+abs(col-dest.second)));
}

void tracePath(cell cellDetails[][COL], Pair dest){
    //printf("\nThe Path is ");
    int row = dest.first;
    int col = dest.second;
 
    stack<Pair> Path;
 
    while (!(cellDetails[row][col].parent_i == row
             && cellDetails[row][col].parent_j == col)) {
        Path.push(make_pair(row, col));
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
        row = temp_row;
        col = temp_col;
    }
 
    Path.push(make_pair(row, col));
    int count=0;
    while (!Path.empty()) {
        pair<int, int> p = Path.top();
        Path.pop();
        //cout<<"("<<p.first<<","<<p.second<<")"<<endl;
        route[count][0]=p.first;
        route[count][1]=p.second;
        count+=1;
    }
 
    return;
}

void aStarSearch(int grid[][COL], Pair src, Pair dest){
    for (int i=0; i<60; i++){
      for (int j=0; j<2; j++){
        route[i][j]=-1;
      }
    }
    
    if (isValid(src.first, src.second) == false) {
        printf("Source is invalid\n");
        return;
    }
 
    
    if (isValid(dest.first, dest.second) == false) {
        printf("Destination is invalid\n");
        return;
    }
 
    
    if (isUnBlocked(grid, src.first, src.second) == false
        || isUnBlocked(grid, dest.first, dest.second)
               == false) {
        printf("Source or the destination is blocked\n");
        return;
    }
 
    
    if (isDestination(src.first, src.second, dest)
        == true) {
        printf("We are already at the destination\n");
        return;
    }
 
    
    bool closedList[ROW][COL];
    memset(closedList, false, sizeof(closedList));
 
    
    cell cellDetails[ROW][COL];
 
    int i, j;
 
    for (i = 0; i < ROW; i++) {
        for (j = 0; j < COL; j++) {
            cellDetails[i][j].f = FLT_MAX;
            cellDetails[i][j].g = FLT_MAX;
            cellDetails[i][j].h = FLT_MAX;
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
        }
    }
 
    
    i = src.first, j = src.second;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;
 
   
    set<pPair> openList;
 
    openList.insert(make_pair(0.0, make_pair(i, j)));
 
    
    bool foundDest = false;
 
    while (!openList.empty()) {
        pPair p = *openList.begin();
 
        
        openList.erase(openList.begin());
 
        
        i = p.second.first;
        j = p.second.second;
        closedList[i][j] = true;
 
        
        double gNew, hNew, fNew;
 
        //----------- 1st Successor (North) ------------
 
        
        if (isValid(i - 1, j) == true) {
            
            if (isDestination(i - 1, j, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i - 1][j].parent_i = i;
                cellDetails[i - 1][j].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest);
                foundDest = true;
                return;
            }
            
            else if (closedList[i - 1][j] == false
                     && isUnBlocked(grid, i - 1, j)
                            == true) {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i - 1, j, dest);
                fNew = gNew + hNew;
 
                
                if (cellDetails[i - 1][j].f == FLT_MAX
                    || cellDetails[i - 1][j].f > fNew) {
                    openList.insert(make_pair(
                        fNew, make_pair(i - 1, j)));
 
                    
                    cellDetails[i - 1][j].f = fNew;
                    cellDetails[i - 1][j].g = gNew;
                    cellDetails[i - 1][j].h = hNew;
                    cellDetails[i - 1][j].parent_i = i;
                    cellDetails[i - 1][j].parent_j = j;
                }
            }
        }
 
        //----------- 2nd Successor (South) ------------
 
        
        if (isValid(i + 1, j) == true) {
            
            if (isDestination(i + 1, j, dest) == true) {
                
                cellDetails[i + 1][j].parent_i = i;
                cellDetails[i + 1][j].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest);
                foundDest = true;
                return;
            }
            
            else if (closedList[i + 1][j] == false
                     && isUnBlocked(grid, i + 1, j)
                            == true) {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i + 1, j, dest);
                fNew = gNew + hNew;
 
                
                if (cellDetails[i + 1][j].f == FLT_MAX
                    || cellDetails[i + 1][j].f > fNew) {
                    openList.insert(make_pair(
                        fNew, make_pair(i + 1, j)));
                    
                    cellDetails[i + 1][j].f = fNew;
                    cellDetails[i + 1][j].g = gNew;
                    cellDetails[i + 1][j].h = hNew;
                    cellDetails[i + 1][j].parent_i = i;
                    cellDetails[i + 1][j].parent_j = j;
                }
            }
        }
 
        //----------- 3rd Successor (East) ------------
 
        
        if (isValid(i, j + 1) == true) {
            
            if (isDestination(i, j + 1, dest) == true) {
                // Set the Parent of the destination cell
                cellDetails[i][j + 1].parent_i = i;
                cellDetails[i][j + 1].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest);
                foundDest = true;
                return;
            }
 
            
            else if (closedList[i][j + 1] == false
                     && isUnBlocked(grid, i, j + 1)
                            == true) {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i, j + 1, dest);
                fNew = gNew + hNew;
 
                
                if (cellDetails[i][j + 1].f == FLT_MAX
                    || cellDetails[i][j + 1].f > fNew) {
                    openList.insert(make_pair(
                        fNew, make_pair(i, j + 1)));
 
                    cellDetails[i][j + 1].f = fNew;
                    cellDetails[i][j + 1].g = gNew;
                    cellDetails[i][j + 1].h = hNew;
                    cellDetails[i][j + 1].parent_i = i;
                    cellDetails[i][j + 1].parent_j = j;
                }
            }
        }
 
        //----------- 4th Successor (West) ------------
 
        
        if (isValid(i, j - 1) == true) {
            
            if (isDestination(i, j - 1, dest) == true) {
                
                cellDetails[i][j - 1].parent_i = i;
                cellDetails[i][j - 1].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest);
                foundDest = true;
                return;
            }
 
            
            else if (closedList[i][j - 1] == false
                     && isUnBlocked(grid, i, j - 1)
                            == true) {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i, j - 1, dest);
                fNew = gNew + hNew;
 
                
                if (cellDetails[i][j - 1].f == FLT_MAX
                    || cellDetails[i][j - 1].f > fNew) {
                    openList.insert(make_pair(
                        fNew, make_pair(i, j - 1)));
 
                    
                    cellDetails[i][j - 1].f = fNew;
                    cellDetails[i][j - 1].g = gNew;
                    cellDetails[i][j - 1].h = hNew;
                    cellDetails[i][j - 1].parent_i = i;
                    cellDetails[i][j - 1].parent_j = j;
                }
            }
        }
 
        
        
    }
 
    // When the destination cell is not found and the open
    // list is empty, then we conclude that we failed to
    // reach the destination cell. This may happen when the
    // there is no way to destination cell (due to
    // blockages)
    if (foundDest == false)
        printf("Failed to find the Destination Cell\n");
 
    return;
}//A star ends here

void color_test(){
  //cout<<"c_tset"<<endl;
  for (int i=0; i<3; i++){
    
    /*R*/rgb[i][0] = camera[i]->imageGetRed(camera[i]->getImage(), (camera[i]->getWidth())/2, 32, 32);
    /*G*/rgb[i][1] = camera[i]->imageGetGreen(camera[i]->getImage(), (camera[i]->getWidth())/2, 32, 32);
    /*B*/rgb[i][2] = camera[i]->imageGetBlue(camera[i]->getImage(), (camera[i]->getWidth())/2, 32, 32);
  
    cout<<i<<" - "<<rgb[i][0]<<","<<rgb[i][1]<<","<<rgb[i][2]<<"   -   ";
    
    if (rgb[i][0]>100 and rgb[i][1]<100 and rgb[i][2]<100){
      barrier[i]=true;
      cout<<"red"<<endl;
    }else{
       barrier[i]=false;
       cout<<"white"<<endl;
    }
  }
}

void get_box_color(){
  
  int red = box_cam->imageGetRed(box_cam->getImage(), (box_cam->getWidth())/2, 32, 32);
  int green = box_cam->imageGetGreen(box_cam->getImage(), (box_cam->getWidth())/2, 32, 32);
  int blue = box_cam->imageGetBlue(box_cam->getImage(), (box_cam->getWidth())/2, 32, 32);

  cout<<"red="<<red<<"  green="<<green<<"  blue="<<blue<<endl;
  
  if(red>150 and green<150 and blue<150){box_color=1;}
  else if(red<150 and green>150 and blue<150){box_color=2;}
  else if(red<150 and green<150 and blue>150){box_color=3;}
  else{box_color=4;}
}