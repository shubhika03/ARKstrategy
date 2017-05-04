#include "strategy.h"
#include <math.h>
#include <vector>

#define PI 3.14

using namespace std;

typedef struct {
  int x;
  int y;
}point;

float dist_whitel(point bot,float orient){

  if(orient>=0 && orient<PI/2){
    if(20-bot.x<20-bot.y)
      return 20-bot.x;
    else
      return 20-bot.y;
  }

  if(orient>=PI/2 && orient>PI){
    if(bot.x<20-bot.y)
      return bot.x;
    else
      return 20-bot.y;
  }

  if(orient<0 && orient>-PI/2){
      return bot.x;
  }

  if(orient>-PI && orient<-PI/2){
      return 20-bot.x;
  }

}

float angle(float ang){
  
  if(ang>=0 && ang<=PI)
    return ang;
  else if(ang>PI)
    return ang-2*PI;
  else if(ang<-PI)
    return ang+2*PI;

}

void action(point t_bot, point bot, float orient){
  float theta1,theta2;  

  theta1=atan((t_bot.y-bot.y)/(t_bot.x-bot.x))-PI/4;
  theta2=theta + PI/2;

  if(orient>=theta1 && orient<=theta2){
    //do nothing
  }
  else if(orient>theta2 && orient<angle(theta2+PI/4)){
    //one 45 degree rotation anticlockwise
  }
  else if(orient>=angle(theta1+PI) && orient<=angle(theta2+PI)){
    //turn 180 degree
  }
  else if(orient<theta1 && orient>=angle(theta1-PI/4)){
    //one 180 degree turn and then one 45 degree anti clockwise rotation
  }
  else if(orient>angle(theta2+PI/4) && orient<=angle(theta2+PI/2)){
    //two 45 degree anticlockwise rotations
  }

  else if(orient<angle(theta1-PI/4) && orient>=angle(theta1-PI/2)){
    //one 180 degree turn and two 45 degree anticlockwise turn
  }

}


void plan(int no,vector<int> a){
  //no for the target bot and the vector for the bots inside the circle
  
  int size=a.size();  
  point target_bot,bot;                      // suscribe to the publisher to get the coordinates of the target bot 
  float dis,bot_no=a[0],dist_line,min,orient;     
  point bot;                                 //for the bots inside the circle

  for(int i=0;i<size;i++){
    //suscribe and get the coordinates of the bot with the no a[i]
    //get the orintation of the bot from x and y axis
    dis=dist_whitel(bot,orient);
    if(i==0)
      min=dis;
    else if(dis<min){
      min=dis;
      bot_no=a[i];
    }

  }
  //suscribe and get the position of the selected bot no
  action(target_bot,bot,orient);
  

} 










