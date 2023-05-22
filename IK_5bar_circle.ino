#include <Servo.h>
#include<math.h>

void ik_2R(float xbyb[], float l1l2[], float xy[], int sol, float op_th1th2[]);
void ik_5bar(float xbLybLxbRybR[], float l1l2l3l4[], float xy[], int sols[], float op_th1th2[]);
// Add your own trajectory function here instead of circle
void circle(float centre_xy[], float rad, float ang_deg, float op_xy[]);

Servo myservoL, myservoR;
float linklengths[4] = {0.2,0.2,0.2,0.2};
float baseLbaseR[4] = {-0.03135,0,0.03125,0};
int solbranch[2] = {1,-1};
float XY[2] = {0,250};
float thLthR[2];

float circ_cen[2] = {0,0.28};
float circ_rad = 0.04;

void setup() {
  Serial.begin(115200);
  // PWM pin no. 9 for motor R
  myservoR.attach(9);
  // PWM pin no. 10 for motor L
  myservoL.attach(10);
}

void loop() {
   
   // Trajectory
   float t = 0.3*millis();
   circle(circ_cen, circ_rad, t, XY);
   
   if (Serial.available() > 0) 
   {
      Serial.print("X coord: ");
      Serial.println(XY[0]);
      Serial.print("Y coord: ");
      Serial.println(XY[1]);
      
      ik_5bar(baseLbaseR, linklengths, XY, solbranch, thLthR);

      // Write to servo
      thLthR[0]-=45; // Left motor offset
      thLthR[1]+=53; // Right motor offset
      Serial.println(thLthR[0]);
      Serial.println(thLthR[1]);
      
      int pulseWidthL = map(thLthR[0], 0, 180, 500, 2500);
      myservoL.writeMicroseconds(pulseWidthL);
      int pulseWidthR = map(thLthR[1], 0, 180, 500, 2500);
      myservoR.writeMicroseconds(pulseWidthR);
      
    }
}

  

void ik_2R(float xbyb[], float l1l2[], float xy[], int sol, float op_th1th2[])
{
  // Function: Inverse kinematics of 2R
  // op_th1th2[] in degrees
  // Vyankatesh Ashtekar, IIT Kanpur, February 2023
  // All inputs in same units
  // 1 means elbow up solution (int sol)
  // -1 means elbow down solution (int sol)

  float l1 = l1l2[0];
  float l2 = l1l2[1];

  float x = xy[0] - xbyb[0];
  float y = xy[1] - xbyb[1];

  //    Serial.print("X is ");
  //    Serial.println(x);
  //    Serial.print("Y is ");
  //    Serial.println(y);

  //***********Solution process********************

  float d = sqrt(x * x + y * y);
  //    Serial.print("d is ");
  //    Serial.println(d);
  // Inverse kinematics computations-->
  float psi = atan2(y, x);
  //    Serial.print("ArcTan2(y,x) is ");
  //    Serial.println(psi);

  float costh2 = (d * d - l1 * l1 - l2 * l2) / (2 * l1 * l2);

  if (abs(costh2) > 1)
  {
    Serial.println("The point is outside workspace (assuming no limits on joint angles)");
  }
  else
  {
    float Pi = 3.14159;
    float theta2 = acos(costh2);
    //      Serial.print("theta2 solutions (in degrees) are +- ");
    //      Serial.println(180 * theta2 / Pi);

    if (sol == 1)
    {
      //        Serial.println("Presenting elbow up solution");
      theta2 = -theta2;
    }
    //      else if (sol == -1)
    //      {
    //        Serial.println("Presenting elbow down solution");
    //      }

    float sinth2 = sin(theta2);
    float a11 = l1 + l2 * costh2;
    float a12 = -1.0 * l2 * sinth2;
    float a21 = -1.0 * a12;
    float a22 = a11;
    float det = a22 * a11 - a12 * a21;
    float detc1 = -(y * a12 - x * a22);
    float dets1 = (y * a11 - a21 * x);
    float c1 = detc1 / det;
    float s1 = dets1 / det;

    float theta1 = atan2(s1, c1);

    op_th1th2[0] = (180 * theta1 / Pi);
    op_th1th2[1] = (180 * theta2 / Pi);

    //      Serial.print("theta1 (in degrees) is ");
    //      Serial.println(op_th1th2[0]);
    //      Serial.print("theta2 (in degrees) is  ");
    //      Serial.println(op_th1th2[1]);

    //      if (abs(op_th1th2[0]) > 90 || abs(op_th1th2[1]) > 90)
    //      {
    //        Serial.println("Point inside geometrical workspace, but outside joint angle limits");
    //      }
    //      else
    //      {
    //        // mapping to actual servos-- offset and direction flip
    //        op_th1th2[0] = 180 - (op_th1th2[0] + 90);
    //        op_th1th2[1] = 180 - (op_th1th2[1] + 90);
    //      }
  }

}

void ik_5bar(float xbLybLxbRybR[], float l1l2l3l4[], float xy[], int sols[], float op_th1th2[])
{
  // op_th1th2 in degrees
  
  float left_th1th2[2], right_th1th2[2];
  ik_2R(xbLybLxbRybR,l1l2l3l4,xy,sols[0],left_th1th2);
  ik_2R(xbLybLxbRybR+2,l1l2l3l4+2,xy,sols[1],right_th1th2);

  op_th1th2[0] = left_th1th2[0];
  op_th1th2[1] = right_th1th2[0];
}

void circle(float centre_xy[], float rad, float ang_deg, float op_xy[])
{
  // Circular trajectory
  // Gives coordinates xy at 'ang (degrees)'

  float ang_rad;
  float Pi = 3.14159;
  ang_rad = ang_deg * Pi / 180;
  op_xy[0] = centre_xy[0] + rad*cos(ang_rad);
  op_xy[1] = centre_xy[1] + rad*sin(ang_rad);
}
