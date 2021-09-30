
//--------------------------------------------------------------------------
// Ann Majewicz Fey, University of Texas at Austin
// Last Modified: 08.27.21
// Code to test basic functionaility of the Longhorn Hapkit (w/ encoder)
//--------------------------------------------------------------------------

// INCLUDES
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <TimerOne.h>  // This library manages the timing of the haptic loop
#include <Encoder.h>   // This library manages the encoder read.

//enable select  functions
//#define ItsWallTime
//#define ItsDampingTime
//#define ItsFrictionTime
//#define ItsBumpTime
//#define ItsTextureTime
#define ItsSurfaceTime

// Pin Declarations
const int PWMoutp = 4;
const int PWMoutn = 5;
const int PWMspeed = 6;

const int encoder0PinA = 2;
const int encoder0PinB = 3;

Encoder encoder(encoder0PinA,encoder0PinB);

double encoderResolution = 48;
double pos = 0;
double lastPos = 0;
double lastVel = 0;
double vh = 0;
double lastXh = 0;
double lastLastVh = 0;
double lastVh = 0;


// Kinematics variables
double xh = 0;           // position of the handle [m]
double theta_s = 0;      // Angle of the sector pulley in deg
double xh_prev;          // Distance of the handle at previous time step
double xh_prev2;
double dxh;              // Velocity of the handle
double dxh_prev;
double dxh_prev2;
double dxh_filt;         // Filtered velocity of the handle
double dxh_filt_prev;
double dxh_filt_prev2;

// *******************************************
// UNCOMMENT THESE AND INCLUDE CORRECT NUMBERS
// *******************************************
double rh = 0.069;   //[m]
double rp = 0.005;  //[m]
double rs = 0.074;  //[m]
// *******************************************

// Force output variables
double force;           // force at the handle
double Tp;              // torque of the motor pulley
unsigned int output = 0;    // output command to the motor

// Timing Variables: Initalize Timer and Set Haptic Loop
boolean hapticLoopFlagOut = false;
boolean timeoutOccured = false;

unsigned long t = 0; // time since program started
unsigned long t_imp = 0; //time wall impact occurred
boolean inWall = false;
boolean impact = false;

double d = 200;  //[ms]
double f = 100;   //[rad/s]
double A = 0.5; //[N/(m/s)]
double force_vib = 0;
//--------------------------------------------------------------------------
// Initialize
//--------------------------------------------------------------------------
void setup()
{
  // Set Up Serial
  Serial.begin(115200);

 // Output Pins
 pinMode(PWMoutp, OUTPUT);
 pinMode(PWMoutn, OUTPUT);
 pinMode(PWMspeed, OUTPUT);

 // Haptic Loop Timer Initalization
   Timer1.initialize();
  long period = 1000; // [us]  10000 [us] - 100 Hz
  Timer1.attachInterrupt(hapticLoop,period);

  // Init Position and Velocity
  lastPos = encoder.read();
  lastVel = 0;

  // Initalize motor direction and set to 0 (no spin)
  digitalWrite(PWMoutp, HIGH);
  digitalWrite(PWMoutn, LOW);
  analogWrite(PWMspeed, 0);

}

//--------------------------------------------------------------------------
// Main Loop
//--------------------------------------------------------------------------

void loop()
{
    if(timeoutOccured)
  {
    Serial.println("timeout occured");
  }
}

// --------------------------
// Haptic Loop
// --------------------------
  void hapticLoop()
  {

      // See if flag is out (couldn't finish before another call)
      if(hapticLoopFlagOut)
      {
        timeoutOccured = true;
      }
      //*************************************************************
      //*** Section 1. Compute position and velocity using encoder (DO NOT CHANGE!!) ***
      //*************************************************************
      pos = encoder.read();
      double vel = (.80)*lastVel + (.20)*(pos - lastPos)/(.01);


        //*************************************************************
        //*** Section 2. Compute handle position in meters ************
        //*************************************************************

          // ADD YOUR CODE HERE

          // SOLUTION:
          // Define kinematic parameters you may need

          // Step 2.1: print updatedPos via serial monitor
          //*************************************************************

           //Serial.println(pos);

          // Step 2.2: Compute the angle of the sector pulley (ts) in degrees based on updatedPos
         //*************************************************************

            double ts = -pos/48*360*rp/rs; // NOTE - THESE NUMBERS MIGHT NOT BE CORRECT! USE KINEMATICS TO FIGRUE IT OUT!


         // Step 2.3: Compute the position of the handle based on ts
          //*************************************************************

            xh = rh*(ts*3.14159/180);       // Again, these numbers may not be correct. You need to determine these relationships.

          // Step 2.4: print xh via serial monitor
          //*************************************************************

           //Serial.println(xh,5);

          // Step 2.5: compute handle velocity
          //*************************************************************
             vh = -(.95*.95)*lastLastVh + 2*.95*lastVh + (1-.95)*(1-.95)*(xh-lastXh)/.0001;  // filtered velocity (2nd-order filter)
             lastXh = xh;
             lastLastVh = lastVh;
             lastVh = vh;

        //*************************************************************
        //*** Section 3. Assign a motor output force in Newtons *******
        //*************************************************************

            // Init force
            double force = 0;
            //double K = 300;  // spring stiffness

            //force = K*xh;

         // This is just a simple example of a haptic wall that only uses encoder position.
         // You will need to add the rest of the following cases. You will want to enable some way to select each case.
         // Options for this are #DEFINE statements, swtich case statements (i.e., like a key press in serial monitor), or
         // some other method.

          // Virtual Wall
        //*************************************************************
           #if defined(ItsWallTime)
           double K=300;
             if(xh > 0.005){
              force = K*(xh-0.005);
             }else{
              force = 0;
             }
           #endif

         // Linear Damping
        //*************************************************************
           #if defined(ItsDampingTime)
           double b = 1;
             force = b*vh;
           #endif

         // Nonlinear Friction
         // I want to try Brown and McPhee continuous non-linear friciton model
        //*************************************************************
           #if defined(ItsFrictionTime)

           double F_C=0;    //coulombic friction
           double F_S=.5;     //static friction
           double v_S=0.06;  //stribeck velocity
           double v_T=vh;    //tangential velocity
           double b=0;
           if (abs(v_T)<0.00001){
            //b=0;
            force=0;
           }else if(abs(v_T)<0.0001){
            //force=.3*vh/abs(vh);
           }else{
            //b=0.1;
            //force=b*vh;
            force=((F_C*tanh(4*abs(v_T)/v_S)+(F_S-F_C)*(abs(v_T)/v_S)/pow((.25*pow((abs(v_T)/v_S),2)+.75),2)))*v_T/abs(v_T);
           }
           #endif

         // A Hard Surface
        //*************************************************************
           #if defined(ItsSurfaceTime)
            //not sure about this one. force needs to be a function of time?
           double K=300;
           t = millis();    //[ms]

           if(xh<0){
            force = 0;
              inWall = false;
           }
           else{
            //force = 0;
            force = K*xh;
            if(!inWall){
              //impact = true;
              t_imp = t;
            }
            inWall = true;
           }

           //if(impact){
            force_vib = A*abs(vh)*exp(log(0.01)*(t-t_imp)/d)*sin(f*(t-t_imp)/1000);
            force = force+force_vib;
            //if(t-t_imp>d){
              //impact = false;
            //}
           //}

           //Serial.println(force_vib);
           #endif

         // Bump and Valley
        //*************************************************************
           #if defined(ItsBumpTime)
           double fq=1000;   // frequency adjuster
           double amp=.5; // amplitude adjuster
           force=amp*sin(fq*xh);
           #endif

          // Texture
        //*************************************************************
           #if defined(ItsTextureTime)
           //I just copied the bump code. Make some changes to get an interesting result
           double fq=100000;   // frequency adjuster
           double amp=.3; // amplitude adjuster
           force=amp*sin(fq*xh);
           #endif

           // CHALLENGE POINTS: Try simulating a paddle ball! Hint you need to keep track of the virtual balls dynamics and
           // compute interaction forces relative to the changing ball position.
        //*************************************************************



      //*************************************************************
      //*** Section 5. Force output (do not change) *****************
      //*************************************************************

        // Determine correct direction
        //*************************************************************
        double Tp = force*rh*rp/rs;
        if(Tp < 0)
        {
        digitalWrite(PWMoutp, HIGH);
        digitalWrite(PWMoutn, LOW);
        } else
        {
        digitalWrite(PWMoutp, LOW);
        digitalWrite(PWMoutn, HIGH);
        }

        // Convert Torque to Duty
        int duty=abs(Tp)*255/0.008;
        // Limit torque to motor and write
        //*************************************************************
        if(duty > 255)
        {
          duty = 255;
        }

        if(duty < 25){
          duty=0; //deadzone
        }
            //Serial.println(force); // Could print this to troublshoot but don't leave it due to bogging down speed
        // Write out the motor speed.
        //*************************************************************
        //setPwmFrequency(9, 8);
        analogWrite(PWMspeed, duty); //abs(force)

  // Update variables
  lastVel = vel;
  lastPos = pos;
}
