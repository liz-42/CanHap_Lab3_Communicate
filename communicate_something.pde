/**
 **********************************************************************************************************************
 * @file       sketch_3_Hello_Ball.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V1.0.0
 * @date       08-January-2021
 * @brief      Ball haptic example with programmed physics for a haptic ball enclosed in a box
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */
 
  /* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  


/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerMeter                      = 4000.0;
float             radsPerDegree                       = 0.01745;

/* pantagraph link parameters in meters */
float             l                                   = 0.07; // m
float             L                                   = 0.09; // m


/* end effector radius in meters */
float             rEE                                 = 0.006;
float             rEEContact                          = 0.006;


/* virtual ball parameters  */
float             rBall                               = 0.015;

float             mBall                               = 0.15;  // kg
float             kBall                               = 600;  // N/m
float             bBall                               = 3.7;
float             penBall                             = 0.0;  // m
float             bAir                                = 0.0;  // kg/s
PVector           fGravity                            = new PVector(0, 9.8*mBall);
float             dt                                  = 1/1000.0;

PVector           posBall                             = new PVector(0, 0.05);  
PVector           posBall2                            = new PVector(0, 0.075);
PVector           posBall3                            = new PVector(0, 0.1);
PVector           posBall4                            = new PVector(0.025, 0.05);
PVector           posBall5                            = new PVector(0.025, 0.075);
PVector           posBall6                            = new PVector(0.025, 0.1);
PVector           posBall7                            = new PVector(0.05, 0.05);
PVector           posBall8                            = new PVector(0.05, 0.075);
PVector           posBall9                            = new PVector(0.05, 0.1);

PVector           velBall                             = new PVector(0, 0); 

PVector[]         solidBallPos                        = {posBall, posBall2, posBall3, posBall4, posBall5, posBall6, posBall7, posBall8, posBall9};

// water
PVector           posWaterBall1                       = new PVector(0, 0.05);

float             rWaterBall                          = rBall;


PVector           fBall                               = new PVector(0 ,0);    
PVector           fContact                            = new PVector(0, 0);
PVector           fDamping                            = new PVector(0, 0);

PVector           posEEToBall;
float             posEEToBallMagnitude;

PVector           velEEToBall;
float             velEEToBallMagnitude;


/* virtual wall parameters */
PVector           fWall                               = new PVector(0, 0);
float             kWall                               = 800; // N/m
float             bWall                               = 2; // kg/s
PVector           penWall                             = new PVector(0, 0);

PVector           posWallLeft                         = new PVector(-0.01, 0.03);
PVector           posWallRight                        = new PVector(0.02, 0.03);
PVector           posWallBottom                       = new PVector(0.0, 0.1);


/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           posEELast                           = new PVector(0, 0);
PVector           velEE                               = new PVector(0, 0);

PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);

/* World boundaries reference */
final int         worldPixelWidth                     = 1000;
final int         worldPixelHeight                    = 650;


/* graphical elements */
PShape pGraph, joint, endEffector;
PShape ball, ball2, ball3, ball4, ball5, ball6, ball7, ball8, ball9, leftWall, bottomWall, rightWall;
PShape waterBall, waterBall2;


// for gas
int rotations = 0;

String curState = "solid";
/* end elements definition *********************************************************************************************/ 


/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 650);
  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */ 
  haplyBoard          = new Board(this, "COM4", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  widgetOne.device_set_parameters();
    
  
  /* visual elements setup */
  background(0);
  deviceOrigin.add(worldPixelWidth/2, 0);
  
  /* create pantagraph graphics */
  create_pantagraph();
  
  
  /* create balls for solid */
  ball = create_ball(rBall);
  ball.setStroke(color(0));
  
  ball2 = create_ball(rBall);
  ball2.setStroke(color(0));
  
  ball3 = create_ball(rBall);
  ball3.setStroke(color(0));
  
  ball4 = create_ball(rBall);
  ball4.setStroke(color(0));
  
  ball5 = create_ball(rBall);
  ball5.setStroke(color(0));
  
  ball6 = create_ball(rBall);
  ball6.setStroke(color(0));
  
  ball7 = create_ball(rBall);
  ball7.setStroke(color(0));
  
  ball8 = create_ball(rBall);
  ball8.setStroke(color(0));
  
  ball9 = create_ball(rBall);
  ball9.setStroke(color(0));
  
  
  // water balls
  waterBall = create_ball(rWaterBall);
  waterBall.setStroke(color(0));
  
  
  /* create left-side wall */
  leftWall = create_wall(posWallLeft.x, posWallLeft.y, posWallLeft.x, posWallLeft.y+0.07);
  leftWall.setStroke(color(0));
  
  /* create right-sided wall */
  rightWall = create_wall(posWallRight.x, posWallRight.y, posWallRight.x, posWallRight.y+0.07);
  rightWall.setStroke(color(0));
  
  /* create bottom wall */
  bottomWall = create_wall(posWallBottom.x-0.07, posWallBottom.y, posWallBottom.x+0.07, posWallBottom.y);
  bottomWall.setStroke(color(0));

  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/




/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255);  
    update_animation(angles.x*radsPerDegree, angles.y*radsPerDegree, posEE.x, posEE.y);
  }
}
/* end draw section ****************************************************************************************************/




/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(device_to_graphics(posEE)); 
      
      velEE.set((posEE.copy().sub(posEELast)).div(dt));
      posEELast = posEE;
     
      
      /* haptic physics force calculation */
      PVector[] ballForces = new PVector[solidBallPos.length];
      
      for (int i=0; i < solidBallPos.length; i++) {
        PVector curBallPos = solidBallPos[i];
              /* ball and end-effector contact forces */
        posEEToBall = (curBallPos.copy()).sub(posEE);
        posEEToBallMagnitude = posEEToBall.mag();
      
        penBall = posEEToBallMagnitude - (rBall + rEE);
        /* end ball and end-effector contact forces */
      
        /* ball forces */
        if(penBall < 0){
          rEEContact = rEE + penBall;
          fContact = posEEToBall.normalize();
          velEEToBall = velBall.copy().sub(velEE);
          velEEToBall = fContact.copy().mult(velEEToBall.dot(fContact));
          velEEToBallMagnitude = velEEToBall.mag();
        }
        else{
          rEEContact = rEE;
          fContact.set(0, 0);
        }
        /* since penBall is negative kBall must be negative to ensure the force acts along the end-effector to the ball */
        fContact = fContact.mult((-kBall * penBall) - (bBall * velEEToBallMagnitude));
        fEE = (fContact.copy()).mult(-1);
        ballForces[i] = fEE;
        /* end ball forces */     
      }
      
      if (curState.equals("solid")) {
              // make sure all ball forces are applied, even when touching multiple balls
        PVector allForces = ballForces[0];
        for (int i=1; i < ballForces.length; i++) {
          allForces.add(ballForces[i]);
        }
        fEE = allForces.copy();
        fEE.set(graphics_to_device(fEE));
      }
      else if (curState.equals("liquid")){
      
        /* ball and end-effector contact forces */
        posEEToBall = (posWaterBall1.copy()).sub(posEE);
        posEEToBallMagnitude = posEEToBall.mag();
      
        penBall = posEEToBallMagnitude - (rWaterBall + rEE);
        /* end ball and end-effector contact forces */
      
      
        /* ball forces */
        if(penBall < 0){
          rEEContact = rEE + penBall;
          fContact = posEEToBall.normalize();
          velEEToBall = velBall.copy().sub(velEE);
          velEEToBall = fContact.copy().mult(velEEToBall.dot(fContact));
          velEEToBallMagnitude = velEEToBall.mag();
        
          /* since penBall is negative kBall must be negative to ensure the force acts along the end-effector to the ball */
          fContact = fContact.mult((-kBall * penBall) - (bBall * velEEToBallMagnitude));
        }
        else{
          rEEContact = rEE;
          fContact.set(0, 0);
        }
      /* end ball forces */
      
      PVector waterDamping = new PVector(0,0);
      if (posEE.y > 0.03 && posEE.y < 0.1 && penBall > 0.01 && posEE.x > -0.01 && posEE.x < 0.02) {
        waterDamping.set(0,3);
      }
      /* forces due to damping */
      fDamping = (velBall.copy()).mult(-bAir);
      /* end forces due to damping*/
      
      
      /* forces due to walls on ball */
      fWall.set(0, 0);
      
      /* left wall */
      penWall.set((posWaterBall1.x - rWaterBall) - posWallLeft.x, 0);
      if(penWall.x < 0){
        fWall = fWall.add((penWall.mult(-kWall))).add((velBall.copy()).mult(-bWall));
      }
      
      /* bottom wall */
      penWall.set(0, (posWaterBall1.y + rWaterBall) - posWallBottom.y);
      if(penWall.y > 0){
        fWall = fWall.add((penWall.mult(-kWall))).add((velBall.copy()).mult(-bWall));
      }
      
      /* right wall */
      penWall.set((posWaterBall1.x + rBall) - posWallRight.x, 0);
      if(penWall.x > 0){
        fWall = fWall.add((penWall.mult(-kWall))).add((velBall.copy()).mult(-bWall));
      }
      /* end forces due to walls on ball*/
      
      
      /* sum of forces */
      fBall = (fContact.copy()).add(fGravity).add(fDamping).add(fWall);  
      fEE = (fContact.copy()).mult(-1).add(waterDamping);
      fEE.set(graphics_to_device(fEE));
      /* end sum of forces */
      
      
      /* end haptic physics force calculation */
    
      /* dynamic state of ball calculation (integrate acceleration of ball) */
      posWaterBall1 = (((fBall.copy()).div(2*mBall)).mult(dt*dt)).add((velBall.copy()).mult(dt)).add(posWaterBall1);
      velBall = (((fBall.copy()).div(mBall)).mult(dt)).add(velBall);
      /*end dynamic state of ball calculation */
      
     
      }
      else {
        if (rotations < 2000) {
          if (posEE.x > 0) {
            rEEContact = rEE;
            fContact.set(-2, 0);
            fEE = fContact.copy().add(fDamping);
          }
          if (posEE.x > 0.05) {
            rEEContact = rEE;
            fContact.set(-2, 2);
            fEE = fContact.copy();
          }
          if (posEE.x > 0 && posEE.y > 0.1) {
            rEEContact = rEE;
            fContact.set(2, 2);
            fEE = fContact.copy();
          }
          if (posEE.x < 0.05 && posEE.y > 0.08) {
            // update rotations
            rotations = rotations + 1;
            rEEContact = rEE;
            fContact.set(-1, -1);
            fEE = fContact.copy();
          }
          if (posEE.x < 0 || posEE.y < 0.03) {
            rEEContact = rEE;
            fContact.set(0, 0);
            fEE = fContact.copy().add(fDamping);
          }
        }
        else if (rotations < 9000) {
          //println(posEE);
          if (posEE.x > -0.05 && posEE.y > 0.04) {
            rEEContact = rEE;
            fContact.set(4, 0);
            fEE = fContact.copy();
          }
          if (posEE.x < -0.05 || posEE.y < 0.03) {
            rEEContact = rEE;
            fContact.set(0, 0);
            fEE = fContact.copy().add(fDamping);
            // reset
            rotations = 0;
          }
        }
        else {
          // reset
          rotations = 0;
        }
       
      }


        /* end haptic physics force calculation */
    }
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
    
     
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/




/* helper functions section, place helper functions here ***************************************************************/
void create_pantagraph(){
  float lAni = pixelsPerMeter * l;
  float LAni = pixelsPerMeter * L;
  float rEEAni = pixelsPerMeter * rEE;
  
  pGraph = createShape();
  pGraph.beginShape();
  pGraph.fill(255);
  pGraph.stroke(0);
  pGraph.strokeWeight(2);
  
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.endShape(CLOSE);
  
  joint = createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, rEEAni, rEEAni);
  joint.setStroke(color(0));
  
  endEffector = createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, 2*rEEAni, 2*rEEAni);
  endEffector.setStroke(color(0));
  strokeWeight(5);
  
}


PShape create_wall(float x1, float y1, float x2, float y2){
  x1 = pixelsPerMeter * x1;
  y1 = pixelsPerMeter * y1;
  x2 = pixelsPerMeter * x2;
  y2 = pixelsPerMeter * y2;
  
  return createShape(LINE, deviceOrigin.x + x1, deviceOrigin.y + y1, deviceOrigin.x + x2, deviceOrigin.y+y2);
}


PShape create_ball(float rBall){
  rBall = pixelsPerMeter * rBall;
  
  return createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, 2*rBall, 2*rBall);
}


void update_animation(float th1, float th2, float xE, float yE){
  background(255);
  
  float lAni = pixelsPerMeter * l;
  float LAni = pixelsPerMeter * L;
  
  xE = pixelsPerMeter * xE;
  yE = pixelsPerMeter * yE;
  
  th1 = 3.14 - th1;
  th2 = 3.14 - th2;
  
  pGraph.setVertex(1, deviceOrigin.x + lAni*cos(th1), deviceOrigin.y + lAni*sin(th1));
  pGraph.setVertex(3, deviceOrigin.x + lAni*cos(th2), deviceOrigin.y + lAni*sin(th2));
  pGraph.setVertex(2, deviceOrigin.x + xE, deviceOrigin.y + yE);
  
    // text
  // instructions
    String s = "Can you identify the states of matter? Press any key to switch to a new state!";
    fill(0);
    textSize(20);
    text(s, 40, 40, 280, 320);
    if (curState.equals("solid")) {
      String state = "State 1";
      fill(0);
      textSize(16);
      text(state, 40, 150, 100, 100);
    }
    else if (curState.equals("liquid")) {
      String state = "State 2";
      fill(0);
      textSize(16);
      text(state, 40, 150, 100, 100);
    }
    else {
      String state = "State 3";
      fill(0);
      textSize(16);
      text(state, 40, 150, 100, 100);
    }
    
  stroke(0);
  
  
  translate(xE, yE);
  shape(endEffector);
}


PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}


PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}

void water_ball_forces(PVector posWaterBall1) {
  /* ball and end-effector contact forces */
        posEEToBall = (posWaterBall1.copy()).sub(posEE);
        posEEToBallMagnitude = posEEToBall.mag();
      
        penBall = posEEToBallMagnitude - (rBall + rEE);
        /* end ball and end-effector contact forces */
      
      
        /* ball forces */
        if(penBall < 0){
          rEEContact = rEE + penBall;
          fContact = posEEToBall.normalize();
          velEEToBall = velBall.copy().sub(velEE);
          velEEToBall = fContact.copy().mult(velEEToBall.dot(fContact));
          velEEToBallMagnitude = velEEToBall.mag();
        
          /* since penBall is negative kBall must be negative to ensure the force acts along the end-effector to the ball */
          fContact = fContact.mult((-kBall * penBall) - (bBall * velEEToBallMagnitude));
        }
        else{
          rEEContact = rEE;
          fContact.set(0, 0);
        }
      /* end ball forces */
      
      
      /* forces due to damping */
      fDamping = (velBall.copy()).mult(-bAir);
      /* end forces due to damping*/
      
      
      /* forces due to walls on ball */
      fWall.set(0, 0);
      
      /* left wall */
      penWall.set((posWaterBall1.x - rBall) - posWallLeft.x, 0);
      if(penWall.x < 0){
        fWall = fWall.add((penWall.mult(-kWall))).add((velBall.copy()).mult(-bWall));
      }
      
      /* bottom wall */
      penWall.set(0, (posWaterBall1.y + rBall) - posWallBottom.y);
      if(penWall.y > 0){
        fWall = fWall.add((penWall.mult(-kWall))).add((velBall.copy()).mult(-bWall));
      }
      
      /* right wall */
      penWall.set((posWaterBall1.x + rBall) - posWallRight.x, 0);
      if(penWall.x > 0){
        fWall = fWall.add((penWall.mult(-kWall))).add((velBall.copy()).mult(-bWall));
      }
      /* end forces due to walls on ball*/
      
      
      /* sum of forces */
      fBall = (fContact.copy()).add(fGravity).add(fDamping).add(fWall);      
      fEE = (fContact.copy()).mult(-1);
      fEE.set(graphics_to_device(fEE));
      /* end sum of forces */
      
      
      /* end haptic physics force calculation */
    
      /* dynamic state of ball calculation (integrate acceleration of ball) */
      posWaterBall1 = (((fBall.copy()).div(2*mBall)).mult(dt*dt)).add((velBall.copy()).mult(dt)).add(posWaterBall1);
      velBall = (((fBall.copy()).div(mBall)).mult(dt)).add(velBall);
      /*end dynamic state of ball calculation */
}

// change state when any key pressed
void keyPressed() {
  if (curState.equals("solid")) {
    curState = "liquid";
  }
  else if (curState.equals("liquid")) {
    curState = "gas";
  }
  else {
    curState = "solid";
  }
}
/* end helper function section *******************************************/ 
