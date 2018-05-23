import KinectPV2.*;

import SimpleOpenNI.*;
import org.openkinect.processing.*;
SimpleOpenNI  kinect;
// import the processing serial library
import processing.serial.*;
// and declare an object for our serial port
Serial port;

int[] userID;
// user colors
color[] userColor = new color[]{ color(255,0,0), 
                                 color(0,255,0), 
                                 color(0,0,255),
                                 color(255,255,0), 
                                 color(255,0,255), 
                                 color(0,255,255)};
                                 
                                 
// postion of head to draw circle
PVector headPosition = new PVector();
// turn headPosition into scalar form
float distanceScalar;
// diameter of head drawn in pixels
float headSize = 200;
 // threshold of level of confidence
float confidenceLevel = 0.5;
// the current confidence level that the kinect is tracking
float confidence;
// vector of tracked head for confidence checking
PVector confidenceVector = new PVector();

void setup() {
  size(640, 480);
  kinect = new SimpleOpenNI(this);
  kinect.enableDepth();
  kinect.enableRGB();
  kinect.enableUser();
  kinect.setMirror(false);
  // Get the name of the first serial port
  // where we assume the Arduino is connected.
  // If it doesn't work, examine the output of
  // the println, and replace 0 with the correct
  // serial port index.
  println(Serial.list());
  //String portName = Serial.list()[1];
  // initialize our serial object with this port
  // and the baud rate of 9600
   port = new Serial(this, "COM3", 9600);
   port.bufferUntil('\n');
}
void draw() {
  kinect.update();
  PImage depthImage = kinect.depthImage();
  PImage rgbImage = kinect.rgbImage();
  image(depthImage, 0, 0);
  image(rgbImage, 640, 0);
  IntVector userList = new IntVector();
 // get all user IDs of tracked users
  userID = kinect.getUsers();

  // loop through each user to see if tracking
  for(int i=0;i<userID.length;i++)
  {
    if(kinect.isTrackingSkeleton(userID[i]))
    {
  // get the positions of the  joints  
  
  drawSkeleton(userID[i]);
  // get confidence level that Kinect is tracking head
      confidence = kinect.getJointPositionSkeleton(userID[i],SimpleOpenNI.SKEL_HEAD,confidenceVector);
 
      // if confidence of tracking is beyond threshold, then track user
      if(confidence > confidenceLevel)
      {
        // change draw color based on hand id#
        stroke(userColor[(i)]);
        // fill the ellipse with the same color
        fill(userColor[(i)]);
        // draw the rest of the body
        drawSkeleton(userID[i]);
 
   //right arm
      PVector rightHand = new PVector();
      kinect.getJointPositionSkeleton(1, SimpleOpenNI.SKEL_RIGHT_HAND, rightHand);
      PVector rightElbow = new PVector();
      kinect.getJointPositionSkeleton(1, SimpleOpenNI.SKEL_RIGHT_ELBOW, rightElbow);
      PVector rightShoulder = new PVector();
      kinect.getJointPositionSkeleton(1, SimpleOpenNI.SKEL_RIGHT_SHOULDER, rightShoulder);
      
    //right leg
      PVector rightKnee = new PVector();
      kinect.getJointPositionSkeleton(1, SimpleOpenNI.SKEL_RIGHT_KNEE, rightKnee);
      PVector rightHip = new PVector();
      kinect.getJointPositionSkeleton(1, SimpleOpenNI.SKEL_RIGHT_HIP, rightHip);
      PVector rightFoot = new PVector();
      kinect.getJointPositionSkeleton(1, SimpleOpenNI.SKEL_RIGHT_FOOT, rightFoot); 
      
    //neck for depth
      PVector torso = new PVector();
      kinect.getJointPositionSkeleton(1, SimpleOpenNI.SKEL_TORSO, torso);         
      
   // convert our arm joints into screen space coordinates
    //right arm
      PVector convertedRightHand = new PVector();
      kinect.convertRealWorldToProjective(rightHand, convertedRightHand);
      PVector convertedRightElbow = new PVector();
      kinect.convertRealWorldToProjective(rightElbow, convertedRightElbow);
      PVector convertedRightShoulder = new PVector();
      kinect.convertRealWorldToProjective(rightShoulder, convertedRightShoulder);
   //right leg
      PVector convertedRightHip = new PVector();
      kinect.convertRealWorldToProjective( rightHip, convertedRightHip);
      PVector convertedRightFoot = new PVector();
      kinect.convertRealWorldToProjective( rightFoot, convertedRightFoot);
      PVector convertedRightKnee = new PVector();
      kinect.convertRealWorldToProjective( rightKnee, convertedRightKnee);
     
     
     
     // reduce our joint vectors to two dimensions
      PVector rightHand2D = new PVector(rightHand.x, rightHand.y);
      PVector rightHand2Dz = new PVector(rightHand.z,  rightHand.y);
      PVector rightElbow2D = new PVector(rightElbow.x, rightElbow.y);
      PVector rightElbow2Dz = new PVector(rightElbow.z, rightElbow.y);
      
      PVector rightShoulder2Dz = new PVector(rightShoulder.z, rightShoulder.y);
      PVector rightShoulder2D = new PVector(rightShoulder.x, rightShoulder.y);
      PVector rightHip2D = new PVector(rightHip.x, rightHip.y);
      PVector rightHip2Dz = new PVector(rightHip.z, rightHip.y);
      PVector rightFoot2D = new PVector(rightFoot.z, rightFoot.y);
      PVector rightKnee2D = new PVector(rightKnee.z, rightKnee.y);    
      
    // calculate the axes against which we want to measure our angles
      PVector torsoOrientationr = PVector.sub(rightShoulder2D, rightHip2D);
      PVector upperArmOrientationr = PVector.sub(rightElbow2D, rightShoulder2D);
      PVector newarmr = PVector.sub(rightShoulder2Dz, rightHand2D);
      PVector upperLegOrientationr = PVector.sub(rightKnee2D, rightHip2D);
      PVector torsoOrientationrz = PVector.sub(rightShoulder2Dz, rightHip2Dz);
      
    // calculate the angles of each of our arms
      float shoulderAngler =angleOf(rightElbow2D, rightShoulder2D, torsoOrientationr);
      float elbowAngler =angleOf(rightHand2D, rightElbow2D, upperArmOrientationr);
      float shoulderAnglezr =angleOf(rightHand2Dz, rightShoulder2Dz, torsoOrientationrz);
      //float shoulderAnglezr =angleOf(rightElbow2Dz, rightShoulder2Dz, torsoOrientationrz);
      
      float legAngler =angleOf(rightFoot2D, rightKnee2D, upperLegOrientationr);
      
      //float shoulderAnglezl =angleOf(leftElbow2Dz, leftShoulder2Dz, torsoOrientationlz);
      
    
      float[] torsof  = torso.array();
 

// gribber control
float rightf= abs(rightFoot.z/500);
int rf =int(rightf);

// right gribber 
int gribr=90 ;
              
               if(rf<=3 ) { gribr=180; } 
                    else {gribr=45;}               
      
      // show the angles on the screen for debugging
      fill(0,0,255);
      scale(2);
      text("shoulder_r: " + int(shoulderAngler) + "\n" +
           " elbow_r: " + int(elbowAngler) + "\n" +
           " Arm_r: " + int(shoulderAnglezr) + "\n" + 
           "torsof_x" + byte(torsof[0]/100) + "\n" +
           "torsof_z" + byte(torsof[2]/500) + "\n" +
           "foort_z" + int(abs(rightFoot.z/500)) + "\n" +
           "gribr" + int(gribr), 20, 20);

    
    byte shoulderAngler_b = byte(shoulderAngler);
    byte elbowAngler_b = byte(elbowAngler);
    byte shoulderAnglezr_b = byte(shoulderAnglezr);
    byte gribr_b = byte(gribr);
    
     
      port.write(shoulderAngler_b);
      port.write(elbowAngler_b);
      port.write(shoulderAnglezr_b);
      port.write(gribr_b);
    }
    }

} 

}


float angleOf(PVector one, PVector two, PVector axis) {
  PVector limb = PVector.sub(two, one);
  return degrees(PVector.angleBetween(limb, axis));
}

/*---------------------------------------------------------------
Draw the skeleton of a tracked user.  Input is userID
----------------------------------------------------------------*/
void drawSkeleton(int userId){
   // get 3D position of head
  kinect.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_HEAD,headPosition);
  // convert real world point to projective space
  kinect.convertRealWorldToProjective(headPosition,headPosition);
  // create a distance scalar related to the depth in z dimension
  distanceScalar = (525/headPosition.z);
  // draw the circle at the position of the head with the head size scaled by the distance scalar
  ellipse(headPosition.x,headPosition.y, distanceScalar*headSize,distanceScalar*headSize);
 
  //draw limb from head to neck
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_HEAD, SimpleOpenNI.SKEL_NECK);
  //draw limb from neck to left shoulder
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_LEFT_SHOULDER);
  //draw limb from left shoulde to left elbow
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_LEFT_ELBOW);
  //draw limb from left elbow to left hand
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_ELBOW, SimpleOpenNI.SKEL_LEFT_HAND);
  //draw limb from neck to right shoulder
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_RIGHT_SHOULDER);
  //draw limb from right shoulder to right elbow
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_RIGHT_ELBOW);
  //draw limb from right elbow to right hand
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, SimpleOpenNI.SKEL_RIGHT_HAND);
 //draw limb from left shoulder to torso
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_TORSO);
  //draw limb from right shoulder to torso
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_TORSO);
  //draw limb from torso to left hip
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_LEFT_HIP);
  //draw limb from left hip to left knee
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_HIP,  SimpleOpenNI.SKEL_LEFT_KNEE);
  //draw limb from left knee to left foot
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_KNEE, SimpleOpenNI.SKEL_LEFT_FOOT);
  //draw limb from torse to right hip
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_RIGHT_HIP);
  //draw limb from right hip to right knee
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_HIP, SimpleOpenNI.SKEL_RIGHT_KNEE);
  //draw limb from right kneee to right foot
  kinect.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_KNEE, SimpleOpenNI.SKEL_RIGHT_FOOT);
} // void drawSkeleton(int userId)
 
/*---------------------------------------------------------------
When a new user is found, print new user detected along with
userID and start pose detection.  Input is userID
----------------------------------------------------------------*/
void onNewUser(SimpleOpenNI curContext, int userId){
  println("New User Detected - userId: " + userId);
  // start tracking of user id
  curContext.startTrackingSkeleton(userId);
} //void onNewUser(SimpleOpenNI curContext, int userId)
 
/*---------------------------------------------------------------
Print when user is lost. Input is int userId of user lost
----------------------------------------------------------------*/
void onLostUser(SimpleOpenNI curContext, int userId){
  // print user lost and user id
  println("User Lost - userId: " + userId);
} //void onLostUser(SimpleOpenNI curContext, int userId)
 
/*---------------------------------------------------------------
Called when a user is tracked.
----------------------------------------------------------------*/
void onVisibleUser(SimpleOpenNI curContext, int userId)
{
    // print user lost and user id
  println("visible - userId: " + userId);
} //void onVisibleUser(SimpleOpenNI curContext, int userId)