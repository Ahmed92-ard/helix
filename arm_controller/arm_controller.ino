#include <Servo.h>

// ---------------- SERVOS ----------------
Servo base, shoulder, elbow, wrist, wristrot, gripper;

// live joint tracking
int pos[6] = {45,40,10,10,170,10};

// first startup pickup
bool firstPickupDone = false;

// ---------------- ATTACH ----------------
void attachAll(){
  base.attach(7);
  shoulder.attach(6);
  elbow.attach(5);
  wrist.attach(4);
  wristrot.attach(3);
  gripper.attach(2);
  base.write(pos[0]);
  shoulder.write(pos[1]);
  elbow.write(pos[2]);
  wrist.write(pos[3]);
  wristrot.write(pos[4]);
  gripper.write(pos[5]);
}

// --------------------------------------------------
// FULL 6-AXIS MOTION (used for travel)
void moveTo(int b,int s,int e,int w,int wr,int g,int speedDelay){
  int target[6]={b,s,e,w,wr,g};
  bool moving = true;
  while(moving){
    moving=false;
    for(int i=0;i<6;i++){
      if(pos[i] < target[i]){ pos[i]++; moving=true; }
      else if(pos[i] > target[i]){ pos[i]--; moving=true; }
    }
    base.write(pos[0]);
    shoulder.write(pos[1]);
    elbow.write(pos[2]);
    wrist.write(pos[3]);
    wristrot.write(pos[4]);
    gripper.write(pos[5]);
    delay(speedDelay);
  }
}

// --------------------------------------------------
// ARM-ONLY MOTION (gripper locked)
void moveArmOnly(int b,int s,int e,int w,int wr,int speedDelay){
  bool moving = true;
  while(moving){
    moving=false;
    if(pos[0] < b){ pos[0]++; moving=true; }
    else if(pos[0] > b){ pos[0]--; moving=true; }
    if(pos[1] < s){ pos[1]++; moving=true; }
    else if(pos[1] > s){ pos[1]--; moving=true; }
    if(pos[2] < e){ pos[2]++; moving=true; }
    else if(pos[2] > e){ pos[2]--; moving=true; }
    if(pos[3] < w){ pos[3]++; moving=true; }
    else if(pos[3] > w){ pos[3]--; moving=true; }
    if(pos[4] < wr){ pos[4]++; moving=true; }
    else if(pos[4] > wr){ pos[4]--; moving=true; }
    base.write(pos[0]);
    shoulder.write(pos[1]);
    elbow.write(pos[2]);
    wrist.write(pos[3]);
    wristrot.write(pos[4]);
    // gripper NOT touched
    delay(speedDelay);
  }
}

// ---------------- GRIPPER ----------------
void closeGrip(){
  gripper.write(10);
  pos[5]=5;
  delay(300);
}
void openGrip(){
  gripper.write(30);
  pos[5]=30;
  delay(300);
}

// ---------------- POSES ----------------
// A station
int A_above[6] = {170,135,70,20,70,10};
int A_pick [6] = {170,170,70,20,70,10};
// B station
int B_above[6] = {90,135,70,20,80,10};
int B_pick [6] = {90,170,70,20,70,10};

// ---------------- HANDOFF ----------------
void handoff(int pickPose[6], int abovePose[6]){
  // 1) LOWER ARM ONLY (gripper stays closed, holding ball)
  moveArmOnly(pickPose[0],pickPose[1],pickPose[2],pickPose[3],pickPose[4],12);
  delay(500);

  // 2) RELEASE ball onto surface
  openGrip();
  delay(300);

  // 3) RE-GRAB in same position (no lift)
  closeGrip();
  delay(300);

  // 4) LIFT AND TRAVEL to above pose
  moveTo(abovePose[0],abovePose[1],abovePose[2],abovePose[3],abovePose[4],pos[5],10);
}

// ---------------- SETUP ----------------
void setup(){
  attachAll();
  delay(1500);
  // safe home
  moveTo(45,40,10,10,170,10,15);
  delay(1500);
}

// ---------------- LOOP ----------------
void loop(){

  // FIRST PICKUP — open gripper BEFORE descending, then grab
  if(!firstPickupDone){
    moveTo(A_above[0],A_above[1],A_above[2],A_above[3],A_above[4],5,5);
    openGrip();           // ← KEY FIX: open before descending to ball
    delay(300);
    moveArmOnly(A_pick[0],A_pick[1],A_pick[2],A_pick[3],A_pick[4],5);
    delay(400);           // settle on ball
    closeGrip();          // grab ball
    delay(300);
    moveTo(A_above[0],A_above[1],A_above[2],A_above[3],A_above[4],5,5);
    firstPickupDone = true;
  }

  // A → B
  moveTo(B_above[0],B_above[1],B_above[2],B_above[3],B_above[4],pos[5],5);
  handoff(B_pick, B_above);

  // B → A
  moveTo(A_above[0],A_above[1],A_above[2],A_above[3],A_above[4],pos[5],5);
  handoff(A_pick, A_above);
}
