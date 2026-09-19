#include <Arduino.h>

#include <PS4Controller.h>

#include "cytron_dcmd.hpp"
CytronDCMD L(17,5,0);
CytronDCMD R(16, 4, 1);

#include <MadgwickAHRS.h>
#include "MPU6500.hpp"
Madgwick mad;

#include <PID.hpp>

Pid pid(3.2,0,0.08);

//補正値
float pitchE=0, rollE=0, yawE=0;

//ループを100Hzで動かすための処理に使う変数
uint32_t urhz = 100;
uint32_t uri = 1000000 / urhz;
uint32_t lastU = 0;

void setup(){
  Serial.begin(115200);

  const char* mac = "8C:94:DF:54:14:C6";
  PS4.begin(mac);
  
  L.begin();
  R.begin();
  L.stop();
  R.stop();

  pid.setTarget(0);

  lastU = micros();
  mad.begin(100);
  beginMPU(22,23);
  setRange();

  delay(100);

  //初期化時の状態を0度として設定
  Serial.println("キャリブレーション中...水平を保ってください");
  for(int i=0;i<100;i++){
    readData();
    mad.updateIMU(gx, gy, gz, ax, ay, az);
    delay(10);
  }
  pitchE = -mad.getPitch();
  rollE = -mad.getRoll();
  yawE = -mad.getYaw();
  Serial.println("キャリブレーション完了！");

}

//最低回転保証出力の絶対値=4
int32_t roll=0, pitch=0, Lpwm=0, Rpwm=0, lastRoll=0, lastPitch=0;

//旋回・前後移動 制御変数
int32_t rotate=0, run=0;

void loop(){
  uint32_t now = micros();
  if (now - lastU >= uri) {
    lastU += uri;

    readData();
    mad.updateIMU(gx, gy, gz, ax, ay, az);

    //安全停止
    if(abs(mad.getPitch()+pitchE) > 35){
      L.stop();
      R.stop();
      while(true);
    }

    //前後の操作
    /*if(PS4.Up()){
      run = 2;
    }else if(PS4.Down()){
      run = -2;
    }else{
      run = 0;
    }*/
    pid.setTarget(run);

    roll = mad.getRoll()+rollE;
    pitch = mad.getPitch()+pitchE;

    pid.update(pitch, uri/1000.0);

    int32_t output = pid.getOutput();

    //旋回操作
    //走行中は旋回速度を下げる
    if(PS4.Right() || PS4.Circle()){
      rotate = abs(output)>15 ? -10: -20;
    }else if(PS4.Left() || PS4.Square()){
      rotate = abs(output)>15 ? 10 : 20;
    }else{
      rotate=0;
    }
    
    if(lastRoll!=roll || lastPitch!=pitch){
      lastRoll = roll;
      lastPitch = pitch;
      Serial.printf("%d,\t %d,\t %d\n", roll, pitch, output);
    }

    Lpwm = output-rotate;
    Rpwm = output+rotate;

    if(abs(Lpwm)>100) Lpwm = Lpwm<0?-100:100;
    if(abs(Rpwm)>100) Rpwm = Rpwm<0?-100:100;

    L.run(-Lpwm);
    R.run(Rpwm);
  }
}
