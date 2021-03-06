#include <Servo.h>
#include "LedControl.h"
#include <Wire.h>        //调用IIC库函数
#include "MH_TCS34725.h" //调用颜色识别传感器库函数
#ifdef __AVR__
#include <avr/power.h>
#endif
#include <MsTimer2.h>
#define grayScale1 A2
#define grayScale2 A3
#define motor1L 5
#define motor1R 6
#define motor2L 9
#define motor2R 10
#define SERVO 11


// 常量
uint8_t SPEED = 200;
uint8_t theta = 20; // 左右轮速度差
uint8_t deltaPWM = 180;
const uint16_t encoderTime = 250;

//初始化目标速度
uint8_t expectedPWML = SPEED - theta;
uint8_t expectedPWMR = SPEED;

//采集到黑线时，传感器为低电平
int gS1 = 1;
int gS2 = 1;
uint8_t le=0;
uint8_t ri=0;
const uint8_t thresholdR = 180;  // 180
const uint8_t thresholdL = 180; //  200
int gSL[2]={100,100};
int gSR[2]={100,100};
//LedControl lc = LedControl(12, 11, 13, 1);

// 颜色识别相关
// 卡片颜色
uint8_t color = 0, card_color = 0;
// 颜色传感器
MH_TCS34725 tcs = MH_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X); // 50ms
// MH_TCS34725 tcs = MH_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);//700ms

// 初始化舵机
//Servo my_servo;
// 舵机位置
uint8_t DOWN = 0, UP = 180;

// 时间定时器
const uint16_t upperBound = 1000; 
uint16_t time_counter = 0;
uint16_t sensor_delay = 650;

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(9600);
    pinMode(grayScale1, INPUT);
    pinMode(grayScale2, INPUT);
    pinMode(motor1L, OUTPUT);
    pinMode(motor1R, OUTPUT);
    pinMode(motor2L, OUTPUT);
    pinMode(motor2R, OUTPUT);
//    MsTimer2::set(encoderTime, onTimer);
//    MsTimer2::start();
//    lc.shutdown(0, false);
//    lc.setIntensity(0, 8);
//    lc.clearDisplay(0);
    if (!tcs.begin())
    {
        while (1)
            ;
    }
    pinMode(SERVO, OUTPUT);
}

void loop()
{
    /********************
     *  传感器调试程序
     * /
    //   Serial.print(analogRead(grayScale1));
    //   Serial.print(",");
    //   Serial.println(analogRead(grayScale2));
    /*******************/
    /********************
     *  当终端引发次数超过upperBound后进入扎气球程序
     */
//    if (time_counter == upperBound)
//    {
//        hitBall();
//    }
//      colorDetect();
      pidMotorControl();
      delay(100);
}

//定时器中断函数
void onTimer(){
  ++time_counter;
}

/******************************
 *  扎气球程序
 * ***************************/
//void hitBall()
//{
//    color = colorDetect();
//    if (color != 0 && card_color == 0)
//    {
//        card_color = color; // 记录色卡颜色
////        showColor(card_color);
//        expectedPWML -= 120;
//        expectedPWMR -= 120;
//        color = 0;
////        lc.clearDisplay(0);
//        delay(100);
//    }
//    if (card_color != 0 && color == card_color)
//    {
//        motorRun(0, 0);
//        expectedPWML += 20;
//        expectedPWMR += 20;
//        my_servo.attach(SERVO);
//        my_servo.write(UP);
//        delay(sensor_delay);
//        my_servo.write(DOWN);
//        delay(sensor_delay);
//        my_servo.detach();
//    }
//}

/*****************************
 * 马达驱动
 * ***************************/
void motorRun(int left, int right)
{
    left = left > 255 ? 255 : left;
    left = left <= 0 ? 0 : left;
    right = right > 255 ? 255 : right;
    right = right <= 0 ? 0 : right;
    analogWrite(motor1L, left);
    analogWrite(motor2R, right);
}

/*****************************
 * 8x8颜色显示模块
 ****************************/
//void turnRight()
//{
//    lc.setRow(0, 0, B00010000);
//    lc.setRow(0, 1, B11111111);
//    lc.setRow(0, 2, B00100000);
//    lc.setRow(0, 3, B00100000);
//    lc.setRow(0, 4, B01111110);
//    lc.setRow(0, 5, B11100010);
//    lc.setRow(0, 6, B11100010);
//    lc.setRow(0, 7, B01111110);
//}
//
//void turnLeft()
//{
//    lc.setRow(0, 0, B00010000);
//    lc.setRow(0, 1, B11111111);
//    lc.setRow(0, 2, B00100000);
//    lc.setRow(0, 3, B00100010);
//    lc.setRow(0, 4, B01111110);
//    lc.setRow(0, 5, B11001000);
//    lc.setRow(0, 6, B10001000);
//    lc.setRow(0, 7, B01111111);
//}
//
//void forward()
//{
//    lc.setRow(0, 0, B00000000);
//    lc.setRow(0, 1, B00111100);
//    lc.setRow(0, 2, B00100000);
//    lc.setRow(0, 3, B00111100);
//    lc.setRow(0, 4, B00111100);
//    lc.setRow(0, 5, B00100000);
//    lc.setRow(0, 6, B00100000);
//    lc.setRow(0, 7, B00000000);
//}
//
//void down()
//{
//    lc.setRow(0, 0, B00000000);
//    lc.setRow(0, 1, B00111100);
//    lc.setRow(0, 2, B00100100);
//    lc.setRow(0, 3, B00100100);
//    lc.setRow(0, 4, B00100100);
//    lc.setRow(0, 5, B00100100);
//    lc.setRow(0, 6, B00111000);
//    lc.setRow(0, 7, B00000000);
//}
//
//void showColor(uint8_t color)
//{
//    if (color == 1)
//    {
//        // red
//        lc.setRow(0, 0, B00000000);
//        lc.setRow(0, 1, B00011100);
//        lc.setRow(0, 2, B00010110);
//        lc.setRow(0, 3, B00011100);
//        lc.setRow(0, 4, B00011100);
//        lc.setRow(0, 5, B00010100);
//        lc.setRow(0, 6, B00010100);
//        lc.setRow(0, 7, B00000000);
//    }
//
//    else if (color == 2)
//    {
//        // green
//        lc.setRow(0, 0, B00001000);
//        lc.setRow(0, 1, B00011100);
//        lc.setRow(0, 2, B00010100);
//        lc.setRow(0, 3, B00010000);
//        lc.setRow(0, 4, B00011100);
//        lc.setRow(0, 5, B00010000);
//        lc.setRow(0, 6, B00011100);
//        lc.setRow(0, 7, B00000000);
//    }
//    else if (color == 3)
//    {
//        // blue
//        lc.setRow(0, 0, B00000000);
//        lc.setRow(0, 1, B00011100);
//        lc.setRow(0, 2, B00010100);
//        lc.setRow(0, 3, B00011100);
//        lc.setRow(0, 4, B00010110);
//        lc.setRow(0, 5, B00010010);
//        lc.setRow(0, 6, B00011100);
//        lc.setRow(0, 7, B00000000);
//    }
//    else
//    {
//        // no color detected
//        lc.setRow(0, 0, B00000000);
//        lc.setRow(0, 1, B00110100);
//        lc.setRow(0, 2, B00110100);
//        lc.setRow(0, 3, B00101100);
//        lc.setRow(0, 4, B00101100);
//        lc.setRow(0, 5, B00101100);
//        lc.setRow(0, 6, B00100100);
//        lc.setRow(0, 7, B00000000);
//    }
//}

/*******************************/

/*****************************
 * 颜色识别模块
 ****************************/
 uint8_t colorDetect()
{
  // get the rgb from the tensor
  uint16_t clear, red, green, blue; 
  tcs.getRGBC(&red, &green, &blue, &clear); 
  tcs.lock();  //禁用中断（可省略）
  uint32_t sum = clear;           
  float r, g, b;                  
  r = red; r /= sum;             
  g = green; g /= sum;            
  b = blue; b /= sum;             
  r *= 256; g *= 256; b *= 256;
  // transform rgb to hsv
  int rgb[3];  
  int hsv[3];
  rgb[0]=(int)r; rgb[1]=(int)g; rgb[2]=(int)b;
  int mx = max(rgb[0], max(rgb[1], rgb[2]));
  int mi = min(rgb[0], min(rgb[1], rgb[2]));
  hsv[1] = (mx - mi);
  hsv[2] = mx;
  if (mx == mi) hsv[0] = 0;
  else{
    if (mx == rgb[0]){
      if (rgb[1] > rgb[2]){
        hsv[0] = 60 * (rgb[1] - rgb[2])/(mx - mi);
      }
      else {
        hsv[0] = 60 * (rgb[1] - rgb[2])/(mx - mi)+360;
      }
    }
    else if (mx == rgb[1]){
      hsv[0] = 60 * (rgb[2] - rgb[0])/(mx - mi) + 120;
    }
    else {
      hsv[0] = 60 * (rgb[0] - rgb[1])/(mx - mi) + 240;
    }
  }
  // check the color
  Serial.print(hsv[0]); Serial.print(",");
  Serial.print(hsv[1]); Serial.print(",");
  Serial.println(hsv[2]);
  if (hsv[2] > 80 && hsv[1] > 130) {
    if (hsv[0] >= 340 && hsv[0] < 360) return 1;
    else if (hsv[0] >=120 && hsv[0] <= 150) return 2;
    else if (hsv[0] >= 220 && hsv[0] <= 240) return 3;
    else return 0;
  }
  else return 0;
}

//uint8_t colorDetect()
//{
//    tcs.getRGBC(&red, &green, &blue, &sum);
//    tcs.lock();
//    // 提取颜色值
//    r = red;
//    r /= sum;
//    g = green;
//    g /= sum;
//    b = blue;
//    b /= sum;
//    // 颜色转换
//    red = transform(r);
//    green = transform(g);
//    blue = transform(b);
//    Serial.println(green);
//    if (red > green && red > blue && red > thresh_red)
//    {
//        return 1;
//    }
//    if (green > red && green > blue && green > thresh_green)
//    {
//        return 2;
//    }
//    if (blue > red && blue > green && blue > thresh_blue)
//    {
//        return 3;
//    }
//    else
//        return 0;
//}

//uint32_t transform(float a)
//{
//    uint32_t ans = exp(10 * a) * 10;
//    if (ans >= 10000)
//        return 0;
//    else
//        return ans;
//}
/***************************/

void pidMotorControl()
{
    gSL[le] = analogRead(grayScale1);
    gSR[ri] = analogRead(grayScale2);
    Serial.print((gSL[le] + gSL[!le])/2); Serial.print(",");
    Serial.println((gSR[ri] + gSR[!ri])/2);
    gS1 = (gSL[le] + gSL[!le])/2 > thresholdL ? 1 : 0;
    gS2 = (gSR[ri] + gSR[!ri])/2 > thresholdR ? 1 : 0;
    le = !le; ri = !ri;
    //直行
    if (gS1 == 1 && gS2 == 1)
    {
        motorRun(expectedPWML, expectedPWMR);
    }
    //左转
    if (gS1 == 0 && gS2 == 1)
    {
        motorRun(expectedPWML - deltaPWM, expectedPWMR + deltaPWM);
    }
    //右转
    if (gS1 == 1 && gS2 == 0)
    {
        motorRun(expectedPWML + deltaPWM, expectedPWMR - deltaPWM);
    }
    //爬坡和爬楼梯
    if (gS1 == 0 && gS2 == 0)
    {
        motorRun(expectedPWML, expectedPWMR);
    }
}
