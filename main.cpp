//Embedded Challenge Fall 2023

//Team members - 
//Yujie Zhu  Siddharth Singh  Siyuan Zhu
//yz9461     ss16915          sz4277 


#include <mbed.h>
#include <cmath>
#include "drivers/LCD_DISCO_F429ZI.h"
#define PI 3.141592654
#define BACKGROUND 1
#define FOREGROUND 0
#define GRAPH_PADDING 5


volatile int flag = 0;
SPI spi(PF_9, PF_8, PF_7);
DigitalOut cs(PC_1);
double dist = 0;//this is the total distance.
double LegLength = 0.8;
double TimeInterval=0.5;
int numberVelocity = 0;
double ForwardVelocity=0;
double LastForwardVelocity=0;
double AngularVelocity=0;
double LastAngularVelocity=0;
double LastDist=0;

// Initial for ConvertDistanceMethod 1
double SumVelocity = 0;

//Initial for led
LCD_DISCO_F429ZI lcd;
char display_buf[2][60];
uint32_t graph_width=lcd.GetXSize()-2*GRAPH_PADDING;
uint32_t graph_height=graph_width;



void setFlag();
int16_t readRegister(int reg);
double NormalDistributionRand();
void ConvertDistanceMethod1(double x);
void ConvertDistanceMethod2(double x1, double x2);
void ConvertDistanceMethod3(double x);
double ConvertRealAngularVelocity(int16_t x);
double ConvertForwardVelocity(double x);
void draw_graph_window(uint32_t horiz_tick_spacing);
void setup_background_layer();
void setup_foreground_layer();

int main() {
  cs=0;
  spi.write(0x20);
  spi.write(0b11001111);//inital the sensor
  cs=1;
  spi.format(8,3);
  spi.frequency(100000);
  Ticker t;
  t.attach(&setFlag,0.5); // Time interval is 0.5s
  setup_background_layer();
  setup_foreground_layer();
  lcd.DisplayStringAt(0,LINE(1),(uint8_t*)"Start to Measure",CENTER_MODE);
  lcd.DisplayStringAt(graph_width-10,graph_width+50,(uint8_t*)"Time",RIGHT_MODE);
  lcd.DisplayStringAt(graph_width-10,LINE(3),(uint8_t*)"Angular Velocity",RIGHT_MODE);
  lcd.DisplayChar(5, graph_width+5, '0');

  draw_graph_window(10);
 



  while(1) {
    if(flag){
        //int16_t x = readRegister(0xA8);
        //int16_t y = readRegister(0xAA);
        int16_t z = readRegister(0xAC);  
        // Data preprocessing to obtain the real angular velocity 
        AngularVelocity = ConvertRealAngularVelocity(z); 
        AngularVelocity = abs(AngularVelocity);
        numberVelocity++;

        ForwardVelocity = ConvertForwardVelocity(AngularVelocity); //Get Forward linear Velocity


        //If you want to use a certain method to measure distance, remember to comment on the other methods.

        // Method 1
        //ConvertDistanceMethod1(ForwardVelocity);
  
        // Method 2
        if(numberVelocity == 1){
          dist = dist + (ForwardVelocity*TimeInterval/2);
        }
        else if(numberVelocity >=400){
           ConvertDistanceMethod2(LastForwardVelocity,ForwardVelocity);
           dist = dist + (ForwardVelocity*TimeInterval/2);
        }
        else{
          ConvertDistanceMethod2(LastForwardVelocity,ForwardVelocity);
        }
        printf("Distance is %d \n", (int) dist);
        
        //Method 3
        //ConvertDistanceMethod3(Angle);



        int lav = (int)LastAngularVelocity;
        int av = (int)AngularVelocity;

        lcd.DrawLine((numberVelocity)*5,graph_width-lav,(1+numberVelocity)*5,graph_width-av);
        char display_buf[60];
        snprintf(display_buf,60,"Distance is %d",(int)dist);
        lcd.DisplayStringAt(0, graph_width+60, (uint8_t *)display_buf, LEFT_MODE);
        LastForwardVelocity = ForwardVelocity;
        LastAngularVelocity = AngularVelocity;
        LastDist = dist;
        flag = 0;
        if(numberVelocity>=41){ //only record 20 seconds
          flag = 0;
          break;
          }
    }
    
  }
}

void setFlag() {flag = 1;}

//**********************
//*** This is 20% score for Ability to successfully and continuously measure gyro values from the angular velocity sensor
int16_t readRegister(int reg){
  cs = 0;
  spi.write(reg);
  uint8_t xl =spi.write(0x00);
  cs = 1;
  cs = 0;
  spi.write(reg+1);
  int8_t xh =spi.write(0x00);
  cs = 1;
  int16_t data = ( ( (uint16_t)xh) <<8 ) | ( (uint16_t)xl );
  return data;
}

// this function will generate a random value that is in standard Normal Distribution N(0, 1);
// As for random value in N(mu, sigma^2), only need to z = x*sigma+mu; where x ~ N(0, 1).

double NormalDistributionRand(){
     double U, V; 
     double x;  
          U = rand() / (RAND_MAX + 1.0); 
          V = rand() / (RAND_MAX + 1.0);           
          x = sqrt(-2.0 * log(U))* sin(2.0 * PI * V);  
          return x; 
}

//*************
//*** Both Data preprocessing and graph drawing are 15% score for Creativity

double ConvertRealAngularVelocity(int16_t x){
  double real = x;
  // a white noise error that is in Normal Distributio N(74.5, 37.46^2);
  double epsilon = NormalDistributionRand()*37.46+74.5;
  real -= epsilon;
  // according to page 10 of the datasheet, we need to times 8.75*10^-3 to get the real data.
  real = real*0.00875;
  return real;
}

//**********************
//***This is 30% score for Ability to convert measured data to forward movement velocity
double ConvertForwardVelocity(double x){
    //V = wr; linear velocity equals to angular velocity multiplied by radius.
    return (x*LegLength*2*PI/360);     
}

//***************
//*** These three methods are 20% score for Ability to calculate distance traveled

// Method 1 Calculate the distance by multiplying the average linear speed by time
void ConvertDistanceMethod1(double x){
  SumVelocity += x;
  dist = SumVelocity*TimeInterval;
  printf("Distance is %d \n", (int) dist);
}

//Method 2 
// Calculate the distance for 0.5s by multiplying the average of two adjacent linear velocities and the time period then sum up
void ConvertDistanceMethod2(double x1, double x2){
    double DeltaDist = (x1+x2)/2*TimeInterval;
    dist += DeltaDist;
}

//Method3 
//Angular velocity multiplied by time is the angle. Use the angle to calculate the chord length and then sum up.
void ConvertDistanceMethod3(double x){
    double angle = x*TimeInterval;
    double DeltaDist = 2*LegLength*sin(angle/2*2*PI/360); //Chord L=2*R*SIN(A/2)
    dist += DeltaDist;
    printf("Distance is %d \n", (int) dist);     
}

//*************
//*** Both Data preprocessing and graph drawing are 15% score for Creativity
void draw_graph_window(uint32_t horiz_tick_spacing){
  lcd.SelectLayer(BACKGROUND);  
  lcd.DrawRect(GRAPH_PADDING,40,graph_width,graph_width);
  //draw the x-axis tick marks
  for (int32_t i = 0 ; i < graph_width;i+=horiz_tick_spacing){
    lcd.DrawVLine(GRAPH_PADDING+i,graph_height+30,GRAPH_PADDING+5);
  }
}

void setup_background_layer(){
  lcd.SelectLayer(BACKGROUND);
  lcd.Clear(LCD_COLOR_BLACK);
  lcd.SetBackColor(LCD_COLOR_BLACK);
  lcd.SetTextColor(LCD_COLOR_GREEN);
  lcd.SetLayerVisible(BACKGROUND,ENABLE);
  lcd.SetTransparency(BACKGROUND,0x7Fu);
}

void setup_foreground_layer(){
    lcd.SelectLayer(FOREGROUND);
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_LIGHTGREEN);
}



