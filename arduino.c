#define lefta 5
#define leftb 6
#define righta 10
#define rightb 11


#define R 3.2
#define L 14
//mpr=2*pi*R/20
//actual(16)1.26 tick might be 20 so, use 20
#define mpr 1.01
#define V 0.1
#define dt 0.250

enum PinAssignments {
  encoderPinA = 2,
  encoderPinB = 3,
  clearButton = 8
};

volatile long encoderPos = 0;
long lastReportedPos = 1;

volatile unsigned int A_set = 0;
unsigned int A_old = 0;
volatile unsigned int B_set = 0;
unsigned int B_old = 0;

/********************************////
double x=0;
double x_t=0;
double l=0;
double r=0;
double y=0;
double y_t=0;
double theta = 0;
double theta_t=0;
double x_g=0;
double y_g=0;
double e_k = 0;
double e_k_1 = 0;
double u_x=0;
double u_y=0;
double theta_g=0;
double e_P=0;
double E_k=0;
double e_I=0;
double e_D=0;
int le=1;
int ri=1;
/**********************************/

void setup() {
  pinMode(lefta,OUTPUT);
  pinMode(leftb,OUTPUT);
  pinMode(righta,OUTPUT);
  pinMode(rightb,OUTPUT);
  pinMode(encoderPinA, INPUT); 
  pinMode(encoderPinB, INPUT); 
  pinMode(clearButton, INPUT);
  digitalWrite(encoderPinA, HIGH);  // turn on pullup resistor
  digitalWrite(encoderPinB, HIGH);  // turn on pullup resistor
  digitalWrite(clearButton, HIGH);

// encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
// encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);
  
  Serial.begin(57600);
 
}

void ahead(int time){
  //digitalWrite(lefta,HIGH);
  analogWrite(lefta,110);
  digitalWrite(leftb,LOW);
  //digitalWrite(righta,HIGH);
  analogWrite(righta,110);
  digitalWrite(rightb,LOW);
  le=1;  
  delay(time*1000);
}

void left(int time){
  //digitalWrite(lefta,HIGH);
  digitalWrite(lefta,LOW);
  analogWrite(leftb,200);
  //digitalWrite(righta,HIGH);
  analogWrite(righta,200);
  digitalWrite(rightb,LOW);
  le=-1;  
  delay(time*1000);
}

void loop(){ 
  left(0);
  if (A_set != 0) {
    Serial.print("r :");
    Serial.println(r);
    Serial.print("l :");
    Serial.println(l);
    Serial.print("x_t:");
    Serial.print(x_t, DEC);
    Serial.println() ;
    Serial.print("y_t:");
    Serial.println(y_t);
    Serial.print("x:");
    Serial.println(x);
    Serial.print("y:");
    Serial.println(y);
    Serial.print("theta_t :");
    Serial.println(theta_t);
    Serial.println(theta);
  }
   odmetry();
   A_old += A_set;
   A_set = 0;
   B_old += B_set;
   B_set = 0;
   
   if(theta>=3){
       Serial.println(x);
       Serial.println(y);
       analogWrite(leftb,0);
       analogWrite(righta,0);
       delay(100000);
   }
  delay(50);
}

// Interrupt on A changing state
void doEncoderA(){
  // Test transition
  if(digitalRead(encoderPinA)==HIGH){ //debouncing
    delay(1);
    A_set += digitalRead(encoderPinA) == HIGH?1:0;
  }
}

// Interrupt on B changing state
void doEncoderB(){
  if(digitalRead(encoderPinB)==HIGH){ //debouncing
    delay(1);
    B_set += digitalRead(encoderPinB) == HIGH?1:0;
  }
}

void odmetry(){
 
  l=A_set*mpr*le;
  r=B_set*mpr*ri;
  x_t=(l+r)/2*cos(theta);
  y_t=(l+r)/2*sin(theta);
  theta_t=(r-l)/L;
  
  x+=x_t;
  y+=y_t;
  theta+=theta_t*0.0174;//to radian

}

void pid(){
  double kp = 4;
  double ki = 0.01;
  double kd = 0.01;
 
 u_x=x_g-x;
 u_y=y_g-y;
 theta_g=atan2(u_y,u_x);
 
 e_k = theta_g-theta;
 e_k=atan2(sin(e_k),cos(e_k));
 
 e_P = e_k;
 
 e_I = E_k + e_k*dt;
 e_D = (e_k - e_k_1)/dt;
 
 //w=kp*e_p + ki*e_I + kd*e_D;
 E_k = e_I;
 e_k_1 = e_k;
 
  
}
