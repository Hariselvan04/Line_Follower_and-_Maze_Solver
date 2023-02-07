#define l1 11
#define l2 10
#define r1 3
#define r2 2

#define enl 9
#define enr 6
#define led 13

int d;
char tur;
int swCount = 0;
int ma=0;
int ms=0;
char path[100];
int i = 0;
int j = 0;
int k =0;
int a[5];
int last_proportional = 0;
int integral = 0;
int spd = 200; 
int c = 0;
float distance;
int flag = 0;
char New[10]={};
char select_turn(unsigned char found_left, unsigned char found_right, unsigned char found_st);
int v; // Modulus Function
int set_motors(int a, int b);
void turn(char dir);
void PID();

int right = 0;
int left = 0;

int s5 = A0;
int s4 = A1;
int s3 = A2;
int s2 = A3;
int s1 = A4;

#define MotorPin1 2
#define MotorPin2 3
#define EnablePin 6
#define Motor2Pin1 10
#define Motor2Pin2 11
#define EnablePin2 9  


int pwmValue = 0, pwmValue2 = 0;
unsigned int s[6];
int detect = 1;
int notDetect = 0;

void setup() 
{
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, INPUT);
  pinMode(13,OUTPUT);
}

int readline() {
  a[0] = digitalRead(A4);
  a[1] = digitalRead(A3);
  a[2] = digitalRead(A2);
  a[3] = digitalRead(A1);
  a[4] = digitalRead(A0);
  int v;
  v = (4000*a[0] + 3000*a[1] + 2000*a[2] + 1000*a[3] + 0*a[4])/
      (a[0] + a[1] + a[2] + a[3] + a[4]);
  return v;
}

void turn(char dir) { //Turning setup
  switch(dir) {
    case 'L':
      set_motors(-spd, spd);
      delay(90);
      break;
    case 'R':
      set_motors(spd, -spd);
      delay(90);
      break;
    case 'B':
      set_motors(spd, -spd);
      delay(160);
      break;
    case 'S':
      break;
  }
}

int set_motors(int l, int r) 
{ // Motor setup
  if(l > 0 && r > 0)  {
    analogWrite(enl, l);
    analogWrite(enr, r);

    digitalWrite(l1, LOW);
    digitalWrite(l2, HIGH);
    
    digitalWrite(r1, LOW);
    digitalWrite(r2, HIGH);
  }

  else if(l < 0 && r > 0) {
    analogWrite(enl, l);
    analogWrite(enr, r);

    digitalWrite(l1, HIGH);
    digitalWrite(l2, LOW);
    
    digitalWrite(r1, LOW);
    digitalWrite(r2, HIGH);
  }

  else if(l > 0 && r < 0) { 
    analogWrite(enl, l);
    analogWrite(enr, r);

    digitalWrite(l1, LOW);
    digitalWrite(l2, HIGH);
    
    digitalWrite(r1, HIGH);
    digitalWrite(r2, LOW);
  }

  else if(l == 0 && r == 0) {
    analogWrite(enl, 0);
    analogWrite(enr, 0);

    digitalWrite(l1, HIGH);
    digitalWrite(l2, HIGH);
    
    digitalWrite(r1, HIGH);
    digitalWrite(r2, HIGH);
  }
}

void PID() {
  int i;       // Control function
  int power_difference = 0;
  float Kp, Ki, Kd;
  unsigned int Position;
  int derivative, proportional;
  while(1) {     
    Position = readline();
    //Serial.println(Position);
    proportional = ((int)Position - 2000);
    
    derivative = proportional - last_proportional;
    integral = integral+proportional;

    last_proportional = proportional;
    // use the tutorial to set initial values of Kp, Ki, and Kd
    Kp = 0.1; 
    Ki = 0;
    Kd = 0;

    power_difference = proportional*Kp + integral*Ki + derivative*Kd;
    const int max = spd/2 + 30;
    if(power_difference > max)
     power_difference = max;
    if(power_difference < -max)
     power_difference = (-1*max);

    if(power_difference < 0)  //left
     set_motors(max+power_difference, max);
    else  //right
     set_motors(max, max-power_difference);    

    readline();
    
    if(a[0] == HIGH && a[1]==HIGH && a[2] == HIGH && a[3] == HIGH && a[4] == HIGH)
      return;
    else if(a[0] == LOW || a[4] == LOW)
      return;     
  }
}

void loop() 
{
  d=digitalRead(8);
  if(d == HIGH)
  {
  swCount++;
  delay(1000);
  }
  if(swCount == 1)
  {
  readsensor();
  condition();
  }
  if(swCount == 2)
  {
    pathSolve(path);
    pathSolve(New);
    while(1){
    d=digitalRead(8);
    if(d==HIGH)
      break;
    
  }
  }
  
  if(swCount == 3)
  {
    readsensor();
    shortestPath();
  }
}
/*######################################################################################################################################
 ##################################################################################################*/
void straightSpeed()
{
    pwmValue=map(1000,0,1023,0,254);
    digitalWrite(MotorPin1,HIGH);
    digitalWrite(MotorPin2,LOW);
    analogWrite(EnablePin,pwmValue);
    pwmValue2=map(1020,0,1023,0,254);
    digitalWrite(Motor2Pin1,HIGH);
    digitalWrite(Motor2Pin2,LOW);
    analogWrite(EnablePin2,pwmValue2);
    PID();
    readsensor();
}

void staticLeft()
{
    
    pwmValue=map(455,0,1023,0,254);
    digitalWrite(MotorPin1,LOW);
    digitalWrite(MotorPin2,HIGH);
    analogWrite(EnablePin,pwmValue);
    pwmValue2=map(455,0,1023,0,254);
    digitalWrite(Motor2Pin1,HIGH);
    digitalWrite(Motor2Pin2,LOW);
    analogWrite(EnablePin2,pwmValue2);
    while(s[4]==detect)
    readsensor();
}

void staticRight()
{
    pwmValue=map(455,0,1023,0,254);
    digitalWrite(MotorPin1,HIGH);
    digitalWrite(MotorPin2,LOW);
    analogWrite(EnablePin,pwmValue);
    pwmValue2=map(455,0,1023,0,254);
    digitalWrite(Motor2Pin1,LOW);
    digitalWrite(Motor2Pin2,HIGH);
    analogWrite(EnablePin2,pwmValue2);
    while(s[2]==detect)
    readsensor();
    Serial.println("staticright");    
}

void softLeft()
{
    pwmValue=map(0,0,1023,0,254);
    digitalWrite(MotorPin1,LOW);
    digitalWrite(MotorPin2,LOW);
    analogWrite(EnablePin,pwmValue);
    pwmValue2=map(500,0,1023,0,254);
    digitalWrite(Motor2Pin1,HIGH);
    digitalWrite(Motor2Pin2,LOW);
    analogWrite(EnablePin2,pwmValue2);
    readsensor();
}

void softRight()
{
    pwmValue=map(500,0,1023,0,254);
    digitalWrite(MotorPin1,HIGH);
    digitalWrite(MotorPin2,LOW);
    analogWrite(EnablePin,pwmValue);
    pwmValue2=map(0,0,1023,0,254);
    digitalWrite(Motor2Pin1,LOW);
    digitalWrite(Motor2Pin2,LOW);
    analogWrite(EnablePin2,pwmValue2);
    readsensor();
}

void Stop()
{
    pwmValue=map(0,0,1023,0,254);
    digitalWrite(MotorPin1,LOW);
    digitalWrite(MotorPin2,LOW);
    analogWrite(EnablePin,pwmValue);
    pwmValue2=map(0,0,1023,0,254);
    digitalWrite(Motor2Pin1,LOW);
    digitalWrite(Motor2Pin2,LOW);
    analogWrite(EnablePin2,pwmValue2);
    flag = 1;  
    digitalWrite(led,HIGH);
    while(1){
    if(digitalRead(8) == HIGH)
    {
      swCount++;
      break;
    }}
    digitalWrite(led,LOW);
   
     
    
}

void lilmoveforward()
{
   pwmValue=map(400,0,1023,0,254);
   digitalWrite(MotorPin1,HIGH);
   digitalWrite(MotorPin2,LOW);
   analogWrite(EnablePin,pwmValue);
   pwmValue2=map(400,0,1023,0,254);
   digitalWrite(Motor2Pin1,HIGH);
   digitalWrite(Motor2Pin2,LOW);
   analogWrite(EnablePin2,pwmValue2);
   delay(300);
}
void U_turn()
{
    pwmValue=map(400,0,1023,0,254);
    digitalWrite(MotorPin1,HIGH);
    digitalWrite(MotorPin2,LOW);
    analogWrite(EnablePin,pwmValue);
    pwmValue2=map(400,0,1023,0,254);
    digitalWrite(Motor2Pin1,LOW);
    digitalWrite(Motor2Pin2,HIGH);
    analogWrite(EnablePin2,pwmValue2);
    readsensor();
}

void condition()
{
  if(s[1] == notDetect && s[2] == notDetect && s[3] == detect && s[4] == notDetect && s[5] == notDetect)
  {
    straightSpeed();
  }
  else if(s[1] == notDetect && s[2] == notDetect && s[3] == detect && s[4] == detect && s[5] == detect)
  {
    lilmoveforward();
    if(s[3] == detect)
    {
      straightSpeed();
      path[i] = 'S';
      while(ms==0){
      Serial.println("S");
     
      ms++;
      i++;
      }
       ma=0;
    }
    else
    {
      staticRight();
      delay(500);
    }
  }
  else if(s[1] == detect && s[2] == detect && s[3] == detect && s[4] == notDetect && s[5] == notDetect)
  {
    if(flag == 0)
    lilmoveforward();
    readsensor();
    if(s[1] == detect && s[2] == detect && s[3] == detect && s[4] == notDetect && s[5] == notDetect)
    {
      staticLeft();
      delay(100);
      Stop();
      path[i] = 'F';
      Serial.println("F");
    }
    else
    {
    staticLeft();
    delay(500);
    }
  }
   else if(s[1] == detect && s[2] == detect && s[3] == detect && s[4] == detect && s[5] == notDetect)
  {
    if(flag == 0)
    lilmoveforward();
    readsensor();
    if(s[1] == detect && s[2] == detect && s[3] == detect && s[4] == detect && s[5] == notDetect)
    {
      staticLeft();
      delay(100);
      Stop();
      path[i] = 'F';
      Serial.println("F");
    }
   
  }
  else if(s[1] == notDetect && s[2] == detect && s[3] == notDetect && s[4] == notDetect && s[5] == notDetect)
  {
    softLeft();
  }
  else if(s[1] == detect && s[2] == notDetect && s[3] == notDetect && s[4] == notDetect && s[5] == notDetect)
  {
    softLeft();
  }
  else if(s[1] == notDetect && s[2] == notDetect && s[3] == notDetect && s[4] == detect && s[5] == notDetect)
  {
    softRight();
  }
  else if(s[1] == notDetect && s[2] == notDetect && s[3] == notDetect && s[4] == notDetect && s[5] == detect)
  {
    softRight();
  }
  else if(s[1] == notDetect && s[2] == notDetect && s[3] == notDetect && s[4] == notDetect && s[5] == notDetect)
  {
    U_turn();
    while(ma==0){
    path[i] = 'U';
    Serial.println("U");
    ma++;
    i++;
    ms=0;
    }
  }
  else if(s[1] == detect && s[2] == notDetect && s[3] == detect && s[4] == notDetect && s[5] == notDetect)
  {
    staticLeft();
    delay(500);
  }
  else if(s[1] == detect && s[2] == detect && s[3] == detect && s[4] == detect && s[5] == detect)
  {
   if(flag == 0)
   lilmoveforward();
   readsensor();
   if(s[1] == notDetect && s[5] == notDetect)
   {
   staticLeft();
   path[i] = 'L';
   ma=0;
   ms=0;
   Serial.println("L");
   i++;
   }
   delay(500);
   if(s[1] == detect && s[2] == detect && s[3] == detect && s[4] == detect && s[5] == detect)
   {
    Stop();
    delay(500);
    path[i] = 'F';
    Serial.println("F");
    for(j =0; j<i; j++)
    {
      Serial.print(path[j]);
      Serial.print(" ");
    }

   }
  }
}

void readsensor()
{
  s[1] = digitalRead(s1);
  s[2] = digitalRead(s2);
  s[3] = digitalRead(s3);
  s[4] = digitalRead(s4);
  s[5] = digitalRead(s5);
  
//  Serial.println(s[1]);
//  Serial.print("   ");
//  Serial.print(s[2]);
//  Serial.print("   ");
//  Serial.print(s[3]);
//  Serial.print("   ");
//  Serial.print(s[4]);
//  Serial.print("   ");
//  Serial.println(s[5]);
}

/*####################################################################################################################################
 * ###################################################################################################################################
 * ###################################################################################################################################
 * ###################################################################################################################################*/

 void shortestPath()
 {
  Serial.println("short");
  if(s[1] == notDetect && s[2] == notDetect && s[3] == detect && s[4] == notDetect && s[5] == notDetect)
  {
    straightSpeed();
    Serial.println("straight");
  }
  else if(s[1] == notDetect && s[2] == notDetect && s[3] == detect && s[4] == detect && s[5] == detect)
  {
   lilmoveforward();
   readsensor();
   if(s[3] == detect)
   {
    Serial.println("path_decide");
    tur = New[k];
    k++;
    Serial.println(tur);
    shortPath(tur);
   }
   else
   {
    staticRight();
    Serial.println("right");
    delay(500);
    
   }
  }
  else if(s[1] == detect && s[2] == detect && s[3] == detect && s[4] == notDetect && s[5] == notDetect)
  {
   lilmoveforward();
   readsensor();
   if(s[3] == detect)
   {
    Serial.println("path_decide");
    tur = New[k];

    k++;
    Serial.println(tur);
    shortPath(tur);
   }
   else
   {
    staticLeft();
    delay(500);
   }
  }
  else if(s[1] == notDetect && s[2] == detect && s[3] == notDetect && s[4] == notDetect && s[5] == notDetect)
  {
    softLeft();
  }
  else if(s[1] == detect && s[2] == notDetect && s[3] == notDetect && s[4] == notDetect && s[5] == notDetect)
  {
    softLeft();
  }
  else if(s[1] == notDetect && s[2] == notDetect && s[3] == notDetect && s[4] == detect && s[5] == notDetect)
  {
    softRight();
  }
  else if(s[1] == notDetect && s[2] == notDetect && s[3] == notDetect && s[4] == notDetect && s[5] == detect)
  {
    softRight();
  }
  else if(s[1] == detect && s[2] == detect && s[3] == detect && s[4] == detect && s[5] == detect)
  {
   if(flag == 0)
   lilmoveforward();
   readsensor();
   if(s[1] == notDetect && s[5] == notDetect)
   {
    Serial.println("path_decide");
    tur = New[k];
    k++;
    Serial.println(tur);
    shortPath(tur);
   }
   delay(500);
   
  }
 }
 void shortPath(char tur)
 {
  if(tur == 'L')
  {
    pwmValue=map(455,0,1023,0,254);
    digitalWrite(MotorPin1,LOW);
    digitalWrite(MotorPin2,HIGH);
    analogWrite(EnablePin,pwmValue);
    pwmValue2=map(455,0,1023,0,254);
    digitalWrite(Motor2Pin1,HIGH);
    digitalWrite(Motor2Pin2,LOW);
    analogWrite(EnablePin2,pwmValue2);
    while(s[4]==detect)
    readsensor();
    Serial.println("staticLeft");
    delay(500);
    readsensor();
  }
  else if(tur == 'R')
  {
    pwmValue=map(455,0,1023,0,254);
    digitalWrite(MotorPin1,HIGH);
    digitalWrite(MotorPin2,LOW);
    analogWrite(EnablePin,pwmValue);
    pwmValue2=map(455,0,1023,0,254);
    digitalWrite(Motor2Pin1,LOW);
    digitalWrite(Motor2Pin2,HIGH);
    analogWrite(EnablePin2,pwmValue2);
    while(s[2]==detect)
    readsensor();
    Serial.println("staticright");
    delay(500);
    readsensor();
  }
  else if(tur == 'S')
  {
    straightSpeed();
    Serial.println("straight");
  }
  else if(tur == 'F')
  {
    Stop();
    Serial.println("stop");
  }
 }

 
void pathSolve(char arr[])
{
    int j=0;
    int Size= sizeof(arr)/sizeof(char);
    for(int i=0; i<Size;i++){
        if(arr[i+1]=='U'){
        if(arr[i]=='S' && arr[i+2]=='L'){
            New[j]='R';
            i=i+2;
            j++;
        }
        else if(arr[i]=='L' && arr[i+2]=='L'){
            New[j]='S';
             i=i+2;
            j++;
        }
         else if(arr[i]=='L' && arr[i+2]=='R'){
            New[j]='U';
             i=i+2;
            j++;
        }
        else if(arr[i]=='R' && arr[i+2]=='L'){
            New[j]='U';
             i=i+2;
            j++;
        }
        else if(arr[i]=='L' && arr[i+2]=='S'){
            New[j]='R';
             i=i+2;
            j++;
        }
        else if(arr[i]=='L' && arr[i+2]=='L'){
            New[j]='S';
             i=i+2;
            j++;
        }
        else if(arr[i]=='S' && arr[i+2]=='S'){
            New[j]='U';
             i=i+2;
            j++;
        }
        
        else if(arr[i]=='U'){
            if(New[j-1]=='S' && arr[i+1]=='L'){
                New[j-1]='R';
                j++;
                i+=1;
            }
         
        }
        }
        else{
            New[j]=arr[i];
           
            j++;
        }
       
    }
    Serial.println("SOLVED");
    delay(3000);
    for(int i=0;i<10;i++){
        Serial.println(New[i]);
    }
}
