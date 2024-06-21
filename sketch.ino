void setup() {
  Serial3.begin(9600);
  Serial.begin(9600);
}

int leftpin1 =4;
int leftpin2 = 9;
int leftpin3 = 6;
int rightpin1 =7;
int rightpin2 = 8;
int rightpin3 = 5;

void go (int ml, int mr){
  Serial.print(ml);
  Serial.print(" ");
  Serial.println(mr);
  if (ml>0){
    digitalWrite(leftpin1, 1);
    digitalWrite(leftpin2, 0);
    analogWrite(leftpin3, ml);
    }
  else{
    digitalWrite(leftpin1, 0);
    digitalWrite(leftpin2, 1);
    analogWrite(leftpin3, 0-ml);
    };
  if (mr>0){

    digitalWrite(rightpin1, 0);
    digitalWrite(rightpin2, 1);
    analogWrite(rightpin3, mr);

    }
  else{
    digitalWrite(rightpin1, 1);
    digitalWrite(rightpin2, 0);
    analogWrite(rightpin3, 0-mr);
    }
  };
int counter = 0;
int y[2] = {0,0};
void loop() {

  while (Serial3.available()){   
    int x = Serial3.parseInt();
    if (x == 0) continue;
    if(x ==1000){
      counter = 0;
      }
    else if(counter == 0){
      y[0] = x;
      counter = 1;
    }
    else if  (counter == 1){
      counter = 0;
      y[1]=x;
      go(y[0], y[1]);
    };
  };
};
