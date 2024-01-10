#include <IBusBM.h>
#include <math.h>

IBusBM ibus;

int rcCH1 = 0;
int rcCH2 = 0;
int rcCH3 = 0;
int rcCH4 = 0;
int rcCH5 = 0;
int rcCH6 = 0;


#define speedPort1 11
#define speedPort2 10
#define speedPort3 9
#define speedPort4 6
#define speedPort5 5

#define dirPort1 12
#define dirPort2 8
#define dirPort3 7
#define dirPort4 4
#define dirPort5 2



int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue)
{
    uint16_t ch = ibus.readChannel(channelInput);
    if (ch < 900)
        return defaultValue;
    return map(ch, 1000, 2000, minLimit, maxLimit);
}

class Motor
{
public:
    int speedPort, dirPort, abm = 0;
    Motor(int speedPortx, int dirPortx)
    {
        speedPort = speedPortx;
        dirPort = dirPortx;
    }

    void moveit(float m)
    {
        abm = abs(m);
        analogWrite(speedPort, abm);
        if (m < 0)
        {
            digitalWrite(dirPort, HIGH);
        }
        if (m > 0)
        {
            digitalWrite(dirPort, LOW);
        }
    }
};
Motor motor1(11, 12), motor2(10, 8), motor3(9, 7), motor4(6, 4), motor5(5, 2);



void setup()
{
    ibus.begin(Serial1);
    Serial.begin(9600);
    pinMode(dirPort1, OUTPUT);
    pinMode(speedPort1, OUTPUT);
    pinMode(dirPort2, OUTPUT);
    pinMode(speedPort2, OUTPUT);
    pinMode(dirPort3, OUTPUT);
    pinMode(speedPort3, OUTPUT);
    pinMode(dirPort4, OUTPUT);
    pinMode(speedPort4, OUTPUT);
    pinMode(dirPort5, OUTPUT);
    pinMode(speedPort5, OUTPUT);

    analogWrite(speedPort1, 0);
    analogWrite(speedPort2, 0);
    analogWrite(speedPort3, 0);
    analogWrite(speedPort4, 0);
    analogWrite(speedPort5, 0);
}

void loop()
{
    rcCH1 = readChannel(0, -1000, 1000, 0);
    rcCH2 = readChannel(1, -1000, 1000, 0);
    rcCH3 = readChannel(2, 0, 255, 0);
    rcCH4 = readChannel(3, -100, 100, 0); // angular velocity
    rcCH5 = readChannel(4, -255, 255, 0);
    rcCH6 = readChannel(5, -255, 255, 0);

    // ax = rcCH1;
    // ay = rcCH2;
    // speedf = rcCH3;
    // angular_velocity = rcCH4;
    // up_pwm = rcCH5;
    // out_pwm = rcCH6;

    int left_wing = rcCH1;
    int right_wing = rcCH2;
    int speedf = rcCH3;

    int left_val = 0 ;
    int right_val = 0;
    
    if(left_wing > 200){
        if(right_wing > 200){
            left_val = speedf;
            right_val = speedf;
            Serial.println("forward");
        }
        if(right_wing < -200){
            left_val = speedf;
            right_val = -speedf;
            Serial.println("right");

        }
    }

    if(left_wing < -200){
        if(right_wing > 200){
            left_val = -speedf;
            right_val = speedf;
            Serial.println("left");

        }
        if(right_wing < -200){
            left_val = -speedf;
            right_val = -speedf;
            Serial.println("back");
        }
    }
    
    Serial.println("----------------------------");
    Serial.println(left_wing);
    Serial.println(right_wing);
    Serial.println(speedf);
    Serial.println("----------------------------");

    run_left(left_val);
    run_right(right_val);
    delay(250);
}

void run_left(int m){
    motor1.moveit(m);
    motor3.moveit(m);
}

void run_right(int m){
    motor2.moveit(m);
    motor4.moveit(m);
    motor5.moveit(m);
}
