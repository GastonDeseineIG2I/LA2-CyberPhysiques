#include "MeMegaPi.h"
#include <string.h>
#include <Wire.h>
#include <TimerFive.h>

#define BELT_PITCH 2.032 // distance between teeth
#define NTEETH 90.0      // number of teeth

#define RPM_2_MMS BELT_PITCH *NTEETH / 60.0 // conversion of the speed from rpm to mm/s
#define VOLTS_2_PWM 255.0 / 12.0            // conversion voltage to pwm [-255 255]
#define MAX_VOLTAGE 9                       // max allowed voltage

//-- -- -- -- SLAVE PARAMS

#define KX 0.01//0.0238
#define KI -0.3//-0.7534

//#define KX 0.0274
//#define KI -0.8647

#define TE 0.02

float xi1_error = 0; //Error m1
float xi2_error = 0; // Error m2
//-- -- -- --

volatile float angle = 0; //actual heading in deg
volatile float vm_1 = 0;
volatile float vm_2 = 0;
volatile int calc_time = 0;
volatile float speed1 = 0; //actual speed in mm/s
volatile float speed2 = 0;
volatile float ref1 = 0; //actual references from serial
volatile float ref2 = 0;
volatile int itr_measure = 0;
float u1 = 0; // control signals
float u2 = 0;
volatile long compTime = 0; // actual computation time of the critical loop
volatile short overrun = 0;

MeGyro gyro;                       // gyroscope object instanciation
MeEncoderOnBoard Encoder_1(SLOT1); // motor with encoder object instanciation
MeEncoderOnBoard Encoder_2(SLOT2);

/*
 * routine that is called every 5ms by the ISR routine
 */

void Update5ms()
{
  
  calc_time += 5;
  itr_measure += 5;
  UpdateSensors();
  if(itr_measure >= 20)
  {
    UpdateControl();
    itr_measure = 0;
  }
  UpdateActuators();
}

void UpdateSensors()
{
  gyro.update();            // update the gyroscope state
  angle = gyro.getAngleZ(); // get the estimated heading in deg

  if(itr_measure >= 20)
  {
    Encoder_1.loop(); // update the encoders state
    Encoder_2.loop();
  
    speed1 = Encoder_1.getCurrentSpeed() * RPM_2_MMS; // compute the speed in mm/s
    speed2 = Encoder_2.getCurrentSpeed() * RPM_2_MMS; 
  }
}

void UpdateControl()
{
  // Lecture de la vitesse du moteur pour lisser la tension

  //vm_1, vm_2 = 0;
  
//vm_1=0;
//vm_2=100;
  // Calcul des erreurs pour chaque moteur
  xi1_error = error_calc(xi1_error, -vm_1, speed1);
  xi2_error = error_calc(xi2_error, vm_2, speed2);

  // Calcul tension
  u1 = servo_system(speed1, xi1_error);
  u2 = servo_system(speed2, xi2_error);
  //u1 = 0;
  //u2 = 0;
  //___________________________________
  

  
}

void UpdateActuators()
{
  setMotorsVoltage(u1, u2); // set the voltages
}
/*
* routine that is called every 5ms by the timer 5
*/
void Timer5ISR()
{
  static char executing = 0; // set to 1 when the update function is running
  if (executing)
  {
    overrun = 1;
    return;
  }
  else
    executing = 1; // if already running => overrunned

  interrupts(); // enable the interrupts during the execution of this loop
  long startTime = micros();

  Update5ms();

  compTime = micros() - startTime;
  executing = 0;
}

/*
 * interruption routine for encoder pulse counting (quadrature)
 */

void isr_process_encoder1(void)
{
  if (digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();
    ;
  }
}

void isr_process_encoder2(void)
{
  if (digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}

/*
 * Setup the motors with the reduction ratio and number of pulses per turn
 */

void setupMotors()
{
  Encoder_1.setPulse(8);
  Encoder_1.setRatio(46);
  Encoder_2.setPulse(8);
  Encoder_2.setRatio(46);

  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

/*
 * Sets the voltage to the motor. Limited by MAX_VOLTAGE
 */
void setMotorsVoltage(float voltage1, float voltage2)
{
  Encoder_1.setMotorPwm(constrain(voltage1, -MAX_VOLTAGE, MAX_VOLTAGE) * VOLTS_2_PWM);
  Encoder_2.setMotorPwm(constrain(voltage2, -MAX_VOLTAGE, MAX_VOLTAGE) * VOLTS_2_PWM);
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial.setTimeout(1);
  Serial2.setTimeout(1);
  gyro.begin();
  Wire.setClock(400000);
  setupMotors();
  Timer5.initialize(5000);
  Timer5.attachInterrupt(Timer5ISR);
}

/**
*Bloc 3
*vc : Vitesse consigne
*err_int : Intégrale de l'erreur
**/
float servo_system(float vs, float err_int)
{
  return -KX * vs - KI * err_int;
}

/**
*
*xi_error : Erreur
*vc : Vitesse consigne
*vs : Vitesse sortie
**/
float error_calc(float xi_error,float vc, float vs)
{
  return xi_error + (vc - vs) * TE;
}

void loop()
{
  static float lastRef1 = 0;
  static float lastRef2 = 0;
  static float lastvm1 = 0;
  static float lastvm2 = 0;

  noInterrupts();
  ref1 = lastRef1;
  ref2 = lastRef2;
  vm_1 = lastvm1;
  vm_2 = lastvm2;
  float angleCopy = angle;
  float speed1Copy = speed1;
  float speed2Copy = speed2;
  long compTimeCopy = compTime;
  interrupts();

  // écriture RPI
  // On donne x,y,theta,vistesse actuelle

  // lecture RPI
  //xi1_error = error_calc(xi1_error, vm_1, 0);
  //xi2_error = error_calc(xi2_error, vm_2, 0);
  //float vcm_1 = 2; // Vitesse consigne moteur 1 lu de la RPI / calculé avec x,y,theta
  //float vcm_2 = 2; // Vitesse consigne moteur 2 lu de la RPI / calculé avec x,y,theta

  //float m1_u = servo_system(vcm_1, xi1_error);
  //float m2_u = servo_system(vcm_2, xi2_error);

  //Serial.print(" compTime: ");
  //Serial.print(compTimeCopy);

  Serial.print(" overrun: ");
  Serial.print(overrun);

  //Serial.print(" speed1: ");
 // Serial.print(speed1Copy);

 // Serial.print(" speed2: ");
 // Serial.print(speed2Copy);

  Serial.print(" angle: ");
  Serial.print(angleCopy);

  Serial.print(" u1: ");
  Serial.print(u1);

  
  Serial.print(" u2: ");
  Serial.print(u2);

Serial.print(" s2: ");
  Serial.print(speed2Copy);


//  Serial.print(" ref1: ");
 // Serial.print(ref1);

  //Serial.print(" ref2: ");
  //Serial.print(ref2);

  Serial.println();

  if (Serial.available())
  {
    lastRef1 = Serial.parseFloat();
    lastRef2 = Serial.parseFloat();
  }

  
  Serial2.println(angleCopy);
  
  if (Serial2.available())
    {
    String vmcStr = Serial2.readStringUntil('\n');
    Serial.println(vmcStr);
    char vmc[255];
    vmcStr.toCharArray(vmc, vmcStr.length());
    // split string to have values
    int i = 0;
    char *p = strtok(vmc, "/");
    char *array[3];

    while (p != NULL)
    {
      array[i++] = p;
      p = strtok(NULL, "/");
    }
  

    // Update FSM every 40ms
    if (calc_time >= 40)
    {
    

      // On attend que la RPI envoie les vcm
      //while (Serial2.available() <= 0)
      //{
      //  Serial.println(calc_time);
      //  delay(1);
      //}
    

      lastvm1 = atof(array[0]); // cast string
      lastvm2 = atof(array[1]);
    
      calc_time = 0;
     }
   }
  
  delay(10);
}
