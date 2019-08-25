#define DEBUG

#define sunrise_shift 60
#define sunset_shift 60

#include <Wire.h>
#include <Sodaq_DS3231.h>
#include <SerialCommand.h>
#include "new_sun.h"

#define PING_BOTTOM_PIN 30 // PWM Выход нижнего датчика;
#define PING_TOP_PIN 28	   // PWM Выход верхниего датчика;
#define CALIBRATION_TIME 5 // секунд
#define MEASURE_TIME 5 // мс
#define DELAY_STEP 3
#define NIGHT_BRIGHT 5 // Яркость подсвеченных ступенек в ночном режиме (первая, последняя);
#define MAX_BRIGHT 50 // Яркость включенных ступенек, максимально - 255;
#define STEP_BRIGHT 3 //шаг уменьшения яркости

#ifdef DEBUG
   #define switch_off_delay 5000ul // Задержка выключения посветки для отладки, милисекунды (здесь 5 секунд) ;
#else
   #define switch_off_delay 15000ul // Задержка выключения посветки, милисекунды (здесь 15 секунд) ;
#endif


#define PINS_COUNT 15
#define MAX_PIN PINS_COUNT - 1

enum {                                                                  // Основной цикл состоит из оператора switch, перечислены его шаги;
   SYNC_RTC,
   CALIBRATING_SENSOR,
   GET_BRIGHTNESS,
   SWITCH_OFF,
   WHAT_TO_DO
} programm_steps;

enum {
   SWITCHED_OFF,
   GO_OFF,
   SWITCHED_ON
} led_mode;

enum {
   NONE,
   BOTTOM,
   TOP
} ping;

const byte footstep_pins[] = {5, 4, 3, 2, 46, 44, 45, 13, 12, 11, 10, 9, 8, 7, 6};
byte led_state[PINS_COUNT];
boolean night = false;
byte bright;
int8_t start, stop, step;
unsigned long time_switched_on;

#define MAGIC_NUMBER 0xBB           //магическое число для определения девайса
#define TIME_SYNC_PERIOD 10000ul    //период ожидания синхронизации часов
unsigned long time_sync_start;
bool sync_rtc_mode = false;
char sync_rtc_command;              //команда синхронизации
SerialCommand cmd;

#ifdef DEBUG
void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}
#endif

void set_bright(byte pin, byte bright) {
   if (pin >= 0 && pin < PINS_COUNT && bright != led_state[pin]) {
      analogWrite(footstep_pins[pin], bright);
      led_state[pin] = bright;
   }  
}

byte get_min_bright(byte pin) {
   return ((pin == 0 || pin == MAX_PIN) && night) ? NIGHT_BRIGHT : 0;
}

void setup() {
   #ifdef DEBUG
      Serial.begin(115200);
      Serial.println("started");
   #endif
   Wire.begin();
   rtc.begin();

   #ifdef DEBUG
    DateTime now = rtc.now();
    print_current_time();
    Serial.println(is_night(now.date(), now.month(), 17, 1) ? "night" : "sunday");
   #endif
     
   night = false;
   
   for (byte i = 0; i < PINS_COUNT; i++) {
      set_bright(i, 0);
   }

  pinMode(PING_BOTTOM_PIN, INPUT);
  pinMode(PING_TOP_PIN, INPUT);  
      
  led_mode = SWITCHED_OFF;
  programm_steps = SYNC_RTC;
}

void check_night() {
      /*#ifdef DEBUG
        Serial.print("check night: ");
      #endif*/
   DateTime now = rtc.now();
   night = is_night(now.date(), now.month(), now.hour(), now.minute());
   #ifdef DEBUG
        Serial.print(now.hour());
        Serial.print(":");
        Serial.print(now.minute());
   #endif
   if (led_mode != SWITCHED_ON) {
	  set_bright(0, get_min_bright(0));
	  set_bright(MAX_PIN, get_min_bright(MAX_PIN));
   }
	 #ifdef DEBUG
	    if (night) {
	       Serial.println("night");
	    } else {
	       Serial.println("sunday");
	    }
	 #endif
}

void get_direction(boolean top) {
   if (top) {
      start = MAX_PIN;
      stop = -1;
      step = -1;   
   } else {
      start = 0;
      stop = PINS_COUNT;
      step = 1;
   }
}

void switch_on(boolean top) {
   get_direction(top);
   while (start != stop) {      
      bright = led_state[start];
      while (bright != MAX_BRIGHT) {
	 if (MAX_BRIGHT - bright < STEP_BRIGHT) {
	    bright = MAX_BRIGHT;
	 } else {
	    bright += STEP_BRIGHT;
	 }      
	 set_bright(start, bright);
	 delay(DELAY_STEP);
      }    
      start += step;
   }
}

boolean need_do_off() {
   return millis() - time_switched_on > switch_off_delay;
}

boolean check_pir(byte pir_port) {
  unsigned long start_measure = millis();
  unsigned long high_count = 0;
  unsigned long low_count = 0;
  while (millis() - start_measure < MEASURE_TIME) {
    if (digitalRead(pir_port) == HIGH) {
      high_count++;
    } else {
      low_count++;
    }
  }
  return high_count > low_count;
}

int old_ping;
void check_ping() {
   if (check_pir(PING_BOTTOM_PIN)) {
      #ifdef DEBUG
	       if (old_ping != ping) {
         Serial.println("bottom");
         old_ping = ping;
      }
      #endif
      ping = BOTTOM;
   } else if (check_pir(PING_TOP_PIN)) {
      #ifdef DEBUG
      if (old_ping != ping) {
         Serial.println("top");
         old_ping = ping;
      }
      #endif
      ping = TOP;
   } else {
    #ifdef DEBUG
    if (old_ping != ping) {
         Serial.println("none");
         old_ping = ping;
      }
      #endif
      ping = NONE;
   }
}

void print_current_time() {
  DateTime now = rtc.now();
  Serial.print("Current time: ");
  Serial.print(now.date());
  Serial.print(".");
  Serial.print(now.month());
  Serial.print(".");
  Serial.print(now.year());
  Serial.print(" ");
  Serial.print(now.hour());
  Serial.print(":");
  Serial.print(now.minute());
  Serial.print(":");
  Serial.print(now.second());
  Serial.println();
}

void unrecognized()
{
  Serial.println("What?"); 
}

void read_rtc()
{
  print_current_time();
}

bool isdigit(char *str)
{
  char *p = str;
  int i = 0;
  while (*p != 0)
  {    
    if (*p < '0' || *p > '9')
      return false;
    p++;
    i++;  
  }
  return i < 5;
}

void write_rtc()
{
   char *arg;
   int year;
   int month;
   int day;
   int hours;
   int minutes;
   int seconds;
   
  arg = cmd.next();  
  year = arg != NULL && isdigit(arg) ? atoi(arg) : -1;
  if (year < 2000 || year > 2100)  
    Serial.println("unrecognized year " + String(arg));
   
  arg = cmd.next();
  month = arg != NULL && isdigit(arg) ? atoi(arg) : -1;
  if (month < 1 || month > 12)  
    Serial.println("unrecognized month " + String(arg));

  arg = cmd.next();
  day = arg != NULL && isdigit(arg) ? atoi(arg) : -1;
  if (day < 1 || day > 31)  
    Serial.println("unrecognized day " + String(arg));  

  arg = cmd.next();
  hours = arg != NULL && isdigit(arg) ? atoi(arg) : -1;
  if (hours > 23)  
    Serial.println("unrecognized hours " + String(arg));

  arg = cmd.next();
  minutes = arg != NULL && isdigit(arg) ? atoi(arg) : -1;
  if (minutes > 59)  
    Serial.println("unrecognized minutes " + String(arg));
  
  arg = cmd.next();
  seconds = arg != NULL && isdigit(arg) ? atoi(arg) : 0;  
  if (seconds > 59)  
    seconds = 0;

  if (year < 0 || month < 0 || day < 0 || hours < 0 || minutes < 0)
    return;
  
  DateTime dt(year, month, day, hours, minutes, seconds, 0);
  rtc.setDateTime(dt);
}

void synchronize_rtc()
{
  Serial.println("Syncronization mode.");
  Serial.println("Commands:");
  Serial.println("  r - get RTC time");
  Serial.println("  w year month day hours minutes [seconds] - set RTC time");  
  cmd.addCommand("r",read_rtc);       // Turns LED on
  cmd.addCommand("w",write_rtc); 
  cmd.addDefaultHandler(unrecognized); 
  while (true) {
     cmd.readSerial();
  }
}

void loop() {
   switch (programm_steps) {
      case SYNC_RTC:
        time_sync_start = millis();
        Serial.println("Wait for RTC syncronization mode.");
        Serial.println("Put 'm' to enter.");
        while (millis() - time_sync_start < TIME_SYNC_PERIOD && !sync_rtc_mode) {
          if (Serial.available() > 0) {
            sync_rtc_command = Serial.read();
            switch (sync_rtc_command) {
              case 'm':                           // команда вернуть магическое число
                Serial.write(MAGIC_NUMBER);
                sync_rtc_mode = true;
                break;
              default:
                while(Serial.available() > 0) {
                  Serial.read();
                }
            }
          }
        }
        if (sync_rtc_mode)
          synchronize_rtc();          
        programm_steps = CALIBRATING_SENSOR;        
        break;
      case CALIBRATING_SENSOR:
          #ifdef DEBUG
            Serial.print("calibrating sensor ");
          #endif
          digitalWrite(PING_BOTTOM_PIN, LOW);
          digitalWrite(PING_TOP_PIN, LOW);
          for(int i = 0; i < CALIBRATION_TIME; i++){
            #ifdef DEBUG
              Serial.print(".");
            #endif
            delay(1000);
          }
          #ifdef DEBUG
            Serial.println(" done");
            Serial.println("SENSOR ACTIVE");
          #endif
          programm_steps = GET_BRIGHTNESS;
        break;
      case GET_BRIGHTNESS:
        check_night();
        programm_steps = WHAT_TO_DO;
        break;

      case WHAT_TO_DO:
	 check_ping();
	 if (night && ping > NONE) {
	    if (led_mode != SWITCHED_ON) {
	       switch_on(ping == TOP);
	       led_mode = SWITCHED_ON;
	    }
	    time_switched_on = millis();
	 } else if (led_mode == GO_OFF) {
	    programm_steps = SWITCH_OFF;
	 } else if (led_mode == SWITCHED_ON && need_do_off()) {
	    get_direction(ping == BOTTOM);
	    led_mode = GO_OFF;
	    programm_steps = SWITCH_OFF;
	 } else {
	    programm_steps = GET_BRIGHTNESS;
	 }
      break;
	 
      case SWITCH_OFF:
	 if (start != stop) {
	    bright = led_state[start];
	    if (bright - get_min_bright(start) < STEP_BRIGHT) {
	       bright = get_min_bright(start);
	    } else {
	       bright -= STEP_BRIGHT;
	    }
	    set_bright(start, bright);
	    delay(DELAY_STEP);
	    if (bright == get_min_bright(start)) {
	       start += step;
	    }
	 } else {
	    led_mode = SWITCHED_OFF;
	 }
	 programm_steps = WHAT_TO_DO; 
      break;
   }
}
