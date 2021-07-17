#include "eeprom.h"
#include "MIDI.h"
#include "wavegeneration.h"


bool pres_rate, pres_depth = 0;

bool mult_follow_midi = 0, mult_follow_pres = 0, mult_follow_pot = 1; //Multiplier flags
bool wave_follow_midi = 0, wave_follow_pres = 0, wave_follow_pot = 1; //Waveform flags
bool symm_follow_midi = 0, symm_follow_pres = 0, symm_follow_exp = 0, symm_follow_pot = 1; //Symmetry flags
bool depth_follow_midi = 0, depth_follow_pres = 0, depth_follow_exp = 0, depth_follow_pot = 1; //Depth flags
bool rate_follow_midi = 0, rate_follow_pres = 0, rate_follow_exp = 0, rate_follow_tap = 0, rate_follow_pot = 1; //Rate flags


struct preset {
  bool  mode;
  bool  wavebank;
  int   wave;
  int   n;
  int   rate;
  int   ramptime;
  float mult;
  float depth;
  float symmetry;
  int   depthramp;
  float randomizer;
};

struct secondary_flags {
  bool rate, depth, symmetry;
};
struct secondary_flags secondary = {0, 0, 0};

//Переменные для мигания
byte q;
bool state;
unsigned long tmr;

//depthramp vars
bool depthrampup;
float depthrampval;
unsigned long depthramptimer;
int depthramptime;
float randomizer_val;
float randomizer_depth;
int sh_rand;
int pattern_rand;
bool pattern_rand_midi;
byte midi_channel = 1;
byte preset_bank = 1;
bool exp_pedal = 0;
byte exp_symmetry, exp_rate, exp_depth;

struct preset preset1 = {0, 1, 3, 10, 160, 0, 1.00, 1.00, 0.50, 0, 0.00};
struct preset preset2 = {0, 1, 5, 0, 500, 0, 1.00, 1.00, 0.50, 0, 0.00};
struct preset preset3 = {0, 1, 3, 0, 100, 0, 4.00, 1.00, 0.50, 0, 0.00};
struct preset preset4 = {0, 0, 1, 10, 600, 0, 2.00, 1.00, 0.50, 0, 0.00};
struct preset preset5 = {1, 1, 5, 0, 1200, 300, 1.00, 1.00, 0.50, 300, 0.00};
struct preset preset6 = {0, 1, 2, 0, 250, 0, 1.00, 0.90, 0.25, 0, 0.00};
struct preset preset7 = {0, 1, 7, 0, 400, 500, 0.90, 1.00, 0.15, 0, 0.75};
struct preset preset8 = {1, 1, 5, 10, 300, 50, 1.00, 1.00, 0.65, 300, 0.10};
bool preset_action_ready;
byte is_presets_written = 0;

#include <MIDI.h>
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midi1);


#include <SimpleKalmanFilter.h>
SimpleKalmanFilter  firstKalmanFilter(12, 1, 0.001);
SimpleKalmanFilter  filtered_ratepot(5, 1, 0.001);
SimpleKalmanFilter  multiplier_kalman(10, 1, 0.1);


#include <Wire.h>
#include <extEEPROM.h>    //http://github.com/JChristensen/extEEPROM/tree/dev

extEEPROM myEEPROM(kbits_16, 1, 16, 0x50);

#include <ArduinoTapTempo.h>
ArduinoTapTempo tapTempo;

#include <Ramp.h>
rampLong myramp;
rampInt mode_move;

#include <Click.h>
click_t in_Click;
Click bpsw;

click_t tap_Click;
Click  tap_sw;

click_t preset1_Click;
Click  preset1_sw;

click_t preset2_Click;
Click  preset2_sw;

//--Добавим таблицы с волнами
#define resolution 999
#include "sinetable.h"
#include "triangletable.h"
#include "lumpstable.h"
#include "rampuptable.h"
#include "squaretable.h"
#include "patterns.h"

int waveval, waveval_inv, waveval2, waveval3, waveval4;
float wavevalf;
unsigned long timing[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int i, j, p, n, b;
float m, multiplier_value = 1.0; //мультипликатор темпа

const byte tap_pin(PA5); //кнопка тапа
const byte bpsw_pin(PB12); //кнопка байпаса
const byte bp_led_r(PA8); //красный светодиод байпаса
const byte bp_led_b(PA9); //синий светодиод байпаса
const byte tap_led_r(PA6); //красный светодиод байпаса
const byte tap_led_b(PA7); //синий светодиод байпаса

const byte relay_plus(PB4); //плюс реле
const byte relay_g(PB5); //минус реле

const byte rate_pot(PB1); //ручка скорости
const byte depth_pot(PA1); //ручка глубины
const byte symm_pot(PA0); //ручка симметрии
const byte wave_sw(PA2); //ручка волны
const byte mult_sw(PB0); //ручка делителя

#include "AnalogPin.h"
AnalogPin rate_raw(rate_pot);
AnalogPin depth_raw(depth_pot);
AnalogPin symm_raw(symm_pot);
AnalogPin wave_raw(wave_sw);
AnalogPin mult_raw(mult_sw);
AnalogPin exp_data(PA4);
AnalogPin mult_data(mult_sw);
AnalogPin wave_data(wave_sw);





int curr_led;
byte blinking;
static bool bpstate;
static bool mode, modelast;
int mode_ramp;
static bool lp_check, hold;
static bool lp_check_mode;
static bool tapflag;
static bool rateflag = 1;
static bool midi_rate;
bool ready_to_off;
bool mode_sw;

bool rampup;
bool rampdown;
bool rampflag;
bool waveformflag;

bool relaystate;
byte timer_zeroing;
unsigned int relay_timer, secondary_blink_timer;

int wave;
int range;
int debounce;
int rateF, rateS, rate, rateG;

int waveform_input, new_waveform, waveform;
int symmetry_input, new_symmetry;
float symmetry;
int depth_input, new_depth;
int exp_input, new_exp_input;
float depth;

int ratepot;
int newratepot;
int ratemapped;
int ratetapped;
int midi_tempo;
int expression_rate, expression_depth;
int tap;
int newtap;
float rampval = 1;
int ramptime;
int newsymmetry, oldsymmetry;
int multiply, oldmultiply, newmultiply;
bool wavebank = 1;

int k = 0;


////////
//-SETUP
void setup() {
  Serial.begin(9600);

  byte i2cStat = myEEPROM.begin(extEEPROM::twiClock400kHz);

  midi1.begin(MIDI_CHANNEL_OMNI);
  midi1.setHandleControlChange(handleControlChange);
  midi1.setHandleProgramChange(handleProgramChange);
  midi1.setHandleClock(handleClock);

  pinMode(PB8, PWM);
  pinMode(PB9, PWM);
  pinMode(tap_led_b, PWM);
  pinMode(tap_led_r, PWM);
  pinMode(bpsw_pin, INPUT_PULLUP);
  pinMode(tap_pin, INPUT_PULLUP);
  pinMode(relay_plus, OUTPUT);
  pinMode(relay_g, OUTPUT);
  pinMode(bp_led_r, OUTPUT);
  pinMode(bp_led_b, OUTPUT);
  pinMode(PA3, INPUT);

  pwmWrite(tap_led_b, 0);
  pwmWrite(tap_led_r, 0);
  digitalWrite(bp_led_b, 0);
  digitalWrite(bp_led_r, 0);

  midi_channel = myEEPROM.read(0);
  if (midi_channel == 0) {
    digitalWrite(bp_led_r, 1);
    digitalWrite(bp_led_b, 1);
    delay(2000);
    midi_channel = 1;
    myEEPROM.write(0, midi_channel);
  }

  is_presets_written = myEEPROM.read(1);
  if (is_presets_written != 54) {
    digitalWrite(bp_led_r, 1);
    digitalWrite(bp_led_b, 1);
    delay(2000);
    fill_presets();
  }

  HardwareTimer timer4 = HardwareTimer(4);
  timer4.setPrescaleFactor(1);
  timer4.setPeriod(15.555f); //50kHz
  timer4.setOverflow(1000);

  HardwareTimer timer3 = HardwareTimer(3);
  timer3.setPrescaleFactor(1);
  timer3.setPeriod(15.555f); //50kHz
  timer3.setOverflow(1000);

  HardwareTimer timer1 = HardwareTimer(1);
  timer1.setPrescaleFactor(1);
  timer1.setPeriod(15.555f); //50kHz
  timer1.setOverflow(1000);

  Timer2.pause(); // останавливаем таймер перед настройкой
  Timer2.setPeriod(100); // время в микросекундах (500мс)
  Timer2.attachInterrupt(TIMER_UPDATE_INTERRUPT, func_tim_2); // активируем прерывание
  Timer2.refresh(); // обнулить таймер
  Timer2.resume(); // запускаем таймер

  //delay(2500);
  startup_settings();

  if (exp_pedal == 0) {
    pinMode(PA4, INPUT_PULLUP);
    pinMode(PB10, INPUT_PULLUP);

    preset1_Click.timestamp_system       = millis();
    preset1_Click.state_hard             = !digitalRead(PA4);
    preset1_Click.duration_bounce        = 25;
    preset1_Click.duration_press         = 1500;

    preset2_Click.timestamp_system       = millis();
    preset2_Click.state_hard             = !digitalRead(PB10);
    preset2_Click.duration_bounce        = 25;
    preset2_Click.duration_press         = 1500;
  }

  if (exp_pedal == 1) {
    pinMode(PA4, INPUT);
    pinMode(PB10, OUTPUT);
    digitalWrite(PB10, HIGH);
  }

  digitalWrite(relay_plus, 0);
  digitalWrite(relay_g, 1);
  delay(2);
  digitalWrite(relay_plus, 0);
  digitalWrite(relay_g, 0);

  in_Click.timestamp_system       = millis();
  in_Click.state_hard             = !digitalRead(bpsw_pin);
  in_Click.duration_bounce        = 25;
  in_Click.duration_press         = 750;
  in_Click.duration_inactivity_Dn = 250;
  in_Click.duration_inactivity_Up = 50;
  in_Click.duration_click_Db      = 300;

  tap_Click.timestamp_system      =  millis();
  tap_Click.state_hard            = !digitalRead(tap_pin);
  tap_Click.duration_press        = 275;
  tap_Click.duration_bounce       = 25;
  tap_Click.duration_click_Db     = 1250;

  tapTempo.setTotalTapValues(2);
  tapTempo.setBeatsUntilChainReset(4);
  tapTempo.setMinBeatLengthMS(125);
  tapTempo.setMaxBeatLengthMS(1250);

  tapTempo.disableSkippedTapDetection();

  myramp.go(1023, 500, SINUSOIDAL_INOUT, ONCEFORWARD);
  mode_move.go(1000, 50, SINUSOIDAL_INOUT, ONCEFORWARD);

  Serial.println("exp_symmetry: ");
  Serial.println(exp_symmetry);

  Serial.println("exp_depth: ");
  Serial.println(exp_depth);

  Serial.println("exp_rate: ");
  Serial.println(exp_rate);

  Serial.println("exp_pedal: ");
  Serial.println(exp_pedal);
}

//////////////////////////////
//-Loop - здесь все происходит
void loop() {
  //Если меняется режим, делаем то, что нужно
  if (mode != modelast) {
    if (!mode && mode_move.isFinished() == 1) {
      mode_move.go(1000, 150, SINUSOIDAL_INOUT, ONCEFORWARD);
    }
    if (mode && mode_move.isFinished() == 1) {
      mode_move.go(0, 150, SINUSOIDAL_INOUT, ONCEFORWARD);
    }
  }
  modelast = mode; //И сохраняем последнее значение режима, для проверки
  midi1.read();
  footswitch_exp();
  ratecontrol();
  symmetry_set();
  bypass();
  relay_handler();
  depthcontrol();
  depthramp();
  pwmout();
  waveform_selector();
  mult_selector();
  ramping();
  timersetup();
  monitoring();
  led_handler();
  randomizer();
}

unsigned long exp_timer;
void footswitch_exp () {
  if (exp_pedal == 0) {
    preset1_Click.timestamp_system = millis();
    preset1_Click.state_hard = !digitalRead(PA4);
    preset1_sw.read(preset1_Click);

    preset2_Click.timestamp_system = millis();
    preset2_Click.state_hard = !digitalRead(PB10);
    preset2_sw.read(preset2_Click);

    //Первая кнопка
    if (preset1_sw.event_press_short() == 1) { //Вызов пресета
      preset_recall((2 * preset_bank) - 1);
    }
    if (preset1_sw.event_press_long() == 1) { //Сохранение пресета
      preset_save((2 * preset_bank) - 1);
    }

    //Вторая кнопка
    if (preset2_sw.event_press_short() == 1) { //Вызов пресета
      preset_recall(2 * preset_bank);
    }
    if (preset2_sw.event_press_long() == 1) { //Сохранение пресета
      preset_save(2 * preset_bank);
    }
  }
  if (exp_pedal == 1) {
    if (millis() - exp_timer >= 175) {
      exp_input = analogRead(PA4);
      exp_input = map(exp_input, 0 , 4095, 0, 127);
      exp_timer = millis();
    }

    if (exp_input - 20 >= new_exp_input || exp_input + 20 <= new_exp_input) {
      if (exp_rate != 0) {
        rate_follow_midi = rate_follow_pres = rate_follow_tap = rate_follow_pot = 0;
        rate_follow_exp = 1;
      }
      if (exp_depth != 0) {
        depth_follow_midi = depth_follow_pres = depth_follow_pot = 0;
        depth_follow_exp = 1;
      }
      if (exp_symmetry != 0) {
        symm_follow_midi = symm_follow_pres = symm_follow_pot = 0;
        symm_follow_exp = 1;
      }
    }
    new_exp_input = exp_input;
  }
}

void bypass () {
  in_Click.timestamp_system = millis();
  in_Click.state_hard = !digitalRead(bpsw_pin);
  bpsw.read(in_Click);

  if (!bpstate && bpsw.event_click_Dn() == 1) { //Одним щелчком включаем/выключаем реле
    if (depthramptime > 0) {
      depthrampval = 0;
      depthrampup = 1;
      depthramptimer = millis();
    }
    bpstate = !bpstate;
    ready_to_off = 0;                           //Обнуляем маркер возможности отключения эффекта коротким щелчком
    i = 0; //ретриггер
    j = 0;

  }

  if (bpsw.event_inactivity_Up() == 1) {        // Если произошло событие кнопки в ненажатом состоянии, ставим маркер возможности отключения эффекта коротким щелчком
    ready_to_off = 1;
  }

  if (bpstate && bpsw.event_press_short() == 1 && ready_to_off) {       //Короткий щелчок на отжатие выключает эффект
    bpstate = !bpstate;
    lp_check = 0;                                                       //Обнуляем маркер длинного нажатия
    ready_to_off = 0;                                                   //Обнуляем маркер возможности отключения эффекта
    mode_sw = 0;                                                        //Обнуляем маркер переключения режимов
  }

  if (bpsw.event_inactivity_Dn() == 1 && bpstate) {                     //Если эффект включен, засекаем lp и ставим флаг
    lp_check = 1;
  }

  if (lp_check && !mode_sw) {
    hold = 1;
  }
  if (bpsw.event_press_long() == 1 && bpstate && mode_sw) {             //Если эффект включен и есть маркер на переключение режима, засекаем длинное нажатие и переключаем режим
    lp_check = 0;
    mode = !mode;
  }
  if (bpsw.event_click_Up() == 1 && lp_check && bpstate && !mode_sw) {  //Если эффект включен, нет маркера на переключение режима и есть маркер длинного нажатия, отключаем эффект по отпусканию кнопки
    secondary = {0, 0, 0};
    hold = 0;
    bpstate = !bpstate;
    lp_check = 0; //сбрасываем маркер длинного нажатия
  }
  if (bpsw.event_click_Up() == 1 && !lp_check && bpstate) {             //Если кнопку отжали до срабатывания маркера длинного нажатия, включаем маркер на смену режима.
    mode_sw = 1;
  }

  if (preset_action_ready) {
    if (bpsw.event_click_Db() == 1) {
      preset_save(b);
      rampval = 1;
      rampflag = 0;
      preset_action_ready = 0;
    }
  }
}

void relay_handler () {
  const int relay_dly = 2750; //Время импульса на реле
  if (bpstate != relaystate) {
    if (timer_zeroing == 0) {
      relay_timer = micros();
      timer_zeroing++;
    }
    if (bpstate == 1) {
      if (micros() - relay_timer < relay_dly) {
        digitalWrite(relay_plus, 1);
        digitalWrite(relay_g, 0);
      }
      if (micros() - relay_timer >= relay_dly) {
        digitalWrite(relay_plus, 0);
        digitalWrite(relay_g, 0);
        relaystate = bpstate;
        timer_zeroing = 0;
      }
    }
    if (bpstate == 0) {
      if (micros() - relay_timer < relay_dly) {
        digitalWrite(relay_plus, 0);
        digitalWrite(relay_g, 1);
      }
      if (micros() - relay_timer >= relay_dly) {
        digitalWrite(relay_plus, 0);
        digitalWrite(relay_g, 0);
        relaystate = bpstate;
        timer_zeroing = 0;
      }
    }
  }
}

//////////////////////////////////////////
//-Прерывание для прохода по таблице волны
void func_tim_2() {
  i++;
  j++;
  p = ((j + 1) / 125);
  if (i > resolution)
  {
    i = 0;
  }
  if (i == 499 || i == 0) {
    sh_rand = random(1000);
  }
  if (j > 3999)
  {
    j = 0;
  }
  if (!pattern_rand_midi) {
    if (((j + 1) % 250) == 0) {
      pattern_rand = random(2);
      pattern_rand *= 1000;
    }
  }
}



///////////////////////
//-Регулировка скорости
unsigned long ratepot_timer;

void ratecontrol () {
  //Ручка скорости
  if (millis() - ratepot_timer >= 175) {
    ratepot = analogRead(rate_pot);
    ratepot = map(ratepot, 0, 4095, 0, 127);
    ratepot_timer = millis();
  }

  if (!rate_follow_pot) {
    if (ratepot - 18 >= newratepot || ratepot + 18 <= newratepot) //Сравниваем новое со старым, если отличается ставим флаг на ручку
    {
      rate_follow_pot  = 1;
      rate_follow_midi = rate_follow_tap = rate_follow_exp = rate_follow_pres = 0;
    }
  }

  if (hold) {
    if (secondary.rate == 0) {
      if (ratepot - 18 >= newratepot || ratepot + 18 <= newratepot) {
        Serial.println("secondary.rate:");
        Serial.println(secondary.rate);
        secondary_blink_timer = millis();
        blinking = 3;
        secondary.rate = 1;
      }
    }

    if (secondary.rate == 1) {
      if (ratepot <= 15) {
        randomizer_depth = 0;
      }
      if (ratepot > 15) {
        randomizer_depth = ratepot;
        randomizer_depth /= 127;
      }
    }
  }
  newratepot = ratepot;

  if (exp_pedal && exp_rate != 0) {
    if (exp_rate == 1) {
      expression_rate = map(exp_input, 0 , 127, 127, 0);
      float exp_curve = expression_rate / 127.0f;
      exp_curve *= exp_curve;
      expression_rate *= exp_curve;
      expression_rate = map(expression_rate, 0, 127, 100, 3000);
    }
    else if (exp_rate == 2) {
      expression_rate = exp_input;
      float exp_curve = expression_rate / 127.0f;
      exp_curve *= exp_curve;
      expression_rate *= exp_curve;
      expression_rate = map(expression_rate, 0, 127, 100, 3000);
    }
  }

  //Ручка скорости
  ratemapped = rate_raw.read(250); //Считываем значение ручки
  ratemapped = map(ratemapped, 0, 4095, 1023, 0); //переворачиваем значения с ручки
  float curve = ratemapped / 1023.0f; //делим
  curve *= curve; //и получаем квадрат
  ratemapped = ratemapped * curve; //умножаем квадрат на значения с ручки
  ratemapped = map(ratemapped, 0, 1023, 100, 3000); //масштабируем на максимальную и минимальную скорость

  //Тапка
  tap_Click.timestamp_system   =  millis();
  tap_Click.state_hard         = !digitalRead(tap_pin);
  tap_sw.read(tap_Click);

  tapTempo.update(tap_sw.event_click_Dn() == 1);
  ratetapped = tapTempo.getBeatLength();

  if (!rate_follow_tap && tap_sw.event_click_Db() == 1)  { //если стоит флаг на пот и цепь тапа активна,
    rate_follow_tap  = 1;
    rate_follow_midi = rate_follow_pot = rate_follow_exp = rate_follow_pres = 0;
  }
  if (rate_follow_pot && !hold) { //устанавливаем скорость в соответствии с флагом
    rate = ratemapped;
  }
  if (rate_follow_tap) {
    rate = ratetapped;
  }
  if (rate_follow_midi) {
    rate = midi_tempo;
  }
  if (rate_follow_exp) {
    rate = expression_rate;
  }
}


//////////////////////
//-Регулировка глубины
unsigned long depthpot_timer;
void depthcontrol () {
  if (millis() - depthpot_timer >= 175) {
    depth_input = analogRead(depth_pot);
    depth_input = map(depth_input, 0, 4095, 0, 127);
    depthpot_timer = millis();
  }

  if (!depth_follow_pot) {
    if (depth_input - 18 > new_depth || depth_input + 18 < new_depth) { //Сравниваем новое со старым, если отличается ставим флаг на ручку
      depth_follow_pot = 1;
      depth_follow_pres = depth_follow_midi = depth_follow_exp = 0;
    }
  }

  if (depth_follow_pot && !hold) {
    /*if (depth_input <= 3) {
      depth_input = 0;
      }
      else if (depth_input > 4) {
      depth_input = 127;
      }*/
    depth = depth_input / 127.00f;
    depth = sqrt(depth);
  }


  if (exp_pedal && exp_depth != 0) {
    if (depth_follow_exp) {
      int depth_exp_input = exp_input;
      if (exp_depth == 2) {
        depth_exp_input = map(depth_exp_input, 0, 127, 127, 0);
      }
      if (depth_exp_input < 2) {
        depth = 0.00f;
      }
      else if (depth_exp_input > 125) {
        depth = 1.00f;
      }
      else {
        depth = depth_exp_input / 127.0f;
        //depth = sqrt(depth);
      }
    }
  }

  // Скорость depthramp
  if (hold) {  //Если крутим пот в режиме холд, активируем настройку вторичного параметра по маркеру
    if (secondary.depth == 0) {
      if (depth_input - 18 > new_depth || depth_input + 18 < new_depth) {
        secondary.depth = 1;
        secondary_blink_timer = millis();
        blinking = 3;
      }
    }
  }
  if (secondary.depth == 1) {
    if (depth_input <= 15) {
      depthramptime = 0;
    }
    if (depth_input > 15) {
      depthramptime = map(depth_input, 0, 127, 2, 400);
    }
  }
  new_depth = depth_input;
}

void randomizer() {
  int t = myramp.update();
  if (myramp.isFinished() == 1) {
    int random_length = random(50, 3000);
    int random_depth = random(-1023, 1023);
    myramp.go(random_depth, random_length, SINUSOIDAL_INOUT, ONCEFORWARD);
  }
  randomizer_val = (t / 1023.0f) * randomizer_depth;
}

///////////////////////////////
//--Установка скорости в таймер
void timersetup() {

  //--Новый вариант симметрии
  rateG = rateround(rate, n); //Окргуление до значений бпм
  rateG /= (randomizer_val + 1);
  rateG /= m;
  rateF = (rateG * symmetry);
  rateS = rateG - rateF;
  rateF = (rateF * 2) / rampval; //*rampval удаляю пока не готово
  rateS = (rateS * 2) / rampval;

  if (wavebank) {
    if ( i < 499) {
      Timer2.setPeriod(rateF); //если первая половина волны, то устанавливаем время на нее
    }
    else {
      Timer2.setPeriod(rateS); //если вторая половина, то ставим время прерывания для нее
    }
  }
  if (!wavebank) {
    if ( j < 1999) {
      Timer2.setPeriod(rateF); //если первая половина волны, то устанавливаем время на нее
    }
    else {
      Timer2.setPeriod(rateS); //если вторая половина, то ставим время прерывания для нее
    }
  }
}

/////
//--Округление скорости до значений бпм
int rateround (int rate, int n) {
  if (n != 0) {
    int bpm = 60000 / rate;
    int bpmround = ((bpm + (n / 2)) / n) * n;
    rate = round (60000.0 / (float)bpmround);
    return rate;
  }
  else {
    return rate;
  }
}

unsigned long wavetimer;
//////////////////////
//--Ручка выбора волны
void waveform_selector () {
  if (millis() - wavetimer >= 250) {
    waveform_input = analogRead(wave_sw);
    waveform_input = map(waveform_input, 0, 4097, 1, 9);
    wavetimer = millis();
  }

  if (hold) {
    if (waveform_input != new_waveform && k == 0) {
      wavebank = !wavebank;
      k++;
    }
  }
  if (!wave_follow_pot) {
    if (waveform_input != new_waveform) {
      wave_follow_pot = 1;
      wave_follow_pres = wave_follow_midi = 0;
    }
  }

  if (waveform_input != new_waveform && !rampflag) {
    pattern_rand_midi = 0;
    secondary_blink_timer = millis();
    blinking = 4;
  }

  if (rampflag) {
    if (waveform_input != new_waveform) {
      secondary_blink_timer = millis();
      blinking = 5;
      switch (waveform_input) {
        case 1:
          preset_bank = 1;
          break;
        case 2:
          preset_bank = 2;
          break;
        case 3:
          preset_bank = 3;
          break;
        case 4:
          preset_bank = 4;
          break;
        case 5:
          preset_bank = 5;
          break;
        case 6:
          preset_bank = 6;
          break;
        case 7:
          preset_bank = 7;
          break;
        case 8:
          preset_bank = 8;
          break;
      }
    }
  }

  new_waveform = waveform_input;

  if (wave_follow_pot && !rampflag) {
    waveform = waveform_input;
  }
  if (!lp_check) {
    k = 0;
  }
}

unsigned long multtimer;
//////////////////////
//--Ручка выбора множителя
void mult_selector () {
  if (millis() - multtimer >= 250) {
    multiply = analogRead(mult_sw);
    multiply = map(multiply, 0, 4097, 1, 9);
    multtimer = millis();
  }

  if (!mult_follow_pot) {
    if (multiply != newmultiply) {
      mult_follow_pot = 1;
      mult_follow_pres = mult_follow_midi = 0;
    }
  }

  if (multiply != newmultiply && !hold && !rampflag) {
    secondary_blink_timer = millis();
    blinking = 4;
  }

  switch (multiply) {
    case 1:
      multiplier_value = 0.25; //четверть
      break;
    case 2:
      multiplier_value = 0.5; //половина
      break;
    case 3:
      multiplier_value = 1;  //целая
      break;
    case 4:
      multiplier_value = 1.5; //триплет
      break;
    case 5:
      multiplier_value = 2; //двойная
      break;
    case 6:
      multiplier_value = 3; //двойной триплет
      break;
    case 7:
      multiplier_value = 4; ////
      break;
    case 8:
      multiplier_value = 6;  ////
      break;
  }

  if (!rampflag && !hold && mult_follow_pot) {
    m = multiplier_value;
  }

  if (hold) {
    if (multiply != newmultiply) {
      secondary_blink_timer = millis();
      blinking = 3;
      switch (multiply) {
        case 2:
          n = 1;
          break;
        case 3:
          n = 2;
          break;
        case 4:
          n = 5;
          break;
        case 5:
          n = 10;
          break;
        case 6:
          n = 20;
          break;
        default:
          n = 0;
          break;
      }
    }
  }

  if (rampflag) {
    if (multiply != newmultiply) {
      secondary_blink_timer = millis();
      blinking = 5;
      preset_action_ready = 1;
      switch (multiply) {
        case 1:
          b = 1;
          break;
        case 2:
          b = 2;
          break;
        case 3:
          b = 3;
          break;
        case 4:
          b = 4;
          break;
        case 5:
          b = 5;
          break;
        case 6:
          b = 6;
          break;
        case 7:
          b = 7;
          break;
        case 8:
          b = 8;
          break;
      }
    }
  }
  newmultiply = multiply;
}

////////////////////////
//-Регулировка симметрии
unsigned long symmpot_timer;
int symmetry_input_detent;

void symmetry_set() {
  if (millis() - symmpot_timer >= 175) {
    symmetry_input = analogRead(symm_pot);
    symmetry_input = map(symmetry_input, 0, 4095, 0, 127);
    symmpot_timer = millis();
  }

  if (!symm_follow_pot) {
    if (symmetry_input - 18 > new_symmetry || symmetry_input + 18 < new_symmetry) { //Сравниваем новое со старым, если отличается ставим флаг на ручку
      symm_follow_pot = 1;
      symm_follow_pres = symm_follow_midi = 0;
    }
  }
  // Скорость ramp
  if (hold) {
    if (secondary.symmetry == 0) {
      if (symmetry_input - 18 > new_symmetry || symmetry_input + 18 < new_symmetry) { //Сравниваем новое со старым, если отличается ставим флаг на ручку
        secondary_blink_timer = millis();
        blinking = 3;
        secondary.symmetry = 1;
      }
    }
  }

  if (exp_pedal && exp_symmetry != 0) {
    if (symm_follow_exp) {
      int symmetry_mapped = exp_input;
      if (exp_symmetry == 2) {
        symmetry_mapped = map(symmetry_mapped, 0, 127, 127, 0);
      }
      symmetry_mapped = map(symmetry_mapped, 127, 0 , (127 - 15), 15);
      symmetry = symmetry_mapped / 127.0f;
    }
  }

  if (secondary.symmetry == 1) {
    ramptime = symmetry_input;
    ramptime = map(ramptime, 0, 127, 0, 350);
  }

  new_symmetry = symmetry_input;

  symmetry_input_detent = symm_raw.read(200);
  symmetry_input_detent = map(symmetry_input_detent, 0, 4095, 0, 127);
  symmetry_input_detent += 1;
  if (symmetry_input_detent < 68 && symmetry_input_detent > 60) {
    symmetry_input_detent = 64;
  }
  else {
    symmetry_input_detent  = map(symmetry_input_detent, 128, 1, (128 - 15), 15);
  }
  if (symm_follow_pot && !hold) {
    symmetry = symmetry_input_detent / 128.00f;
  }
}


///////
//--RAMP
void ramping () {
  if (tap_sw.event_press_long() == 1 && tap_sw.state_button() == 1) {
    tapTempo.resetTapChain();
    if (!rampup) {
      timing[3] = millis();
    }
    rampup = 1;
    rampdown = 0;
    Serial.println("rampup1");
    Serial.println(rampup);
    Serial.println(rampdown);
    rampflag = 1;
    k = 0;
  }
  if (tap_sw.state_button() == 0 && rampval != 1 && !rampdown) {
    rampdown = 1;
    rampup = 0;
    Serial.println("rampdown2");
    Serial.println(rampup);
    Serial.println(rampdown);
    rampflag = 0;
    if (preset_action_ready) {
      preset_recall(b);
      rampval = 1;
      preset_action_ready = 0;
    }
  }
  if (rampup) {
    if (rampval < 2) {
      if (millis() - timing[3] > ramptime) {
        Serial.println("+++");
        rampval += 0.05;
        timing[3] = millis();
      }
    }
    if (rampval >= 2) {
      rampup = 0;
    }
  }
  if (rampdown) {
    if (rampval <= 1) {
      rampval = 1;
      rampdown = 0;
    }
    if (millis() - timing[4] > ramptime) {
      Serial.println("---");
      rampval -= 0.05;
      timing[4] = millis();
    }
  }
}

void depthramp () {
  if (depthrampup) {
    if (depthrampval < 1) {
      if (millis() - depthramptimer >= depthramptime) {
        depthrampval += 0.05;
        depthramptimer = millis();
      }
    }
    if (depthrampval >= 1) {
      depthrampup = 0;
    }
  }
}


void preset_recall (int number) {
  int x = (number * 46) + 46;
  blinking = 1;


  preset1.mode = ee_float_read(0 + x);
  preset1.wavebank = ee_float_read(4 + x);
  preset1.wave = ee_float_read(8 + x);
  preset1.n = ee_float_read(12 + x);
  preset1.rate = ee_float_read(16 + x);
  if (preset1.rate <= 0) {
    preset1.rate = 500;
  }
  preset1.ramptime = ee_float_read(20 + x);
  preset1.mult = ee_float_read(24 + x);
  preset1.depth = ee_float_read(28 + x);
  preset1.symmetry = ee_float_read(32 + x);
  if (preset1.symmetry <= 0.02 || preset1.symmetry >= 0.98) {
    preset1.symmetry = 0.5;
  }
  preset1.depthramp = ee_float_read(36 + x);
  preset1.randomizer = ee_float_read(40 + x);

  mode = preset1.mode;
  wavebank = preset1.wavebank;
  n = preset1.n;
  ramptime = preset1.ramptime;
  depthramptime = preset1.depthramp;
  randomizer_depth = preset1.randomizer;

  //Rate
  rate = preset1.rate;
  rate_follow_pres = 1;
  rate_follow_pot = rate_follow_tap = rate_follow_exp = rate_follow_midi = 0;

  //Multiplier
  m = preset1.mult;
  mult_follow_pres = 1;
  mult_follow_pot = mult_follow_midi = 0;

  //Waveform
  waveform = preset1.wave;
  wave_follow_pres = 1;
  wave_follow_pot = wave_follow_midi = 0;

  //Symmetry
  symmetry = preset1.symmetry;
  symm_follow_pres = 1;
  symm_follow_pot = symm_follow_midi = symm_follow_exp = 0;

  //Depth
  depth = preset1.depth;
  depth_follow_pres = 1;
  depth_follow_pot = depth_follow_midi = depth_follow_exp = 0;
  depthrampval = 1.0;

  Serial.println ("");
  Serial.print ("preset number: ");
  Serial.println (number);
  Serial.print ("{");
  Serial.print (preset1.mode);
  Serial.print (", ");
  Serial.print (preset1.wavebank);
  Serial.print (", ");
  Serial.print (preset1.wave);
  Serial.print (", ");
  Serial.print (preset1.n);
  Serial.print (", ");
  Serial.print (preset1.rate);
  Serial.print (", ");
  Serial.print (preset1.ramptime);
  Serial.print (", ");
  Serial.print (preset1.mult);
  Serial.print (", ");
  Serial.print (preset1.depth);
  Serial.print (", ");
  Serial.print (preset1.symmetry);
  Serial.print (", ");
  Serial.print (preset1.depthramp);
  Serial.print (", ");
  Serial.print (preset1.randomizer);
  Serial.println ("};");
}

void preset_save (int number) {
  int x = (number * 46) + 46;
  blinking = 2;

  ee_float_write((0 + x), mode);
  ee_float_write((4 + x), wavebank);
  ee_float_write((8 + x), waveform);
  ee_float_write((12 + x), n);
  ee_float_write((16 + x), rate);
  ee_float_write((20 + x), ramptime);
  ee_float_write((24 + x), m);
  ee_float_write((28 + x), depth);
  ee_float_write((32 + x), symmetry);
  ee_float_write((36 + x), depthramptime);
  ee_float_write((40 + x), randomizer_depth);

  //На всякий случай
  mult_follow_pres = 1;
  mult_follow_pot = mult_follow_midi = 0;
}

void led_handler () {
  //Включаем светодиоды в соответствии с режимом
  if (blinking != 1) {
    if (!mode) {
      digitalWrite(bp_led_r, bpstate);
      digitalWrite(bp_led_b, 0);
    }
    if (mode) {
      digitalWrite(bp_led_b, bpstate);
      digitalWrite(bp_led_r, bpstate);
    }
  }
  if (blinking == 1) {
    digitalWrite(bp_led_r, 0);
    if (millis() - tmr > 100) {
      q++;
      if (q == 5) {
        blinking = 0;
        q = 0;
        state = 0;
      }
      digitalWrite(bp_led_b, state);
      state = !state;
      tmr = millis();
    }
  }

  if (blinking == 3) {
    digitalWrite(bp_led_r, 0);
    digitalWrite(bp_led_b, 0);
    if (millis() - secondary_blink_timer > 35) {
      blinking = 0;
    }
  }

  if (blinking == 4) {
    if (bpstate) {
      if (mode) {
        digitalWrite(bp_led_r, 1);
        digitalWrite(bp_led_b, 0);
      }
      if (!mode) {
        digitalWrite(bp_led_r, 1);
        digitalWrite(bp_led_b, 1);
      }
      if (millis() - secondary_blink_timer > 75) {
        blinking = 0;
      }
    }
    if (!bpstate) {
      if (mode) {
        digitalWrite(bp_led_r, 1);
        digitalWrite(bp_led_b, 0);
      }
      if (!mode) {
        digitalWrite(bp_led_r, 1);
        digitalWrite(bp_led_b, 1);
      }
      if (millis() - secondary_blink_timer > 25) {
        blinking = 0;
      }
    }
  }

  if (blinking == 5) {

    digitalWrite(bp_led_r, 0);
    digitalWrite(bp_led_b, 1);

    if (millis() - secondary_blink_timer > 75) {
      blinking = 0;
    }
  }

  //Светодиод тапа
  if (blinking == 2) {
    if (millis() - tmr > 150) {
      q++;
      if (q == 4) {
        blinking = 0;
        q = 0;
        state = 0;
      }
      pwmWrite(tap_led_r, 0);
      pwmWrite(tap_led_b, (state * 1000));
      state = !state;
      tmr = millis();
    }
  }
  if (blinking != 2) {
    if (wavebank) {
      pwmWrite(tap_led_r, waveval);
      pwmWrite(tap_led_b, 0);
    }
    if (!wavebank) {
      pwmWrite(tap_led_b, waveval);
      pwmWrite(tap_led_r, waveval);
    }
  }
}

void fill_presets () {
  for (i = 1; i <= 16; i++) {
    switch (i) {
      case 2:
        {
          preset1 = preset2;
          break;
        }
      case 3:
        {
          preset1 = preset3;
          break;
        }
      case 4:
        {
          preset1 = preset4;
          break;
        }
      case 5:
        {
          preset1 = preset5;
          break;
        }
      case 6:
        {
          preset1 = preset6;
          break;
        }
      case 7:
        {
          preset1 = preset7;
          break;
        }
      case 8:
        {
          preset1 = preset8;
          break;
        }
    }

    int x = (i * 46) + 46;

    ee_float_write((0 + x), preset1.mode);
    ee_float_write((4 + x), preset1.wavebank);
    ee_float_write((8 + x), preset1.wave);
    ee_float_write((12 + x), preset1.n);
    ee_float_write((16 + x), preset1.rate);
    ee_float_write((20 + x), preset1.ramptime);
    ee_float_write((24 + x), preset1.mult);
    ee_float_write((28 + x), preset1.depth);
    ee_float_write((32 + x), preset1.symmetry);
    ee_float_write((36 + x), preset1.depthramp);
    ee_float_write((40 + x), preset1.randomizer);
  }
  is_presets_written = 54;
  myEEPROM.write(1, is_presets_written);
}

void startup_settings () {
  exp_pedal = myEEPROM.read(3);
  exp_symmetry = myEEPROM.read(4);
  exp_depth = myEEPROM.read(5);
  exp_rate = myEEPROM.read(6);

  bool tap_hold = 0, bp_hold = 0;

  bool first_tap_read = !digitalRead(tap_pin);
  bool first_bp_read = !digitalRead(bpsw_pin);
  Serial.println("first_tap_read: ");
  Serial.println(first_tap_read);
  Serial.println("first_bp_read: ");
  Serial.println(first_bp_read);
  delay(20);
  bool second_tap_read = !digitalRead(tap_pin);
  bool second_bp_read = !digitalRead(bpsw_pin);
  Serial.println("second_tap_read: ");
  Serial.println(second_tap_read);
  Serial.println("second_bp_read: ");
  Serial.println(second_bp_read);
  delay(20);
  bool third_tap_read = !digitalRead(tap_pin);
  bool third_bp_read = !digitalRead(bpsw_pin);
  Serial.println("third_tap_read: ");
  Serial.println(third_tap_read);
  Serial.println("third_bp_read: ");
  Serial.println(third_bp_read);

  if (first_tap_read && second_tap_read && third_tap_read) {
    tap_hold = 1;
  }
  else {
    tap_hold = 0;
  }
  if (first_bp_read && second_bp_read && third_bp_read) {
    bp_hold = 1;
  }
  else {
    bp_hold = 0;
  }

  Serial.println("tap_hold: ");
  Serial.println(tap_hold);
  Serial.println("bp_hold: ");
  Serial.println(bp_hold);

  if (tap_hold && !bp_hold) {
    exp_pedal = 0;
    myEEPROM.write(3, exp_pedal);
    pwmWrite(tap_led_b, 1000);
    delay(175);
    pwmWrite(tap_led_b, 0);
    delay(175);
    pwmWrite(tap_led_b, 1000);
    delay(175);
    pwmWrite(tap_led_b, 0);
  }

  if (bp_hold && !tap_hold) {
    exp_pedal = 1;
    myEEPROM.write(3, exp_pedal);

    //Читаем симметрию
    int symm_read = analogRead(symm_pot);
    if (symm_read > 3000) { //1 для обычного
      exp_symmetry = 1;
    }
    else if (symm_read < 1000) { //2 для инвертированного
      exp_symmetry = 2;
    }
    else {
      exp_symmetry = 0; //0 отключен
    }

    //Читаем глубину
    int depth_read = analogRead(depth_pot);
    if (depth_read > 3000) { //1 для обычного
      exp_depth = 1;
    }
    else if (depth_read < 1000) { //2 для инвертированного
      exp_depth = 2;
    }
    else {
      exp_depth = 0; //0 отключен
    }

    //Читаем скорость
    int rate_read = analogRead(rate_pot);
    if (rate_read > 3000) { //1 для обычного
      exp_rate = 1;
    }
    else if (rate_read < 1000) { //2 для инвертированного
      exp_rate = 2;
    }
    else {
      exp_rate = 0; //0 отключен
    }

    if (exp_symmetry > 2) {
      exp_symmetry = 0;
    }
    if (exp_depth > 2) {
      exp_depth = 0;
    }
    if (exp_rate > 2) {
      exp_rate = 0;
    }

    myEEPROM.write(4, exp_symmetry);
    myEEPROM.write(5, exp_depth);
    myEEPROM.write(6, exp_rate);

    digitalWrite(bp_led_b, HIGH);
    delay(175);
    digitalWrite(bp_led_b, LOW);
    delay(175);
    digitalWrite(bp_led_b, HIGH);
    delay(175);
    digitalWrite(bp_led_b, LOW);
  }
}