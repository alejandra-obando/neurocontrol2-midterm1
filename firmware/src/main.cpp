// ======================================================================
// MAIN_DEF -- DEFINITIVE ESP32 FIRMWARE (locomotion slave)
// ----------------------------------------------------------------------
// Where each piece comes from (none of this was invented here):
//
//   carro_v08/firmware/src/main.cpp ...... ALL THE CONTROL. Per-wheel
//       speed loop in rpm with feed-forward, learned and saved take-off
//       PWM, runaway protection, PD heading loop, mecanum kinematics and
//       the inverted polarity of the 4 motors. They are the only numbers
//       MEASURED WHILE DRIVING ON BATTERY in the whole project (see
//       notas/2026-09-02_carro_calibracion_resultado.md).
//   robot_esp32_modular.cpp .............. block structure, moving
//       average filter of the ToF, neurocontroller, CSV to the Pi,
//       stall watchdog, open field.
//   pruebas_campo/prueba_comunicacion_giro/ IMU-closed turn with
//       correction passes, and radio silence while actuating.
//   pruebas_campo/prueba_solo_motores/ ... physical confirmation of the
//       pinout, of SIGNO_ADELANTE and of the rotation candidates.
//   Robot/pi/rl.py ....................... the 'R'/'L'/'T' contract and
//       event 7, which that file already declared as pending.
//
// WHAT WAS REMOVED, AND WHY:
//   - The PID in rad/s with a theoretical KV and PWM_MUERTO=30 (old BLOCK
//     12). Those numbers were never measured; the carro_v08 ones were.
//   - The open-loop PWM turn. Now the turn goes through the same speed
//     loop as the forward motion, which in carro_v08 gave 724 degrees
//     turned out of 721 requested over two full revolutions.
//   - The raw-PWM manual mode (Sep 14). It sent the SAME PWM to the four
//     wheels, and DD is a different motor (189 rpm at PWM 160 against ~90
//     for the others): with the same PWM it went twice as fast and the
//     robot curved or turned instead of going straight. The manual control
//     now goes through the same speed loop as navigation, as in carro_v08,
//     and it also accepts the lateral translation (vy) of the full mecanum
//     kinematics.
//
// SINGLE-OWNER RULE FOR THE MOTORS (prevents control clashes):
// ONE controller rules at a time. EVERYTHING goes through
// corregirVelocidad(), and what changes between states is only WHO writes
// ordenVx/ordenVy/ordenW: the locomotion machine (advance, turn) or the Pi
// by hand ('#m='). The direction calibration is the only one that touches
// the motors from outside (moverCrudo), and only with the robot IDLE.
//
// HARDWARE NOTE (2026-09-12, confirmed with the car's I2C scanner):
//   IMU = BNO085 (GY-BNO08X module) at 0x4B, on the direct bus. The SAME as
//   carro_v08. The Sep 8 version assumed an MPU6050 at 0x68 and there is
//   none: it booted with imu=0 and every turn returned EV_ERR_GIRO. The
//   BNO085 delivers the heading already fused (BLOCK 14), nothing is
//   integrated here.
//   Mux: encoders on the carro_v08 channels (0,1,6,7) and the four ToF on
//   the remaining free ones (2,3,4,5). See BLOCK 03: that ToF assignment is
//   the FIRST thing to verify on the robot.
//
// Naming: identifiers are in Spanish. Directions N/E/S/O = North/East/
// South/West (O = Oeste). Wheels DI/DD/TI/TD = front-left/front-right/
// rear-left/rear-right.
// ======================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>
// The BNO085 is not read by registers: it speaks SH-2 over SHTP and does the
// fusion internally. That is a library, not four writes (see platformio.ini).
#include <SparkFun_BNO080_Arduino_Library.h>
#include "Adafruit_VL53L0X.h"
// Only for reprogramming over WiFi (BLOCK 27). The robot does NOT use the
// network for anything else: the link with the Pi is still the USB cable,
// and if the network does not connect the firmware works the same.
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "secrets.h"

// Tag of the flashed version. It shows up in the READY line at boot and in
// the 'S' status: it is the way to check from the Pi that a WiFi upload
// really arrived, without opening the monitor. Change it on every upload.
#define FW_VERSION "2026-09-16v_pegar_suave"

// ######################################################################
// BLOCK 01 -- COMMUNICATION CONTRACT WITH THE RASPBERRY PI
// WHAT IT DOES: the only point of agreement between the two boards. Link
//   over the USB-C cable that joins the Pi with the ESP (`Serial`, 115200).
// THE ESP ONLY EXPOSES PRIMITIVES: it does not compose motions, does not
//   remember where it is facing and does not know the maze. The
//   orientation lives in AgenteRL.orientacion, on the Pi.
// IMPORTANCE: CRITICAL -- changing a code here forces changing it in
//   raspberry_pi/serial_link.py in the same commit.
// ######################################################################
#define CMD_PARAR        0   // immediate brake, setpoints to 0, loops cleared
#define CMD_AVANZAR      5   // advances ONE cell forward (the only translation)
#define CMD_CENTRAR      9   // does not translate: it centers between walls
// 🔑 ADVANCES UP TO THE WALL, without the cell limit. It is CMD_AVANZAR
// without the two rules that come from the cell maze: it neither refuses
// because another whole cell does not fit (UMBRAL_N), nor stops at
// PASO_CELDA_CM. Only for when the front sensor sees DIST_STOP_N, with the
// same dedicated reading and no filter.
// 🔴 IT EXISTS BECAUSE THE LOOP CANNOT LIVE ON THE Pi. Closing it over WiFi,
// between the ToF filter and the network the script decided with data up to
// 1.3 s old, and the robot ended up touching the wall (Sep 15: stops at 1 to
// 4 mm with a target of 41). Here the cycle is 40 ms and the reading is direct.
// CMD_AVANZAR and the maze demo stay untouched.
#define CMD_HASTA_PARED 'A'  // advances up to DIST_STOP_N, no cell limit
// 🔑 REVERSE ONE CELL (Sep 15 night): in a dead end the robot does NOT make a
// half turn (it sweeps 263 mm in a 250 mm corridor): it backs up one cell,
// with the SAME forward motion as always (ramp, taps, heading loop) and the
// same stopping criterion, DIST_STOP_N, applied to the rear sensor. It
// closes with FIN_CELDA / PARED like the forward motion.
#define CMD_REVERSA     'D'
// 🔑 ALIGN WITH THE HEADING GRID (Sep 15, afternoon). The heading is kept in
// ABSOLUTE terms: 'Z' sets the base (the North of the maze) and each turn
// adds quarter turns; the "right" heading is always base - 90*quarters. 'Y'
// turns whatever is missing to be within TOL_ALINEAR_DEG of that heading and
// closes with EV_FIN_GIRO (or EV_ERR_GIRO if it fails). No motion if it
// already is.
// 🔴 CMD_AVANZAR REFUSES if the heading is more than TOL_AVANCE_DEG away from
// the grid one: it answers EV_DESALINEADO instead of starting. A tilted
// advance is a diagonal in the maze, and that has to be impossible.
#define CMD_ALINEAR    'Y'
#define CMD_GIRO_DER   'R'   // +90 degrees (clockwise seen from above)
#define CMD_GIRO_IZQ   'L'   // -90 degrees
#define CMD_GIRO_180   'T'   // half turn

// Codes that travel in the `cmd` field of the CSV (the turns as 10/11/12 so
// they do not clash with the digits).
#define CODCSV_GIRO_DER  10
#define CODCSV_GIRO_IZQ  11
#define CODCSV_GIRO_180  12

// Events. The Pi advances its state machine ONLY with these, never by a
// timer. Each one is reported ONLY once.
#define EV_NADA          0
#define EV_ACK           1   // command accepted, starts executing
#define EV_NACK          2   // command rejected (wall, invalid code)
#define EV_FIN_CELDA     3   // cell completed, robot stopped and ready
#define EV_PARED         4   // braked for a wall before completing the cell
#define EV_CAMPO_ABIERTO 5   // the 4 ToF out of range: it left the maze
#define EV_BLOQUEO       6   // active setpoint but the encoders do not turn
#define EV_FIN_GIRO      7   // turn completed within tolerance
#define EV_ERR_GIRO      8   // the turn did not converge, or there is no IMU
#define EV_RUEDA_CORTADA 9   // a wheel ran away and the controller cut it off
#define EV_DESALINEADO  10   // CMD_AVANZAR rejected: heading far from the multiple of 90

// ######################################################################
// BLOCK 02 -- PINS, WHEELS AND POLARITY
// WHAT IT DOES: physical map of the 4 H-bridges and the wheel table, which
//   is the central structure of the whole control.
//
// WHEEL ORDER: DI, DD, TI, TD (front/rear x left/right). It is NOT the
//   M1..M4 order of the old firmware.
//
// 🔴 MAPPING, POLARITY AND SIGNS: MEASURED with tests 03 and 04 on Sep 14,
//   robot on its stand with the wheels in the air, three rounds with the
//   same figures.
//     DI = M2, mux c1     DD = M1, mux c7
//     TI = M4, mux c6     TD = M3, mux c0
//
//   POLARITY (invertirMotor) and ENCODER SIGN: CHECKED on the robot by
//   asking it to drive forward on its stand and watching the four wheels
//   turn. Only DD is inverted; the other three are not. The signs that make
//   the four measure POSITIVE when going forward are -1, +1, -1, +1 in table
//   order.
//
//   🔴 THE CHECK IS VISUAL AND NO MEASUREMENT REPLACES IT: with a wheel's
//   polarity reversed AND its sign reversed, that wheel measures positive
//   while pushing backwards, the loop is happy and the numbers look
//   perfect. The two errors hide each other. That is why a new table is
//   validated by LOOKING at the four wheels, and in both modes: driving and
//   turning.
//
//   When changing invertirMotor of a wheel its sign must change too,
//   because "forward" becomes the other encoder direction.
//
//   SIGNO_YAW=+1.
//
// 🔴 THE DD ENCODER (channel 7) HAS ITS AGC AT 5 OUT OF 128: magnet too
//   close to the chip. It is the only channel that throws I2C errors during
//   the test and the only one whose speed does not repeat between rounds
//   (261, 202 and 87 rpm in the three). Move that magnet away before
//   trusting its odometry or its rpmA160.
//
// rpmA160: MEASURED in test 04 on Sep 14, wheels in the air, three rounds
//   that repeat within half an rpm -- except DD. The DD one (190) is
//   PROVISIONAL: its encoder never gives the same value twice (261, 202 and
//   87 rpm in the three rounds) because the magnet is loose, so it comes
//   from the two stable measurements of Sep 1 on this same motor (189.4 and
//   191.0). Measure it again as soon as the magnet is firm.
//
// The PWM comes from `pwmDelModelo`: 160 * rpm / rpmA160. A wheel with an
//   underestimated rpmA160 gets MORE PWM than it needs and starts before the
//   other three, which is what DD did with the 111.4 it had.
//
// The take-off PWMs are still to be measured again ON THE FLOOR, one wheel
//   at a time: the table ones and the ones learned in the air are lower
//   than the real ones.
// ######################################################################
const int M4_PWM =  0, M4_IN1 = 16, M4_IN2 =  4;
const int M3_PWM = 32, M3_IN1 =  5, M3_IN2 = 23;
const int M2_PWM = 33, M2_IN1 = 26, M2_IN2 = 25;
const int M1_PWM = 13, M1_IN1 = 14, M1_IN2 = 12;
const int CH_M4 = 0, CH_M3 = 1, CH_M2 = 2, CH_M1 = 3;

const int FREQ_PWM = 5000;
const int BITS_PWM = 8;
const int PWM_MIN = 0;
const int PWM_MAX = 255;

// ######################################################################
// BLOCK 03 -- I2C BUS, MULTIPLEXER AND ADDRESSES
// WHAT IT DOES: everything (4 ToF + 4 AS5600 encoders + IMU) hangs from the
//   same bus. ToF and encoders go through the TCA9548A; the BNO085 goes
//   directly on the trunk.
//
// 🔴 CHANNEL ASSIGNMENT -- THE FIRST THING TO VERIFY ON THE ROBOT.
//   The ENCODER channels are measured (they come from carro_v08, which
//   drove with them). The ToF ones are NOT: the modular firmware had them
//   on 0..3, which now hold two encoders, so they were moved to the four
//   free ones. If a ToF always reads 600 mm (MAX_DIST), the first thing to
//   look at is this table, not the sensor.
//
//   (table updated with the CANAL_* below, measured on Sep 12; the previous
//   version of this table listed the Sep 8 assignment)
//        channel 0  encoder TI         channel 4  ToF East   (right)
//        channel 1  encoder DI         channel 5  ToF North  (front)
//        channel 2  ToF South (back)   channel 6  encoder DD
//        channel 3  ToF West  (left)   channel 7  encoder TD
//   🔴 The UMBRAL_* and DIST_STOP_* of BLOCK 04 are "per sensor" (each ToF
//   with its offset) and come from the modular firmware, where N was
//   channel 3. The channels were swapped (N<->O, E<->S) but the thresholds
//   were NOT swapped with them: the sensor that is N today uses the
//   threshold measured for another one. See BLOCK 04.
// ######################################################################
const int SDA_BUS = 18, SCL_BUS = 19;
#define MUX_ADDR 0x70
// The BNO085 answers at 0x4A or 0x4B depending on the module jumper. Both
// are tried at boot instead of assuming one (on this car it is 0x4B).
const uint8_t DIR_IMU_A = 0x4A;
const uint8_t DIR_IMU_B = 0x4B;
const uint8_t DIR_AS5600 = 0x36;
const uint8_t REG_ANGULO_AS = 0x0C;
const uint8_t REG_ESTADO_AS = 0x0B;
const uint8_t REG_AGC_AS    = 0x1A;

// ToF channels. The letters are in the CHASSIS FRAME:
//   N = sensor facing the FRONT of the robot, E = its right,
//   S = back, O = its left.
// The Pi converts them to the absolute frame with a_marco_absoluto(orientation).
//
// 🔴 MEASURED ON THE ROBOT (Sep 12): with the Sep 8 assignment (N=3 E=2 S=4
//   O=5) the sensor called N faced WEST, the O one faced NORTH, and E/S were
//   swapped. It was fixed here: N<->O and E<->S. This matters more than on
//   the Pi, because CMD_AVANZAR refuses if pared[IDX_FRENTE]: with the swap,
//   the ESP refused to advance because of a west wall and advanced against
//   the front one. With this fixed, the Pi's config.TOF_REMAPA has to be
//   EMPTY, or they get swapped twice.
#define CANAL_N  5
#define CANAL_E  4
#define CANAL_S  2
#define CANAL_O  3

void tcaSelect(uint8_t canal){
    if(canal>7) return;
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(1<<canal);
    Wire.endTransmission();
    delay(2);
}
// Opens a channel WITHOUT the delay(2). The AS5600 can talk immediately;
// the VL53L0X, which make a whole measurement, cannot.
bool tcaAbrirRapido(uint8_t canal){
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(1<<canal);
    return Wire.endTransmission() == 0;
}
// Closes ALL the channels. Mandatory before talking to the IMU, which lives
// on the trunk.
void tcaCerrar(){
    Wire.beginTransmission(MUX_ADDR);
    Wire.write((uint8_t)0);
    Wire.endTransmission();
}

Adafruit_VL53L0X lox;
Preferences memoria;

// 🔴 THE BUS IS SHARED BY THE TWO CORES: core 0 reads encoders and IMU and
// core 1 reads the ToF. EVERY transaction goes inside this mutex; without
// it, a half-done tcaSelect between the two tasks leaves the mux open on two
// channels and the readings come out mixed. It is NOT recursive: nobody can
// take it twice.
SemaphoreHandle_t mtxI2C = NULL;
#define I2C_TOMAR()  xSemaphoreTake(mtxI2C, portMAX_DELAY)
#define I2C_SOLTAR() xSemaphoreGive(mtxI2C)

// ######################################################################
// BLOCK 04 -- PERCEPTION THRESHOLDS AND GEOMETRY
// SOURCE of the thresholds: measured in the field, from the confirmed pinout.
// SOURCE of the geometry: carro_v08/estacion/geometria.py, taken from the CAD.
// ######################################################################
// 🔴 UMBRAL_N=300 against a cell step of 270 mm (PASO_CELDA_CM): the wall at
// the END OF THE NEXT CELL is ~270 + (half a cell - sensor overhang) from the
// ToF, i.e. ~300-330 mm with the robot centered. It is enough for the robot
// to be a few cm ahead in its cell (a turn that translates, a late stop;
// there is no longitudinal re-centering: CMD_CENTRAR does nothing) for the
// wall of the OTHER cell to fall below 300 and CMD_AVANZAR to refuse with a
// NACK against a free cell; the Pi also records that ghost wall on the map
// (anotar_pared_descubierta). The number is not changed blindly: dN has to
// be measured with the robot centered (a) with a wall at the end of ITS
// cell and (b) with the cell in front free and a wall at its end; the
// threshold goes between the two with a margin. If (a) and (b) overlap, it
// is geometry, not threshold.
// 🔴 IN REAL DISTANCE since 2026-Sep-15. The ToF readings already come out
// corrected (BLOCK 04b), so each threshold is translated with the line of
// ITS sensor: 'real = (read - c) / m'. This way every wall decision is
// exactly the same as before the correction. The raw values they come from
// were N 300, E 277, S 300, O 352.
// 🔑 AND A WARNING THAT COMES FROM BLOCK 02: these four numbers were different
// from each other because they compensated each ToF's offset by hand, BUT
// they are swapped -- the sensor that is N today uses the threshold measured
// for another one. That compensation is now done by the correction, so their
// spread (266 to 314 mm) no longer describes anything physical: the maze
// geometry is symmetric. Unifying them into a single real threshold is the
// next step, and it has to be measured before doing it.
// 🔴 SIDES AT 200 (Sep 15 night): with 313.6 the left one marked a wall with
// an opening at 296 mm and the robot turned back in normal corners. A wall
// to the side reads 40-70; a side opening 250-300. Editable live:
// '#umbralN=' '#umbralE=' '#umbralS=' '#umbralO='.
// 🔴 ALL AT 200 (Sep 15 night, 2nd time): the front one at 284.6 against
// readings of 285-290 rejected the '5' 19 times in a row depending on the
// noise. The rule "another whole cell does not fit" belonged to a maze of
// exactly 27 cm; the advance brakes by sensor at DIST_STOP_N anyway.
float UMBRAL_N = 200.0f;
float UMBRAL_E = 200.0f;
float UMBRAL_S = 200.0f;
float UMBRAL_O = 200.0f;

// Translated the same way, from raw N 110, S 110, E 90, O 90.
// ⚠️ THESE FOUR FALL OUTSIDE THE CALIBRATED RANGE (150-350 mm): the line is
// extrapolated downwards, where it is not measured. Closing that requires a
// point at ~100 mm on each sensor. Only DIST_STOP_N is used today (see the
// advance).
// 🔴 61 since Sep 15 night (was 71.1): one centimeter closer to the corner
// before turning and sticking to the wall. Editable: '#parada='.
// 🔴 Sep 16 (afternoon): '5' and 'A' stop at DIST_STOP_N (61) in corners, and
// at DIST_ARRIMAR (110) ONLY IN A T: wall in front with BOTH sides open
// (> UMBRAL_E/O). With 110 everywhere the robot did not get deep enough into
// the corners and the turn went wrong; with 61 in the T it ended up stuck at
// the back. The REVERSE stops at DIST_STOP_S (61) on the rear sensor.
float DIST_STOP_N = 61.0f;
float DIST_STOP_S = 61.0f;
// 🔴 SAFETY THRESHOLDS PER SIDE (Sep 16): in ANY automatic motion (advance,
// reverse, move closer, stick to the wall, manual), if a ToF reads below its
// own threshold the robot brakes hard: it is about to touch. They are low on
// purpose -- the normal lateral setpoints are 43-52 mm -- so they do not
// trigger in the corridor, but they do before knocking a wall down.
// The rear one is the least reliable (it reads too little), that is why its
// threshold is the lowest. Editable '#segN=' '#segE=' '#segS=' '#segO='.
float SEG_N = 35.0f, SEG_E = 22.0f, SEG_S = 28.0f, SEG_O = 22.0f;
// The reverse goes slower than the advance (a fraction of the cruise
// speed): the rear sensor does not see well and with the full cruise
// inertia it crashed.
float FRAC_REVERSA = 0.6f;
int   ladoSeguridad(float d[4]){          // 0 = none, 1..4 = N,E,S,O
    if(d[0] > 1.0f && d[0] < SEG_N) return 1;
    if(d[1] > 1.0f && d[1] < SEG_E) return 2;
    if(d[2] > 1.0f && d[2] < SEG_S) return 3;
    if(d[3] > 1.0f && d[3] < SEG_O) return 4;
    return 0;
}
// 🔴 'A' (move closer) stops at DIST_ARRIMAR, not at DIST_STOP_N (Sep 16):
// measured on the floor, with the front sensor ~110 mm from the back wall
// the robot is CENTERED in the crossing cell (asymmetric T included: the
// reference is the back wall, not the arms). It turns from there.
// Editable '#arrimar='.
float DIST_ARRIMAR = 110.0f;
const float DIST_STOP_E = 86.5f;
const float DIST_STOP_O = 62.4f;

const float MAX_DIST        = 600.0f;
const float NEURO_NORM_DIST = 1200.0f;

// 500 raw was equivalent to 509.3 / 480.8 / 469.4 / 455.5 real mm depending
// on the sensor; since it is a SINGLE threshold for the four the average is
// taken, so this is the only place where the behavior changes a bit with
// the correction.
// 🔴 300 since Sep 15 night, and WITHOUT the rear sensor (it reads <100 mm
// almost always): front and sides beyond 30 cm = out of the maze. Editable
// '#fuera='.
float UMBRAL_FUERA           = 500.0f;   // 50 cm (Sep 15 night)
const int   N_CONFIRMA_FUERA = 3;        // consecutive loops to confirm it

// ######################################################################
// BLOCK 04b -- ToF CALIBRATION (2026-Sep-15)
// WHAT IT DOES: converts the raw reading of each VL53L0X into real distance.
// IMPORTANCE: HIGH. Without this the four sensors lie in different ways:
//   measured against a board at 150, 250 and 350 mm, the error went from
//   +5.7 to +40.3 mm depending on the sensor and the distance. Corrected,
//   they stay below 1 mm on the three side/rear ones and 3.5 mm on the front.
//
// Each sensor responds 'read = c + m * real', so 'real = (read - c) / m'.
// Method, raw data and fit in bench_tests/13_tof_calibration/.
//
// 🔑 THREE OF THE FOUR SHARE THE SLOPE (1.040 / 1.032 / 1.043): that 3.8%
// long scale belongs to the sensor batch and not to the mounting, and the
// only thing that distinguishes them is the offset (0.0 / 15.6 / 24.9 mm).
// 🔴 THE FRONT ONE STANDS OUT IN BOTH: it compresses (0.890) instead of
// stretching, and it carries a 46.7 mm offset, the largest of the four. A
// large and constant positive offset is the symptom of a signal returning
// too early -- something inside the 25-degree cone: a cover, tape or the
// mount itself. CHECK ITS MOUNTING; if something is removed from there, this
// sensor has to be measured again.
//
// ⚠️ IT IS ONLY MEASURED BETWEEN 150 AND 350 mm. The 600 mm point of Sep 14
// falls 10.1 mm below the line, ten times the spread: near its cap the
// sensor compresses more than the line describes.
// ######################################################################
const float TOF_C[4] = {46.7f, 0.0f, 15.6f, 24.9f};      // N, E, S, O
const float TOF_M[4] = {0.890f, 1.040f, 1.032f, 1.043f};

// 🔴 MAX_DIST ("I see nothing") IS NOT CORRECTED AND IS THE CEILING OF THE
// RESULT. Passed through the lines it would give 621.7 / 576.9 / 566.3 /
// 551.4 and would stop being a recognizable value, which is exactly what
// distinguishes "I see nothing" from "there is something far away".
float corregirToF(int i, float d){
    if(d >= MAX_DIST) return MAX_DIST;
    float r = (d - TOF_C[i]) / TOF_M[i];
    if(r < 0.0f)     r = 0.0f;
    if(r > MAX_DIST) r = MAX_DIST;
    return r;
}

// 🔴 These two come from carro_v08 and do NOT match the old firmware, which
// assumed a 65 mm diameter wheel. This is the right one: it comes from the
// CAD and it is the one with which each motor's rpm are measured.
// 🔴 0.030, NOT 0.029. The mecanum is 60 mm in diameter, not 58. With the old
// value the robot advanced 3.4% MORE than its odometry told it: in a 27 cm
// cell, almost one centimeter more, accumulating along the path.
// `distanciaTramoCm` and all the navigation come from here.
// Whatever error is left after this -- roller slip, which is not zero on
// mecanum wheels -- goes into FACTOR_ODOM, which exists for that.
const float R_RUEDA = 0.030f;    // m (60 mm wheel)
const float LSUM    = 0.14525f;  // half wheelbase + half track

float PASO_CELDA_CM = 27.0f;             // ADJUST to the exam maze
// safety cap of CMD_HASTA_PARED: if no wall appears within that distance, it
// stops anyway and reports it as end of cell
float TOPE_HASTA_PARED_CM = 150.0f;
// 🔴 1.09, MEASURED WITH TAPE ON THE FLOOR on Sep 14 and CHECKED over 9
// consecutive cells: the real cell measures 26.94 cm against the 27
// requested, and the accumulated error over those 9 cells is HALF A
// CENTIMETER (0.2% over 2.4 m). The cell-to-cell spread is +-0.5 cm and it
// is random, not biased: that is why it does not grow with the distance.
//
// 🔑 DO NOT compare the telemetry `cm_tramo` with the tape to recalibrate
// this: that value is the last one that manages to be sampled over the
// network before stopping, so it comes out short by construction (26.5
// against 26.9 real) and leads to inflating the factor. The tape is the
// reference.
//
// Converted to a diameter, the wheel ROLLS as if it were 65.5 mm, not the 60
// its body measures: it is the mecanum rollers. (The old firmware used 65
// and someone "fixed" it to 58 with the CAD figure; the CAD describes the
// part, not the rolling.)
//
// 🔑 IT IS CORRECTED HERE AND NOT IN R_RUEDA ON PURPOSE: R_RUEDA also
// converts the speed in m/s to wheel rpm (see `repartir`), so touching it
// would change each wheel's target from 77 to 70 rpm and decalibrate
// everything measured in rpm -- which is everything. FACTOR_ODOM corrects
// ONLY the distance.
float FACTOR_ODOM = 1.09f;               // fine correction of the odometry

#define TX_S3 22
#define RX_S3 21

const String DIRS4[4] = {"N","E","S","O"};   // CHASSIS frame

// ######################################################################
// BLOCK 05 -- TIMING AND NEUROCONTROLLER CONSTANTS
// ######################################################################
// Control rates, from carro_v08. The encoder is read FAST so as not to lose
// counts; the loop corrects SLOWLY to give the motor time to respond.
const int MS_MUESTRA = 20;    // encoder reading
// 🔴 THE LOOP RAN AT 100 ms, i.e. the PWM was recomputed TEN TIMES PER SECOND
// (Sep 14: "it does not move smoothly, it jerks, the inertia moves it too
// much"). For a motor speed control that is very slow, and it shows right at
// the start, which is when there is something to correct: the four wheels
// jump to their take-off PWM at the same time and come out at speeds that
// look nothing alike -- DD at 98 rpm, DI at 56, TI at 48, TD at 21, almost a
// five-to-one spread -- and the loop cannot touch anything until 100 ms
// later. In that time the car has already jerked, and what one sees is one
// wheel shooting off and the others apparently still while the loop equalizes
// them in jumps.
//
// At 25 ms the loop corrects four times more often and still has a fresh
// measurement, because the encoder is read every 20 ms.
//
// 🔑 ALFA GOES DOWN WITH THE PERIOD, AND IT IS NOT OPTIONAL. The rpm filter is
// applied ONCE PER LOOP ITERATION, so its time constant is measured in
// iterations, not in seconds: 0.4 every 100 ms filters the same as 0.12 every
// 25 ms. Keeping 0.4 with a loop four times faster would filter four times
// less and feed the PID all the encoder counting noise.
//
// If the control becomes nervous, the way back is this pair together:
// MS_CONTROL 100 and ALFA 0.4 (the carro_v08 values). Never one without the
// other.
const int MS_CONTROL = 25;    // speed and heading loop
const int MS_SALUD   = 500;   // AGC and magnet of the AS5600
// 50 ms (before 100) so that a 5 s push leaves 100 samples and not 50: with 50
// one cannot see what happens at the start, which is exactly the half second
// that needs diagnosing. A line is ~330 bytes, so at 20 lines/s the port runs
// at 57% of the 115200 baud -- with margin. DO NOT raise it further without
// recomputing that: beyond the bandwidth characters are lost and the lines
// arrive cut.
const unsigned long PERIODO_CSV_MS = 50;
// 🔴 ALFA STAYS AT 0.12, AND 0.06 WAS ALREADY TRIED ON THE ROBOT. The AS5600
// magnets are off-center with respect to the shaft (a mechanical tolerance
// that cannot be removed), so each wheel over-measures at one point of its
// revolution and under-measures at the opposite one: raw, the front left
// deviates 36% (test 11, fixed PWM, shafts moved by hand).
//
// On that data, offline, 0.06 looked better than 0.12 -- it left the worst
// wheel at 7.3% instead of 12.1%. ON THE ROBOT IT DOES NOT HOLD. Two
// comparable 4.5 s batches, on the stand:
//   steady state: DI improves (11.5% -> 7.4%), TI the same (7.7%), TD GETS
//            WORSE (4.4% -> 7.5%).
//   start: the peaks go up (DI +27% -> +40%, TI +23% -> +31%,
//            TD +27% -> +35%) and take longer to get into the band.
//
// 🔑 WHY: measuring the signal in OPEN loop says how much noise it has, but
// not what the delay costs INSIDE the loop. With 0.06 the time constant goes
// from 208 to 417 ms, and that feedback delay is what produces the
// overshoot. Gaining cleanliness on one wheel does not pay for making the
// start of all four worse.
//
// What is still to be done is OUTSIDE the loop: the odometry averages the
// four wheels equally today, and in a 1 s window the noise of each one is
// very different (TD 2.1%, TI 2.8%, DI 6.5%, DD 17.2%). Weighting by that
// noise adds no delay to any loop.
const float ALFA = 0.12f;     // smoothing of the measured rpm (see above)

const float DT         = 0.05f;
const float TAU_NR     = 0.35f;
const float TAU_GA     = 0.10f;
const float TAU_MEM    = 0.50f;
const float TAU_RET    = 0.80f;
const float UMBRAL_RET = 0.55f;
const float W_EXCIT    = 0.40f;
const float W_INHIB    = 0.60f;
const float SIGMA_GA   = 35.0f;

// ######################################################################
// BLOCK 06 -- CONTROL GAINS  (EDITABLE LIVE FROM THE Pi)
// ----------------------------------------------------------------------
// 🔑 ALL MEASURED WHILE DRIVING ON BATTERY on 2026-09-02 in carro_v08. They
// are not bench or theoretical values: each one came from reading a
// recorded run. Result with them: final deviation 0.15 +- 0.11 degrees over
// four 8-9 s stretches, and steady-state speed error of 0.2 to 2.8 rpm per
// wheel.
//
// They are left as NON-const `float` on purpose: the Pi interface changes
// them live (command '#'), which is how they were calibrated.
//
// WHY KD AND KI_YAW ARE ZERO (and it is not that they still need tuning):
//   Speed KD -- the loop never overshot, there was nothing to damp, and the
//     signal is bad for differentiating (rpm counted in a 100 ms window and
//     filtered on top).
//   Heading KI -- the imbalance is already corrected one level below: each
//     wheel has its own integral that keeps it within 1-3 rpm. The mean
//     steady-state heading correction was half an rpm; if there were a
//     constant disturbance, the proportional term would settle at a fixed
//     non-zero value, and it does not.
// ######################################################################
// 🔴 KP 0.40 AND KI 0.10, MEASURED HOT (Sep 14). With 0.25 and 0.20 the front
// right wheel did a slow back-and-forth -- it slowed down a bit and
// corrected, with a period of over a second and the PWM in antiphase with
// the speed: the loop chasing, not sensor noise. Sweep of six sets of gains
// in a single run, measuring the peak-to-peak excursion of each wheel over
// 10 s stretches:
//
//   kp    ki   | f.left   f.right  r.left    r.right
//   0.25  0.20 |   13.5     21.0      7.8       5.0
//   0.40  0.20 |   15.0      4.4      6.6       1.7
//   0.55  0.20 |   14.1      4.3      7.0       1.7
//   0.25  0.10 |   13.3      3.4      6.3       1.9
//   0.40  0.10 |   13.4      2.9      6.3       1.5   <-- the chosen one
//   0.25  0.05 |   12.8      2.9      6.4       1.7
//
// Raising KP above 0.40 adds nothing. The front left does not move with any
// combination because its problem is NOT the loop: it is its encoder, with a
// 36% periodic error due to the off-center magnet.
// A low KI is affordable because the feed-forward line (BLOCK 07) already
// puts the PWM in place; the integral only has to clean up what is left.
float KP = 0.40f;        // speed: current error
float KI = 0.10f;        // speed: removes the difference between wheels
float KD = 0.0f;         // speed: derivative ON THE MEASUREMENT

float KP_YAW = 3.6f;     // heading: degrees of deviation -> rpm of difference (carro_v08)
float KI_YAW = 0.0f;     // heading: removes the fixed deviation
float KD_YAW = 0.7f;     // heading: brakes with the measured turn rate (carro_v08)
float CORR_MAX = 30.0f;  // cap of the heading correction, in rpm (carro_v08)

// 🔴 SIGNO_YAW IS APPLIED TO THE MEASUREMENT, NOT TO THE OUTPUT. Putting it at
// the end fixes going straight but NOT turning: the reference advances in
// the control convention and the gyroscope may count the other way, so when
// a turn is requested the reference goes up while the measurement goes
// down, the error grows without bound and the correction stays stuck at the
// cap. No value put at the output fixes that.
//
// 🔑 CONVENTION (Sep 12, the usual math): POSITIVE yaw = turn to the LEFT
// (counter-clockwise seen from above), which is how the BNO085 delivers it
// mounted with Z up, and W>0 in the kinematics is also left (right side
// faster). Turning the robot to the right by hand the yaw DROPS (-88 in the
// Sep 12 test) and that is CORRECT. The heading loop is written in that
// convention; the 90-degree turn (ejecutarGiro) translates "steps to the
// right" into negative degrees. That is why SIGNO_YAW is +1.
// On Sep 12 it was set to -1 for a while: it was a symptom of the motor
// polarity (all four reversed), not of the sensor. THE SYMPTOM OF A WRONG
// SIGN, to recognize it in a minute: the robot ALWAYS drifts to the same
// side and the correction does NOT reduce the deviation at all, because it
// adds speed to the opposite side from the one needed. With the wrong sign
// the loop is POSITIVE feedback and no gain fixes it. It is flipped from the
// Pi interface and takes effect immediately.
float SIGNO_YAW = +1.0f;

// Working speeds. The cruise one is the one calibrated in carro_v08.
// 🔴 0.22 m/s = 72 rpm per wheel, AND THAT MINIMUM IS NOT A PREFERENCE. The
// front right wheel has a shorter gearbox than the other three (it gives
// 365 rpm at PWM 255 against 160-175 for its companions), so it has less
// torque and does not turn repeatably below about 70 rpm. The hardware is
// final, it does not change. Below 70 rpm that wheel stalls, the integral
// pushes it, it breaks free suddenly and jerks.
// Before, this was 0.114 m/s = 38 rpm: THE WHOLE working range of the robot
// was below the minimum of its worst wheel. See bench_tests/12_pwm_sweep.
float VEL_CRUCERO = 0.12f;     // m/s of the cell advance (V_CRUCERO 102/255 of the square test; was 0.22)
// Turn cruise speed. Moderate on purpose: the turn is not a race, and the
// faster it goes the more inertia to brake at the end. Editable ('#vgiro=').
// 🔴 60 degrees/s = 50 rpm per wheel, MEASURED ON THE FLOOR on Sep 14. With 45
// the turn closed well (error -0.57) but took 3.5-3.8 s; with 80 it arrives
// a second earlier and OVERSHOOTS by 2 to 3 degrees, having to come back.
// At 60, braking from 45 degrees, it closes just as well and a bit faster.
// Three repetitions per side: mean error -0.22 to the right and -0.20 to the
// left, half a degree of spread.
// 🔴 RAISING IT DOES NOT MAKE THE TURN SMOOTHER: IT MAKES IT WORSE. Full sweep
// on the floor on Sep 14 -- 60, 70, 72, 80, 90 and 100 degrees/s, with five
// braking points and two minimum speeds, FOUR turns per point (two to each
// side):
//
//   vgiro/brake/min | "hesit." | jitter | mean error
//       60 / 45 / 25 |  7.3%  |  13.9  | +1.5
//       72 / 60 / 25 | 11.4%  |  21.9  | +1.4
//       80 / 70 / 45 | 18.0%  |  17.5  | +2.0  (spread 2.9)
//       90 / 78 / 50 | 25.5%  |  24.5  | +1.0  (spread 1.9)
//
//   "hesitation" = percentage of the turn with some wheel below 12 rpm while
//   it is being asked to turn. It is what from outside looks like a turn
//   that stops and starts. **The lowest speed wins in all three columns.**
//
// 🔑 What DID reduce the shaking were the LOADED LINES of BLOCK 07: at the
// same braking, the mean PWM jump went from 13.3/13.9 to 8.8/8.1. Not the
// speed.
//
// 🔴 THE HARD STOP WAS TRIED AND IS WORSE (gradfreno=5, which keeps the speed
// constant until the end):
//
//   vgiro | hesit. | jitter | error
//     60  | 23.3%  |  15.6  | +2.5
//     66  | 14.9%  |  19.1  | -0.5 (spread 2.7)
//     80  | 15.5%  |  22.8  | +2.4 (spread 0.5)
//
//   The cause is in this very loop: when the angle is reached the motors are
//   cut, it waits T_INERCIA_MS and CHECKS whether it ended within
//   TOL_GIRO_DEG; if not, the while RETRIES the whole turn from standstill.
//   Cutting hard the car overshoots, ends outside the tolerance and retries
//   -- and each retry is one more start. That is what from outside looks
//   like hesitating, and that is why the durations go up to 5.2 s.
//
// 🔴 AND COMPENSATING THE INERTIA DOES NOT WORK EITHER. Measured how much the
//   car rotates AFTER the motors are cut: 12.3 degrees at 60 degrees/s and
//   18.1 at 80 -- it grows in proportion to the speed, so in principle it
//   could be subtracted from the target. **But its spread between turns is
//   +-3.5 and +-2.6 degrees.** Compensating the mean would leave a residual
//   error of +-3, worse than the +1.5 with spread 1.2 of the current tuning.
//
// 🔑 AND THAT IS THE REASON THE PROPORTIONAL BRAKING WINS: arriving slowly,
//   the angle carried by inertia is small AND SO IS ITS VARIABILITY. Cutting
//   hard hands the last 12-18 degrees of the turn to something that is not
//   controlled.
//
// 🔴 THE HESITATION IS NOT REMOVED WITH THESE THREE PARAMETERS: 7% of the turn
// remains with some wheel below 12 rpm in the BEST tuning. Its cause is that
// the turn asks for 20-30 rpm per wheel during a good part of the motion,
// which is the weak zone of this hardware. Really attacking it requires
// changing HOW the turn brakes -- for example, constant speed and a hard
// stop instead of proportional braking --, not how fast it goes.
float VEL_GIRO_DPS = 60.0f;    // degrees/s of the turn on the spot
// In the last degrees the speed is lowered proportionally down to
// VEL_GIRO_MIN_DPS, to enter the band slowly instead of arriving at full
// speed ('#gradfreno='). 45. Braking from farther (52 or 60) closes somewhat
// better but the car spends more time in the braking zone, which is where
// the target is low and changing, and that is where the turn looks hesitant.
float GRADOS_FRENADA_GIRO = 45.0f;
// 🔴 THE RAMP WAS 0.15 m/s2, and that is the "it starts like a ramp, it is not
// progressive" of Sep 14: to reach the 0.30 m/s of the manual cap it took TWO
// whole SECONDS. In a five-second push, almost half of the distance was spent
// ramping up, and the car never drove at constant speed long enough to look
// smooth.
// At 0.45 m/s2 it took 0.67 s, but that was already a jolt: on Sep 14, with
// that value, "it starts too hard, it has to go from less to more". 0.22
// leaves the ramp at 1.4 s -- a real ramp, which feels progressive, without
// going back to the 2 s in which the whole push was spent ramping up. It is
// the first number to move if the start feels abrupt or lazy ('#acel=').
float ACEL_MAX  = 0.22f;   // m/s per second
// Braking. Livelier than the start on purpose: starting slowly is what
// avoids the jerk, but lengthening the braking only lengthens the stretch in
// which the wheels drift apart. See topeRampa().
float DECEL_MAX = 0.60f;   // m/s per second

// 🔴 THE HEADING CORRECTION ALSO HAS ITS OWN RAMP AND CAP.
// ACEL_CORR: how much the correction may move per second, in rpm. Without it
//   it jumps to its new value in one cycle and what one feels in the car are
//   the heading steps, not the acceleration.
// FRAC_CORR: maximum imbalance between sides, as a fraction of what is
//   requested. It was a fixed 0.40, which leaves one side at 140% and the
//   other at 60%: too much to straighten and enough for the car to stop
//   going at the set speed.
float ACEL_CORR = 25.0f;   // rpm per second
float FRAC_CORR = 0.22f;   // fraction of the requested rpm
float CORR_MIN  = 13.0f;   // floor of the heading authority, in rpm
// 🔴 TURN RAMP, INDEPENDENT OF THE DRIVING ONE ('#acelgiro=').
// At 2.5 rad/s2 the turn went from 0 to 35 degrees/s in a quarter of a
// second: for the wheels that is practically a step, and the one with the
// highest take-off starts late while the chassis is already rotating and
// drags it backwards. Rising together, there is time for the four to break
// free before the chassis moves -- which is what makes the turn happen on
// the spot and not as a displacement.
// 🔑 This number is ONLY for the turn: driving forward and backward keep
// ACEL_MAX and DECEL_MAX, unaffected by what is touched here.
// 🔴 TURN START RAMP, the equivalent of ACEL_MAX for driving.
// At 2.5 rad/s2 the turn reached cruise in a tenth of a second: for the
// wheels that is a step, and the one with the highest take-off starts late
// while the chassis is already rotating. At 1.2 it takes half a second to
// reach 40 degrees/s -- plenty of time for the four to break free together,
// which is what makes it turn on its axis instead of drifting. Editable
// ('#acelgiro=').
// 🔑 ONLY THE TURN: forward and backward keep ACEL_MAX and DECEL_MAX intact.
float ACEL_GIRO = 1.8f;    // rad/s per second

// Search for the take-off PWM (BLOCK 12). Asymmetric on purpose: it rises
// fast and goes down slowly. This was reached through two bad paths -- only
// rising, it climbed from 95 to 119 in five stretches, and starting below
// it only went down, from 119 to 91 in one run, which is literally what felt
// like "it gets worse little by little over several passes".
// 🔴 PASO_BUSQUEDA IS APPLIED ONCE PER LOOP CYCLE, so it also depends on
// MS_CONTROL. The 8 PWM points were measured with the loop at 100 ms: that
// was 80 points per second. With the loop at 25 ms, keeping 8 would mean
// rising 320 per second -- the wheel would "take off" with an inflated PWM
// in the first cycle and learn that number, which is exactly the current
// spike on every start that DESPEGUE_MAX exists to prevent. It is written as
// points per second and converted, so it does not get out of step again.
const float PASO_BUSQUEDA_POR_S = 80.0f;
const float PASO_BUSQUEDA = PASO_BUSQUEDA_POR_S * MS_CONTROL / 1000.0f;
// This one is NOT touched: it is applied once per take-off, not per cycle.
const float BAJA_TANTEO = 2.0f;
const float FACTOR_MANTIENE = 0.55f;

// ######################################################################
// BLOCK 07 -- THE WHEEL TABLE
// WHAT IT DOES: it is the central structure of the control. One row per
//   wheel with its wiring, its model (the pwmBase/pwmPorRpm line) and what
//   it learns (pwmDespegue).
// COST: one I2C reading per wheel every MS_MUESTRA.
// IMPORTANCE: CRITICAL. `signoEncoder` is NOT assumed: it is measured with
//   the calibration routine (command 'V') and saved. With a reversed sign
//   the loop is positive feedback and the motor goes to the cap in less
//   than a second -- that is why the runaway protection exists.
//
// 🔴 EACH WHEEL'S MODEL IS A LINE WITH AN INTERCEPT:
//   PWM = pwmBase + pwmPorRpm * rpm. Before it was 160*rpm/rpmA160, a line
//   forced through zero, and THAT WAS THE MISTAKE: no wheel starts moving
//   with zero PWM. Measured in test 12 (three full sweeps, 2 s steps, no loop
//   in between):
//
//     wheel              pwmBase   pwmPorRpm   valid between   max error
//     front left           22.6      1.369      15-158 rpm     4 PWM
//     front right          53.4      0.533      70-325 rpm    13 PWM
//     rear left            20.4      1.468      18-148 rpm     3 PWM
//     rear right           20.9      1.362      17-162 rpm    14 PWM
//
//   That 20-to-53-point step is what the PID had to invent with the integral
//   on every start: late, in jumps, and with the jerk at the end. With the
//   full line the feed-forward is already in place and the loop only has to
//   correct what is left.
//
//   It is checked against what the loop really used: for 38.8 rpm the front
//   left line asks for 76 and the loop was settling at 79.
//
// 🔴 LINES MEASURED UNDER LOAD, with the robot on the floor (Sep 14). Three
//   points per wheel -- 42, 58 and 76 rpm -- reading the PWM the loop settles
//   at in steady state. Maximum fit error: 3 PWM points.
//
//     wheel              pwmBase   pwmPorRpm     (in the air it was)
//     front left           38.6      1.213       22.6 / 1.369
//     front right          39.2      0.735        0.0 / 0.842
//     rear left            31.0      1.500       20.4 / 1.468
//     rear right           34.0      1.451       20.9 / 1.362
//
//   Under load the intercept GOES UP on all four -- there is more friction to
//   overcome -- and the slope barely changes. The front right had a ZERO
//   intercept because its line in the air no longer described it; under load
//   it does have an intercept, and that is what allows bounding its floor at
//   low speed without stalling it.
//
// 🔑 This model was first tried with the robot at 38 rpm and the batch came
//   out WORSE: at that speed the front right line was being extrapolated far
//   below where it was measured. With the robot already in its range (76 rpm)
//   the model is checked against what the loop really uses: for 76.5 rpm the
//   line asks for 127.3 / 132.7 / 125.1 and the loop settled at 127.3 /
//   134.4 / 127.3.
//
// 🔴 ALSO, THEY ARE MEASURED WITH THE WHEELS IN THE AIR. With the robot loaded
//   the pwmBase goes up (more friction to break) and so does the slope (each
//   PWM gives fewer rpm). So the model falls SHORT under load, which is the
//   safe side: the loop completes it. Measuring them again on the floor is
//   the pending task -- see `bench_tests/12_pwm_sweep/README.md`, which
//   explains the setup.
//
//   The starting signs are those measured in carro_v08 against the car:
//   DI -1, DD +1, TI -1, TD +1.
// ######################################################################
struct Rueda {
    const char *corto;
    int pwm, in1, in2, canalPwm;
    uint8_t canalMux;
    float pwmBase;        // PWM at which this wheel starts moving
    float pwmPorRpm;      // how much more PWM per rpm. See BLOCK 07
    bool derecha;         // to distribute the heading correction
    bool invertirMotor;   // true only on DD. See BLOCK 02
    int signoEncoder;     // MEASURED, not assumed

    int   anguloPrevio;
    float gradosVentana;  // since the last loop iteration
    float gradosTramo;    // since the stretch started (odometry)
    float rpm;            // signed: negative = turning backwards
    float objetivo;
    float integral;
    float salida;         // PWM being applied
    bool  vivo;           // its encoder answers
    bool  iman;
    int   agc;
    bool  desbocada;      // the controller cut it off
    int   sospechas;
    int   sentidoPrevio;
    float pwmDespegue;    // learned and saved in permanent memory
    float suelo;          // the one being tried now
    int   fallosI2C;
    float rpmPrevia;
    int   pasosBusqueda;
};

// The order is DI, DD, TI, TD. Motor, mux channel, polarity and encoder
// sign MEASURED with tests 03 and 04 on Sep 14 (see BLOCK 02).
Rueda ruedas[4] = {
  {"DI", M2_PWM, M2_IN1, M2_IN2, CH_M2, 1,  38.6f, 1.213f, false, false, -1,
   -1, 0,0,0,0,0,0, true, true, 0, false, 0, 0, 95.0f, 0.0f, 0, 0.0f, 0},
  // Its encoder is the one with AGC at 5/128: the speed it reports is
  // doubtful until the magnet is moved away. See BLOCK 02.
  {"DD", M1_PWM, M1_IN1, M1_IN2, CH_M1, 7,  39.2f, 0.735f, true,  true,  +1,
   -1, 0,0,0,0,0,0, true, true, 0, false, 0, 0, 83.0f, 0.0f, 0, 0.0f, 0},
  {"TI", M4_PWM, M4_IN1, M4_IN2, CH_M4, 6,  31.0f, 1.500f, false, false, -1,
   -1, 0,0,0,0,0,0, true, true, 0, false, 0, 0, 93.0f, 0.0f, 0, 0.0f, 0},
  {"TD", M3_PWM, M3_IN1, M3_IN2, CH_M3, 0,  34.0f, 1.451f, true,  false, +1,
   -1, 0,0,0,0,0,0, true, true, 0, false, 0, 0, 39.0f, 0.0f, 0, 0.0f, 0},
};
// The take-off PWMs the table above starts with (same column), to be able to
// go back to them with command 'B' when the learned one goes wrong -- on
// Sep 12 DI "learned" 247 because its encoder did not see it.
const float DESPEGUE_INICIAL[4] = {95.0f, 83.0f, 93.0f, 39.0f};
// Above this a take-off is not friction: it is an encoder that does not see
// the wheel or a badly wired motor, and learning it only adds a current
// spike on every start.
const float DESPEGUE_MAX = 170.0f;
// Output threshold per wheel (see corregirVelocidad). How much it may exceed
// the PWM the model asks for the target speed: 1.8 times plus 30 points.
// Generous on purpose -- the loop has to be able to correct -- but bounded,
// which is what prevents a loose wheel from going to the cap. Editable live.
float FACTOR_TECHO = 1.8f;
float MARGEN_TECHO = 30.0f;
// Below these requested rpm no take-off is searched: the wheel is still
// because little is asked of it (start ramp), not because it lacks push.
const float RPM_MIN_BUSQUEDA = 18.0f;
// Below these requested rpm the floor is not bounded to the model PWM: the
// target is still outside the range where the BLOCK 07 lines were measured,
// and the front right one is worthless there (it has a 0 intercept).
// 60 rpm is just below the cruise (72) and the minimum at which the four
// wheels turn repeatably (70).
// 🔴 15, not 60. The 60 was a precaution while the front right had a ZERO
// intercept: bounding its floor at low speed would have given it almost
// nothing and it would have stalled. Not anymore: its loaded line has a 39.2
// intercept, so at 25 rpm the model asks 58 PWM of it and not 21.
// Lowering it matters for the TURN, which works at 20-35 rpm per wheel: with
// the threshold at 60 the floor was NOT bounded there and the PWM alternated
// between 93 and 50-60 during the whole turn. That was the jitter visible
// from outside.
const float RPM_RANGO_TRABAJO = 15.0f;
const int N_RUEDAS = 4;

// What is asked of the robot body: forward and sideways in m/s, turn in
// rad/s. The command jumps to these through a ramp (carro_v08).
float ordenVx = 0, ordenVy = 0, ordenW = 0;
float pedidoVx = 0, pedidoVy = 0, pedidoW = 0;
bool  habilitado = false;           // the motors switch of the interface

// ######################################################################
// BLOCK 08 -- MOVING AVERAGE FILTER OF THE ToF (5 samples)
// IMPORTANCE: HIGH -- without it, a single outlier triggers a ghost wall.
// ######################################################################
struct Filtro {
    static const int N=5;
    float buf[N]; int idx; float suma;
    Filtro(){ memset(buf,0,sizeof(buf)); idx=0; suma=0.0f; }
    float agregar(float v){
        suma-=buf[idx]; buf[idx]=v; suma+=v;
        idx=(idx+1)%N; return suma/N;
    }
    // After a turn the 5 stored samples belong to ANOTHER orientation: if they
    // are not purged, the average mixes two worlds and the first "there is a
    // wall" after the turn is a lie.
    void purgar(float v){ for(int i=0;i<N;i++) buf[i]=v; suma=v*N; idx=0; }
} filtros[4]; // 0=N(front) 1=E(right) 2=S(back) 3=O(left)

// ######################################################################
// BLOCK 09 -- BIO-INSPIRED NEUROCONTROLLER
// WHAT IT DOES: 10 states. z[0..3] Naka-Rushton per sensor, z[4..7] gaussian
//   layer with excitation to the neighbours and inhibition to the opposite,
//   z[8] memory and z[9] reverse permission.
// IMPORTANCE: CRITICAL -- gauss_act[front] MODULATES the cruise speed
//   (BLOCK 13): the robot brakes by itself when approaching a wall, without
//   any hand-written ramp.
// ######################################################################
struct NeuroController {
    float z[10];
    float naka_act[4];
    float gauss_act[4];
    float mem_act, ret_act;
    bool  retroceso_permitido;

    NeuroController(){
        memset(z,0,sizeof(z));
        memset(naka_act,0,sizeof(naka_act));
        memset(gauss_act,0,sizeof(gauss_act));
        mem_act=ret_act=0; retroceso_permitido=false;
    }
    float _naka(float x){ x=max(0.0f,x); return (x*x)/(x*x+0.25f+1e-12f); }
    float _gauss(float a, float b){
        float d=a-b;
        while(d> 180.0f) d-=360.0f;
        while(d<-180.0f) d+=360.0f;
        return expf(-0.5f*(d/SIGMA_GA)*(d/SIGMA_GA));
    }
    float _ang(int i){ const float a[4]={90,0,270,180}; return a[i]; }

    void step(float dists[4], bool hay_salida){
        for(int i=0;i<4;i++){
            float sp=constrain(dists[i]/NEURO_NORM_DIST,0.0f,1.0f);
            z[i]=constrain(z[i]+(DT/TAU_NR)*(-z[i]+_naka(sp)),0.0f,1.0f);
            naka_act[i]=z[i];
        }
        for(int i=0;i<4;i++){
            int il=(i+3)%4, ir=(i+1)%4, io=(i+2)%4;
            float u=constrain(
                _gauss(_ang(i),_ang(i) )*z[i]
               +W_EXCIT*_gauss(_ang(i),_ang(il))*z[il]
               +W_EXCIT*_gauss(_ang(i),_ang(ir))*z[ir]
               -W_INHIB*_gauss(_ang(i),_ang(io))*z[io],
                0.0f,1.0f);
            z[4+i]=constrain(z[4+i]+(DT/TAU_GA)*(-z[4+i]+u),0.0f,1.0f);
            gauss_act[i]=z[4+i];
        }
        float dead=hay_salida?0.0f:1.0f;
        z[8]=constrain(z[8]+(DT/TAU_MEM)*(-z[8]+dead),0.0f,1.0f);
        z[9]=constrain(z[9]+(DT/TAU_RET) *(-z[9]+dead),0.0f,1.0f);
        mem_act=z[8]; ret_act=z[9];
        retroceso_permitido=(z[9]>UMBRAL_RET);
    }
} neuro;

// ######################################################################
// BLOCK 10 -- DIRECTION UTILITIES (chassis frame)
// ######################################################################
uint8_t canalDir(const String& d){
    if(d=="N") return CANAL_N; if(d=="E") return CANAL_E;
    if(d=="S") return CANAL_S; return CANAL_O;
}
bool esPared(const String& d, float dist){
    if(d=="N") return dist<UMBRAL_N; if(d=="E") return dist<UMBRAL_E;
    if(d=="S") return dist<UMBRAL_S; return dist<UMBRAL_O;
}
int indiceDir(const String& d){
    for(int i=0;i<4;i++) if(DIRS4[i]==d) return i;
    return 0;
}
const int IDX_FRENTE = 0;   // DIRS4[0] == "N" == chassis front

// ######################################################################
// BLOCK 11 -- AS5600 ENCODERS AND STRETCH ODOMETRY
// WHAT IT DOES: reads the angle of each AS5600 through the mux and
//   accumulates degrees, with their sign. The rpm (BLOCK 12) and the
//   centimeters of the stretch come from there.
// COST: 4 I2C transactions every MS_MUESTRA.
// IMPORTANCE: CRITICAL -- the odometry is the only thing that tells the Pi
//   "I crossed a cell". If it lies, the Pi map gets out of sync with the world.
// ######################################################################
bool leerRegistroAS(uint8_t reg, int bytes, int *valor){
    Wire.beginTransmission(DIR_AS5600);
    Wire.write(reg);
    if(Wire.endTransmission(false) != 0) return false;
    if(Wire.requestFrom((uint8_t)DIR_AS5600, (uint8_t)bytes) != bytes) return false;
    int v = 0;
    for(int i=0;i<bytes;i++) v = (v<<8) | Wire.read();
    *valor = v;
    return true;
}

// 🔴 WHEN the last encoder window was closed, and whether there is something
// new to measure. Both are written by muestrearEncoders() and read by the
// loop.
// The sampling is NOT punctual even though it is requested every MS_MUESTRA:
// it shares the I2C bus with the ToF, which hold it for 20-30 ms at a time on
// every loop iteration, so between two samples 20 ms or 45 may pass. While
// the loop ran at 100 ms it did not matter -- five samples fitted in each
// window and the distribution averaged out -- but at 25 ms ONE fits, and
// sometimes none or two. Dividing by the loop period instead of by the real
// time turns that distribution into 100% jumps of measured speed, which the
// PID chases: that is the "intermittent trajectory" of Sep 14.
uint32_t usUltimoMuestreo = 0;
bool     hayMuestraNueva  = false;

void muestrearEncoders(){
    for(int i=0;i<N_RUEDAS;i++){
        Rueda *r = &ruedas[i];
        // An immediate retry: the motors starting inject noise on the bus and
        // a single NACK does not mean the encoder is not there.
        int angulo; bool leido = false;
        for(int intento=0; intento<2 && !leido; intento++){
            if(!tcaAbrirRapido(r->canalMux)) continue;
            leido = leerRegistroAS(REG_ANGULO_AS, 2, &angulo);
            tcaCerrar();
        }
        if(!leido){ r->vivo=false; r->fallosI2C++; continue; }
        angulo &= 0x0FFF;
        r->vivo = true;

        if(r->anguloPrevio >= 0){
            int delta = angulo - r->anguloPrevio;      // jump from 4095 to 0
            if(delta >  2048) delta -= 4096;
            if(delta < -2048) delta += 4096;
            float grados = r->signoEncoder * delta * 360.0f / 4096.0f;
            r->gradosVentana += grados;
            r->gradosTramo   += grados;
        }
        r->anguloPrevio = angulo;
    }
    // 🔑 TIMESTAMP OF THE WINDOW. The loop divides the accumulated degrees by
    // the time they really took to accumulate, not by its own period. See
    // corregirVelocidad().
    usUltimoMuestreo = micros();
    hayMuestraNueva = true;
}

// The AS5600 says whether it sees the magnet and with how much gain. It is
// queried slowly: it does not change fast and it takes the bus that the
// encoder counts need.
void muestrearSalud(){
    for(int i=0;i<N_RUEDAS;i++){
        Rueda *r = &ruedas[i];
        if(!tcaAbrirRapido(r->canalMux)){ tcaCerrar(); continue; }
        int estado=0, agc=0;
        bool ok = leerRegistroAS(REG_ESTADO_AS, 1, &estado)
               && leerRegistroAS(REG_AGC_AS, 1, &agc);
        tcaCerrar();
        if(!ok) continue;
        r->iman = (estado & 0x20) != 0;   // MD: magnet present
        r->agc  = agc;
    }
}

void iniciarTramo(){
    for(int i=0;i<N_RUEDAS;i++) ruedas[i].gradosTramo = 0.0f;
}
// Centimeters traveled in the stretch. The FOUR live wheels are averaged: a
// single reference wheel (what the modular firmware did) lies as soon as
// that wheel slips, and here they slip.
float distanciaTramoCm(){
    float suma=0; int n=0;
    for(int i=0;i<N_RUEDAS;i++){
        if(!ruedas[i].vivo) continue;
        suma += fabsf(ruedas[i].gradosTramo); n++;
    }
    if(!n) return 0.0f;
    float grados = suma / n;
    return (grados/360.0f) * (2.0f*PI*R_RUEDA) * 100.0f * FACTOR_ODOM;
}
int celdasTramo(){
    return (int)(distanciaTramoCm() / PASO_CELDA_CM + 0.5f);
}

// ######################################################################
// BLOCK 12 -- MOTORS AND KINEMATICS
// WHAT IT DOES: translates (body speed) -> (rpm per wheel), and writes the
//   PWM with the direction that corresponds to each H-bridge.
// IMPORTANCE: CRITICAL. The kinematics go in their natural direction: no
//   sign is left patching them, because the root cause (the polarity of the
//   four bridges) is fixed where it belonged.
// ######################################################################
void aplicar(int i){
    Rueda *r = &ruedas[i];
    if(!habilitado || r->desbocada || fabsf(r->objetivo) < 1.0f){
        digitalWrite(r->in1, LOW);
        digitalWrite(r->in2, LOW);
        ledcWrite(r->canalPwm, 0);
        return;
    }
    bool adelante = r->objetivo > 0;
    bool sentido  = r->invertirMotor ? !adelante : adelante;
    digitalWrite(r->in1, sentido ? HIGH : LOW);
    digitalWrite(r->in2, sentido ? LOW  : HIGH);
    int v = (int)(r->salida + 0.5f);
    ledcWrite(r->canalPwm, constrain(v, 0, PWM_MAX));
}
// Moves a motor WITHOUT going through the control. Only for the direction
// calibration, and only with the robot IDLE.
void moverCrudo(int i, int pwm, bool adelante){
    Rueda *r = &ruedas[i];
    bool sentido = r->invertirMotor ? !adelante : adelante;
    digitalWrite(r->in1, sentido ? HIGH : LOW);
    digitalWrite(r->in2, sentido ? LOW  : HIGH);
    ledcWrite(r->canalPwm, constrain(pwm, 0, PWM_MAX));
}
void frenarTodo(){
    for(int i=0;i<N_RUEDAS;i++){
        digitalWrite(ruedas[i].in1, LOW);
        digitalWrite(ruedas[i].in2, LOW);
        ledcWrite(ruedas[i].canalPwm, 0);
        ruedas[i].objetivo = 0;
        ruedas[i].integral = 0;
        ruedas[i].salida = 0;
        ruedas[i].suelo = 0;
    }
    ordenVx = ordenVy = ordenW = 0;
    pedidoVx = pedidoVy = pedidoW = 0;
}

// Mecanum kinematics in X configuration, COPIED from carro_v08 (the one in
// simulacion/rl/cinematica.py): from the body speed to each wheel's speed.
// The order is that of the wheel table: DI, DD, TI, TD.
//   vx > 0 forward; vy > 0 to the LEFT (the front wheels open and the rear
//   ones close: DI and TD subtract, DD and TI add); W > 0 left (right side
//   faster), the same convention as the yaw and ejecutarGiro.
// The three axes go in their natural direction because the bridge polarity
// is fixed in the table (invertirMotor), not here.
// 🔴 THIS DISTRIBUTION IS THE RIGHT ONE AND IS NOT TOUCHED. On Sep 14 the
// turn sign of DD and TI was changed as a test, because the data said those
// two measured the opposite of what was asked of them. With the change the
// four signs matched... and the robot turned WORSE: asking the front wheels
// for one direction and the rear ones for the other is not rotating, on
// mecanum it is translating sideways. So the distribution was right and what
// is swapped is SOMETHING ELSE (see the note at the end of the block).
// Reverted.
void repartir(float rpmObjetivo[4]){
    float v[4];
    v[0] = pedidoVx - pedidoVy - LSUM * pedidoW;   // DI
    v[1] = pedidoVx + pedidoVy + LSUM * pedidoW;   // DD
    v[2] = pedidoVx + pedidoVy - LSUM * pedidoW;   // TI
    v[3] = pedidoVx - pedidoVy + LSUM * pedidoW;   // TD
    for(int i=0;i<4;i++)
        rpmObjetivo[i] = v[i] / R_RUEDA * 60.0f / (2.0f*PI);
}

// The PWM this wheel's model asks for a given rpm. It is a line that goes
// through the measured point: at PWM 160, so many rpm.
// The PWM this wheel's model asks for a given rpm: a MEASURED line, with its
// intercept. See BLOCK 07: the line does NOT go through zero.
float pwmDelModelo(Rueda *r, float rpm){
    float pwm = r->pwmBase + r->pwmPorRpm * fabsf(rpm);
    return constrain(pwm, 0.0f, (float)PWM_MAX);
}
// The inverse: which rpm to expect from a given PWM. Below pwmBase the wheel
// does not move, so there it is zero.
float rpmDelModelo(Rueda *r, float pwm){
    if(pwm <= r->pwmBase) return 0.0f;
    return (pwm - r->pwmBase) / r->pwmPorRpm;
}

void limitar(float *v, float destino, float tope, float dt){
    float falta = destino - *v;
    float paso = tope * dt;
    if(falta >  paso) falta =  paso;
    if(falta < -paso) falta = -paso;
    *v += falta;
}
// The command jumps to the ramp. Without this a sudden change makes the four
// wheels grip at different instants and the robot jerks.
// 🔴 RISING AND FALLING ARE NOT THE SAME. Braking suddenly leaves each wheel
// stopping on its own -- with its inertia and its friction, which are not
// equal -- and the car veers right at the end of the motion: it is the "at
// the end the wheels do not keep the speed and it deviates" of Sep 14.
// Going down with a ramp, the loop keeps ruling during the whole braking
// and the four reach zero together.
// DECEL_MAX is tuned separately from ACEL_MAX ('#decel='): if the car
// overshoots at the end, raise it; if the end still veers, lower it.
static inline float topeRampa(float orden, float pedido){
    // It is braking when what is requested is SMALLER in magnitude than what
    // it already has; in any other case (starting, or reversing) the rising
    // ramp rules.
    return (fabsf(orden) < fabsf(pedido)) ? DECEL_MAX : ACEL_MAX;
}
// 🔴 THE RAMP DOES NOT START FROM ZERO, IT STARTS FROM VEL_ARRANQUE. The start
// pulse (see the floor in corregirVelocidad) puts the wheels at about 20 rpm
// in a tenth of a second, but at that instant the ramp still asks for 11.
// The wheel goes faster than requested, so the loop cuts its PWM: measured
// on Sep 14, the front right went from PWM 151 to 6 and to 3 for two tenths
// -- it turned, stopped, and then the four started. It happens to it and not
// to the others because with the same push it accelerates more, which is
// what a shorter gearbox does.
//
// 🔑 And that 0 to 20 rpm stretch is not lost: it is a zone in which the robot
// cannot work anyway -- the front right does not give a repeatable speed
// below 70 rpm (test 12). Skipping it is acknowledging what we already knew
// about the hardware.
const float VEL_ARRANQUE = 0.0911f;   // m/s -- about 30 rpm per wheel
// The same jump for the turn: the same 30 rpm per wheel, converted to rad/s
// with LSUM. The turn started with the same dip as driving -- the front
// right PWM dropped to 16 and the wheel slowed from 37.5 to 31.9 rpm.
const float W_ARRANQUE   = 0.6272f;   // rad/s -- 36 degrees/s

void rampa(float dt){
    float oVx = habilitado ? ordenVx : 0.0f;
    float oVy = habilitado ? ordenVy : 0.0f;
    // Start jump: if motion is requested and the setpoint was at zero, the
    // ramp starts where the pulse leaves the wheels, not at zero.
    if(fabsf(oVx) > 1e-3f && fabsf(pedidoVx) < 1e-3f)
        pedidoVx = (oVx > 0 ? VEL_ARRANQUE : -VEL_ARRANQUE);
    if(fabsf(oVy) > 1e-3f && fabsf(pedidoVy) < 1e-3f)
        pedidoVy = (oVy > 0 ? VEL_ARRANQUE : -VEL_ARRANQUE);
    limitar(&pedidoVx, oVx, topeRampa(oVx, pedidoVx), dt);
    limitar(&pedidoVy, oVy, topeRampa(oVy, pedidoVy), dt);
    float oW = habilitado ? ordenW : 0.0f;
    if(fabsf(oW) > 1e-3f && fabsf(pedidoW) < 1e-3f)
        pedidoW = (oW > 0 ? W_ARRANQUE : -W_ARRANQUE);
    limitar(&pedidoW,  oW, ACEL_GIRO, dt);
}

// ######################################################################
// BLOCK 13 -- PER-WHEEL SPEED LOOP
// PORTED WHOLE from carro_v08. Each piece is here because it fixed a
// specific symptom measured on the floor:
//
//   FEED-FORWARD -- the PWM the model asks for the CURRENT speed is applied
//     directly and the loop only corrects the difference. Without this the
//     operating point was built by the integral alone, at ~40 PWM points per
//     second: at a start the four wheels stayed stuck at 25 rpm for almost
//     two seconds while being asked to rise to 66. Speed error: from 16-41
//     rpm to 5.6-7.2.
//   DO NOT INTEGRATE WITH THE WHEEL STOPPED -- there the loop does not
//     correct speed, it overcomes friction. Integrating, the four
//     accumulated the equivalent of a PWM of 115 during the second in which
//     nothing moved and broke free suddenly with that excess on top. That was
//     the start lag, and it was not a wheel starting earlier.
//   STOP THRESHOLD AT 10 rpm (not 3) -- the encoder count noise reaches 5
//     with the wheel still.
//   JUMP TO THE TAKE-OFF PWM -- instead of ramping up from zero. The four
//     break free in the same control cycle: the lag between the first and
//     the last went from 1.2 s to zero.
//   RUNAWAY PROTECTION -- if the sign of an encoder is reversed, the loop is
//     POSITIVE feedback and the motor goes to the cap in less than a second.
//     It is not fixed with gains: the wheel is cut off.
//   THE LOOP WORKS IN MAGNITUDE -- the PWM goes from 0 to 255 and the
//     direction is set by the H-bridge. If the error carried the sign, asking
//     for reverse would give a negative output, clipped at the bottom, and
//     the wheel would not start.
// ######################################################################
float correccion = 0;          // rpm of difference between sides (heading loop)
bool  pedirGuardarDespegues = false;
int   ruedaCortada = -1;       // last one cut off, to warn the Pi

// 🔴 THIS LOOP IS THE carro_v08 ONE AS IS (Sep 14), with ONE exception. Of the
// three things that had been added to it on Sep 12 -- open loop with a mute
// encoder, automatic sign flip, and grace cycles when changing direction --
// the first two stay out: the one tested on the floor is this version, and
// a wrongly measured sign or polarity is measured with 'V', not patched on
// the fly.
//
// 🔑 THE GRACE WHEN CHANGING DIRECTION DOES STAY, and not because of the flip
// (which no longer exists) but because of the CUT-OFF. `rpm` is an
// exponential moving average filter with ALFA=0.4 that feeds back on itself:
// after a turn, a wheel that was at -50 rpm and stops dead keeps MEASURING
// -30, -18, -10.8, -6.5 for four cycles, because the filter drags its own
// past. The cut-off requires five consecutive cycles below -10 rpm: the
// margin is ONE cycle. A forward start right after a turn falls within that
// margin and cuts healthy wheels -- it is the "not all the wheels start" of
// the Sep 12 audit, and the stall at 900 ms that comes after it.
// Forgiving the first cycles of the new direction, a truly reversed encoder
// is still detected: it takes 1 s instead of 0.5.
// 🔴 THESE TWO ARE COUNTED IN CYCLES, SO THEY DEPEND ON MS_CONTROL. They were
// written as "5", which with a 100 ms cycle was half a second; when the loop
// went down to 25 ms that same 5 would have become 125 ms and the protection
// would cut healthy wheels at the slightest jerk. They are declared in
// MILLISECONDS and the cycles are derived: this way the next person who
// touches MS_CONTROL does not have to remember this.
const int MS_GRACIA_MARCHA = 500;   // when changing direction there is no suspicion
const int MS_CORTE         = 500;   // measuring backwards before cutting off
const int CICLOS_GRACIA_MARCHA = MS_GRACIA_MARCHA / MS_CONTROL;
const int CICLOS_CORTE         = MS_CORTE / MS_CONTROL;

// 🔴 THE TURN HAS ITS OWN GRACE, SEPARATE FROM THE DRIVING ONE (Sep 14).
// In the turn the wheels on one side go forward and those on the other go
// backward, so as soon as one pair starts before the other the chassis
// starts rotating and DRAGS along the floor the ones that have not yet
// broken free -- in the direction of the rotation, which for them is the
// opposite of what is asked. Measured in the Sep 14 batch: DI and TD
// started first, DD and TI were dragged backwards, and the protection cut
// them off at 0.8 and 0.9 s. With two wheels out, the turn was done by the
// DI-TD diagonal, which on a mecanum chassis TRANSLATES besides rotating:
// "it turns but it is drifting".
// That initial drag is normal and goes away by itself; what it cannot do is
// cost the robot a wheel. During the turn more margin is given before
// cutting off.
// 🔑 THIS DOES NOT TOUCH STRAIGHT DRIVING: there MS_CORTE still rules, which
// is the number that was already right.
const int MS_CORTE_GIRO    = 1600;  // margin before cutting off, while turning
const int CICLOS_CORTE_GIRO = MS_CORTE_GIRO / MS_CONTROL;
// Set and cleared by ejecutarGiro(). It is declared here, before the loop,
// because it is the loop that queries it (estadoLoco lives further down, in
// BLOCK 17).
volatile bool girando = false;

void sembrarIntegral(Rueda *r){ r->integral = 0.0f; }

// Encoder window closed by the previous sampling, to measure the REAL time
// the degrees took to accumulate.
uint32_t usVentanaPrev = 0;

void corregirVelocidad(unsigned long dt){
    float dtSeg = dt / 1000.0f;
    if(dtSeg < 0.001f) dtSeg = MS_CONTROL / 1000.0f;

    // 🔑 THE TIME OF THE MEASUREMENT WINDOW IS NOT THE LOOP PERIOD.
    // `dtSeg` is used for the integral and the derivative, which belong to
    // the loop. But the rpm come from degrees accumulated by the ENCODER, and
    // those took what they took: one has to divide by the real interval
    // between the sampling that closed the previous window and the one that
    // closes this one.
    uint32_t usAhora = usUltimoMuestreo;
    float dtVentana = (usAhora - usVentanaPrev) / 1e6f;   // unsigned subtraction: the
                                                          // micros() overflow
                                                          // comes out right
    bool medidaFresca = hayMuestraNueva && dtVentana > 0.004f;
    if(medidaFresca){ usVentanaPrev = usAhora; hayMuestraNueva = false; }

    float base[4];
    repartir(base);

    for(int i=0;i<N_RUEDAS;i++){
        Rueda *r = &ruedas[i];

        // It is divided by dtSeg, which already comes with a floor: with dt at
        // zero and the window at zero this was 0/0 and the rpm stayed NaN
        // forever because the filter feeds them back.
        if(!r->vivo) continue;   // no reading, the PWM is left as it was

        // Without a new measurement `rpm` is NOT touched: the last good one is
        // kept. Before, this computed 0 rpm (the window was empty) and the
        // loop read it as "the wheel stopped", adding a PWM kick that was not
        // needed.
        if(medidaFresca){
            float rpmCruda = r->gradosVentana / dtVentana / 6.0f;
            r->gradosVentana = 0.0f;
            r->rpm = ALFA*rpmCruda + (1.0f-ALFA)*r->rpm;
        }
        // The correction is extra turn rpm: it is distributed with THE SAME
        // signs as the turn row of the kinematics.
        r->objetivo = base[i] + (r->derecha ? correccion : -correccion);

        // The loop works in MAGNITUDE: the PWM goes from 0 to 255 and the
        // direction is set separately by the H-bridge.
        float sentido = (r->objetivo >= 0) ? 1.0f : -1.0f;
        float objMag  = fabsf(r->objetivo);
        float rpmSentido = r->rpm * sentido;

        if(!habilitado || r->desbocada || objMag < 1.0f){
            r->objetivo=0; r->integral=0; r->salida=0; r->suelo=0;
            r->sospechas=0; r->rpmPrevia=rpmSentido;
            continue;
        }

        if((int)sentido != r->sentidoPrevio){   // when changing direction
            sembrarIntegral(r);
            r->rpmPrevia = rpmSentido;
            // The first cycles of the new direction do NOT count as suspicion:
            // what the filter measures is still the previous direction. See
            // above.
            r->sospechas = -CICLOS_GRACIA_MARCHA;
        }
        r->sentidoPrevio = (int)sentido;

        // Runaway protection. If the encoder sign is reversed, the error comes
        // out huge and of the opposite sign: it is POSITIVE feedback and it is
        // not fixed with gains. The wheel is cut off when it has been
        // measuring the opposite of what is asked for half a second, or shot
        // up far above. The "shot up" threshold takes the wheel's own floor
        // into account: comparing only against what is asked punishes the
        // fastest one, which does not go below its ~54 rpm.
        float pwmSuelo = fmax(r->suelo, r->pwmDespegue*FACTOR_MANTIENE);
        float rpmSuelo = rpmDelModelo(r, pwmSuelo);
        bool alReves   = rpmSentido < -10.0f;
        bool disparada = rpmSentido > fmax(2.5f*objMag, 1.6f*rpmSuelo) + 40.0f;
        if(alReves || disparada){
            // The limit depends on what the robot is doing: turning, the
            // initial drag is expected (see above); driving, it is not.
            int limite = girando ? CICLOS_CORTE_GIRO : CICLOS_CORTE;
            if(++r->sospechas >= limite){
                Serial2.printf("WHEEL %s CUT OFF: %.0f rpm were requested and it measures %.0f\n",
                               r->corto, r->objetivo, r->rpm);
                r->desbocada = true; ruedaCortada = i;
                r->objetivo=0; r->integral=0; r->salida=0;
                continue;
            }
        } else r->sospechas = 0;

        float error = objMag - rpmSentido;

        // 🔴 THE FLOOR ONLY PUSHES THE WHEEL THAT IS SLOW (kept), BUT IT IS
        // NOT BOUNDED (reverted on Sep 14). Bounding it to the model PWM
        // seemed like a good idea -- it prevented a wheel saturated at the
        // bottom from ignoring the loop -- but at low speed the ceiling fell
        // below the take-off PWM and the wheel did not start: that broke the
        // 90-degree turn, which works exactly at those speeds. The turn was
        // fine before and stays as it was.
        const float RPM_PARADA = 10.0f;
        bool parada = fabsf(r->rpm) < RPM_PARADA;

        // 🔴 THE FLOOR CEILING IS NOT APPLIED TO THE STOPPED WHEEL.
        // They are two different jobs with the same number: with the wheel
        // still the floor has to BREAK THE FRICTION, and there the whole
        // take-off PWM is needed; with the wheel already turning, the floor
        // only gets in the way, because it imposes a minimum speed the loop
        // cannot lower.
        // Bounding it also while stopped was what broke the 90-degree turn on
        // Sep 14: at the end of the turn the command drops to the minimum
        // speed (25 degrees/s, about 21 rpm), and with such a low target the
        // ceiling ended up BELOW the DD take-off (42 against 45.7), so DD did
        // not start while the other three did -- "it does not move all the
        // wheels, only some".
        // 🔴 START PULSE: WITH THE WHEEL STILL, THE FLOOR IS THE WHOLE TAKE-OFF
        // PWM, not its 55%. It is what the comment above already said and the
        // code did not do: when still one has to BREAK THE FRICTION.
        //
        // What this fixes: each wheel breaks free when the start ramp reaches
        // ITS threshold, and the four thresholds are different. Measured on
        // Sep 14: the front right broke free at 107 ms and the other three at
        // 317 -- a 210 ms lag, visible from outside as "they do not all start
        // at once". Giving them their whole take-off from the first cycle,
        // the four break free together and each with what it needs, which is
        // not the same for all (113, 121, 127 and 151).
        //
        // As soon as the wheel moves it stops being `parada` and the floor
        // drops to 55%, so the pulse lasts just enough. The hard cap is still
        // DESPEGUE_MAX.
        float suelo = fmax(r->suelo, parada ? r->pwmDespegue
                                            : r->pwmDespegue*FACTOR_MANTIENE);

        // 🔴 WITH THE WHEEL ALREADY TURNING, THE FLOOR CANNOT EXCEED THE PWM
        // THE MODEL ASKS FOR. The floor comes from
        // `pwmDespegue*FACTOR_MANTIENE`, and that 0.55 is global: it assumes
        // that starting and keeping going scale the same way on the four
        // wheels. On the front right they do NOT scale the same -- short
        // gearbox, it is hard to break free and takes almost nothing to keep
        // going -- so its take-off rose to 151 and its floor ended up at 83
        // while it works with 60.
        //
        // What was seen (Sep 14, driving at 77 rpm): its PWM alternated
        // between 58 and 83 cycle by cycle, with no values in between. It
        // falls a hair below the target, the floor catapults it to 83, it
        // overshoots, the floor stops applying, it drops to 58, it slows down,
        // and it starts again. It is a limit cycle, and from outside it looks
        // like a wheel turning in jerks even though its MEAN speed is right.
        // On the other three the floor stays between 59 and 69 with working
        // PWMs of 127 to 135: it does not touch them.
        //
        // 🔑 THIS WAS ALREADY TRIED ON Sep 14 AND IT BROKE THE TURN, and now it
        // is different: back then the model was a line through the origin and
        // at low speed its PWM dropped almost to zero, so the ceiling left
        // the wheels unable to take off. Now three of the four have a line
        // with an intercept and never go below it. The front right still has
        // a zero intercept, so at low speeds its ceiling is small again --
        // one more reason to measure its line again (test 12), and it has to
        // be kept in mind when touching the turn, which goes down to 21 rpm.
        //
        // With the wheel STOPPED it is not bounded: there the floor has
        // another job, breaking the friction, and the whole take-off PWM is
        // needed.
        // 🔴 AND ONLY WITH THE TARGET ALREADY IN THE WORKING RANGE. During the
        // start ramp the target goes through low values, and there the front
        // right model -- which has a ZERO intercept, see BLOCK 07 -- asks for
        // almost nothing: bounding the floor with that cut its PWM to 15
        // right after breaking free and the wheel slowed down again.
        // Measured (Sep 14): it broke free at 54 ms with PWM 83, the bound
        // left it at 15, it dropped from 15.9 to 9.5 rpm and did not recover
        // until 425 ms. That is the "accelerates, slows down, accelerates"
        // seen from outside.
        if(!parada && objMag >= RPM_RANGO_TRABAJO)
            suelo = fmin(suelo, pwmDelModelo(r, objMag));

        // 🔑 AND NO TAKE-OFF IS SEARCHED WHILE LITTLE IS REQUESTED. The
        // searcher raises the floor while the wheel is still, assuming it
        // does not take off. But with the start ramp the wheel is still at the
        // beginning BECAUSE ALMOST NOTHING IS ASKED OF IT, not because it
        // lacks push: the searcher spent that stretch rising, and that is
        // where the 170 came from. It only searches when a speed is requested
        // that the wheel should already be giving.
        bool buscando = parada && objMag > RPM_MIN_BUSQUEDA;

        // 🔴 THERE WAS A BOTTOM ANTI-WINDUP HERE WITH THE CONDITION REVERSED,
        // and it left a wheel permanently turning above what was requested.
        // It said: freeze the integral if the output is at the floor AND the
        // error is negative. But `error = objMag - rpmSentido`, so error < 0
        // is exactly "the wheel goes FASTER than requested"... which is the
        // only case in which the floor is NOT applied (see the end of this
        // function: the floor only pushes the slow one). So it froze the
        // integral exactly when the output was not limited by anything, and
        // the integral stayed up without being able to come down.
        //
        // What was seen (Sep 14, driving at 76.5 rpm): DD settled at 94.2 rpm
        // with the PWM stuck at 78.5, when its model only asked for 64.4. Its
        // take-off PWM had risen to 151 -- it is the short-gearbox wheel, it is
        // hard to break free -- and 151*0.55 = 83 was above what it needs to
        // keep going. The output fell below that 83, the integral froze, and
        // the wheel stayed there.
        //
        // 🔑 The case that anti-windup meant to cover DOES NOT EXIST: for the
        // floor to really limit, the wheel has to be SLOW, and then the error
        // is positive and the integral must rise, not fall.
        // The real windup is the top one (output at PWM_MAX with a positive
        // error), and the integral's constrain takes care of that.
        //
        // NO integration with the wheel stopped: there the loop does not
        // correct speed, it overcomes friction (see BLOCK 13). That IS kept.
        if(!parada)
            r->integral += error * (dtSeg*1000.0f/MS_CONTROL);
        if(KI > 0.0001f){
            float tope = PWM_MAX / KI;
            r->integral = constrain(r->integral, -tope, tope);
        }

        float derivada = (rpmSentido - r->rpmPrevia) / dtSeg;
        r->rpmPrevia = rpmSentido;

        float anticipa = pwmDelModelo(r, objMag);
        r->salida = anticipa + KP*error + KI*r->integral - KD*derivada;

        // 🔴 PER-WHEEL THRESHOLD: NONE CAN GO FAR ABOVE WHAT IS ASKED OF IT
        // (Sep 14, on request). Until now the only limit was the maximum PWM,
        // so when the loop got it wrong -- or when the heading correction
        // loaded one side heavily -- a wheel could shoot up to the cap while
        // the others went at their pace: that is the wheel that "spins out of
        // control to compensate".
        // The ceiling is computed from WHAT IS REQUESTED, not from a fixed
        // number: it is the PWM the model needs for that speed, with margin
        // so the loop can really correct. It never goes below the wheel's
        // take-off PWM, because otherwise a stopped wheel could not start.
        float techo = fmax(r->pwmDespegue,
                           pwmDelModelo(r, objMag)*FACTOR_TECHO + MARGEN_TECHO);
        if(r->salida > techo) r->salida = techo;
        r->salida = constrain(r->salida, (float)PWM_MIN, (float)PWM_MAX);

        // Take-off PWM searcher.
        if(buscando){
            if(r->suelo < 1.0f){
                r->suelo = fmax(20.0f, r->pwmDespegue);
                r->pasosBusqueda = 0;
            } else {
                r->suelo += PASO_BUSQUEDA;
                r->pasosBusqueda++;
            }
            if(r->suelo > DESPEGUE_MAX){
                r->suelo = DESPEGUE_MAX;
                // 🔴 IT HAS ALREADY BEEN GIVEN ALL THE PWM THE TABLE ALLOWS AND
                // IT DOES NOT MOVE. That is not start friction: it is a wheel
                // that CANNOT turn -- the car against a wall, a jammed wheel, a
                // disconnected motor. Rising further will not free it, and
                // "learning" that number poisons the following starts: on
                // Sep 14 the four ended up with 170 saved after the car hit
                // something while reversing, and from then on the PWM floor
                // prevented the loop from lowering the speed of any wheel.
                r->pasosBusqueda = 0;   // so that this value is NOT recorded
            }
        } else if(r->suelo > 0.0f){
            float antes = r->pwmDespegue;
            // The FLOOR that freed it is recorded, not the loop output: the
            // output contains the proportional term, which has nothing to do
            // with the start friction. And only if it is credible: recording
            // anything made three wheels "learn" that they take off with PWM
            // 12, 15 and 17 from a noise reading.
            if(r->pasosBusqueda > 0){
                if(r->suelo >= 20.0f) r->pwmDespegue = fmin(r->suelo, DESPEGUE_MAX);
            } else {
                r->pwmDespegue = fmax(20.0f, r->pwmDespegue - BAJA_TANTEO);
            }
            r->suelo = 0.0f;
            if(fabsf(r->pwmDespegue - antes) > 4.0f) pedirGuardarDespegues = true;
        }
        // 🔴 THE FLOOR ONLY PUSHES THE WHEEL THAT IS SLOW.
        // Measured on Sep 14, stretch 5: the left wheels were asked for 21.9
        // rpm and turned at 31.7. Those low rpm were precisely what the
        // heading correction was asking of them to straighten the car -- and
        // the floor prevented it. The correction was computed right,
        // distributed right, and then did not reach the wheels.
        // The floor exists so that a stopped wheel breaks the friction; to one
        // already turning FASTER than requested it has nothing to add, and
        // forcing it there takes away the loop's only way to slow it down.
        // Same criterion as above: stopped, the whole floor to take off;
        // already turning, bounded so it does not impose a minimum speed.
        // Same start pulse as above, see the block there.
        suelo = fmax(r->suelo, parada ? r->pwmDespegue
                                      : r->pwmDespegue*FACTOR_MANTIENE);
        // Same bound as above, see the block there.
        // Same criterion as above, see the block there.
        if(!parada && objMag >= RPM_RANGO_TRABAJO)
            suelo = fmin(suelo, pwmDelModelo(r, objMag));
        // 🔑 THE ONLY THING KEPT FROM THAT CHANGE: the floor only pushes the
        // wheel that goes SLOWER than requested. To one already turning too
        // fast it has nothing to add, and forcing it there took away the
        // loop's only way to slow it down -- that is what prevented correcting
        // the heading.
        // The CEILING I added on top (bounding it to the model PWM) is
        // removed: it broke the take-off at low speed and with it the turn
        // got spoiled.
        if(r->salida < suelo && rpmSentido < objMag) r->salida = suelo;
    }
}

// ######################################################################
// BLOCK 14 -- BNO085 IMU + HEADING LOOP
// WHAT IT DOES: takes the ALREADY FUSED heading delivered by the BNO085,
//   unwraps it into a continuous angle, and runs the PD that keeps the
//   heading of the stretch.
//
// WHY THERE IS NO LONGER A KALMAN FILTER OR BIAS (ported from carro_v08,
//   Sep 2): the MPU6050 gave the raw turn rate and the heading had to be
//   integrated here, subtracting a bias measured at every boot because it
//   changes with temperature -- uncorrected, the heading drifted 34 degrees
//   per minute. The BNO085 carries the processor that does that fusion and
//   delivers the finished heading. `biasGyroZ` is kept ONLY as a diagnostic
//   (the drift seen at rest): it shows up in READY and in 'S' to see at a
//   glance whether the sensor is healthy, but it is NOT subtracted from
//   anything.
//
// 🔑 THE HEADING IS REQUESTED WITHOUT MAGNETOMETER (the "game rotation
//   vector"). The other mode references it to magnetic north, and this car
//   has four brushed motors hanging from the same 5 V rail: the field they
//   see is not the Earth's. Without magnetometer there is no absolute north,
//   but the control does not need it -- it only defends the heading it had.
//
// 🔴 THE HEADING LOOP DOES NOT ACT WITH THE ROBOT STOPPED, and the reference
//   sticks to the current heading. Without this, at the end of a stretch the
//   robot kept correcting indefinitely: the requested speed drops to zero
//   but the heading error does not, so the correction stayed at its cap with
//   the wheels turning in opposite directions. A heading loop is useful
//   while the robot advances, not as a parking brake.
//
// 🔑 THE HEADING IS NOT FORGOTTEN WHEN STOPPING (carry-over): the pending
//   error is corrected in the NEXT stretch, while rolling, where the loop
//   does have authority because the wheels are already turning.
//   Straightening on the spot was tried, measured and DISCARDED: one stop
//   came in with +1.3 degrees and reached -7.2 before settling. The smallest
//   motion this chassis can make is larger than the error one wanted to
//   correct.
// ######################################################################
BNO080 bno;
uint8_t dirImu = 0;               // the address that really answered
const uint16_t MS_REPORTE = 20;   // how often the sensor sends its reports

float rollActual=0.0f;     // from the sensor quaternion; they only go to the CSV
float pitchActual=0.0f;
float yaw = 0.0f;          // continuous (unwrapped) heading, sensor convention
float yawMedido = 0.0f;    // the same, already in the control convention
float yawRef = 0.0f;       // the heading the loop defends
// --- the heading grid (Sep 15, afternoon) -----------------------------------
// yawBase is the heading (in the control convention, continuous) that was set
// with 'Z' at the start of the run: the North of the maze. cuartos are the
// 90-degree turns accumulated since then (+ = right, like the Pi's pasos90).
// The heading that corresponds at each moment is rumboCuadricula(), and it
// does NOT depend on how any turn came out: that is why the error of the
// turns stops accumulating.
float yawBase = 0.0f;
int   cuartos = 0;
float TOL_ALINEAR_DEG = 4.0f;   // 'Y' considers it aligned within this ('#tolalin=')
float TOL_AVANCE_DEG  = 6.0f;   // '5' refuses beyond this ('#tolavance=')
bool  huboGiroCuadricula = false;  // the last turn was a grid turn (not manual)
// 🔴 WHILE THE GRID IS ACTIVE ('Z' turns it on, the manual control turns it
// off) THE HEADING REFERENCE NEVER ADOPTS THE MEASURED ONE: neither through
// the 25-degree carry-over at rest nor after a failed turn. Sep 15 night:
// the robot ended up going diagonally, defending a tilted heading that the
// reference had copied from the sensor. With this, if it is tilted, the '5'
// is rejected (misaligned) and the Pi aligns it with 'Y' before advancing.
bool  cuadriculaActiva = false;
static inline float rumboCuadricula(){ return yawBase - cuartos * 90.0f; }

// --- ADVANCE AND CENTERING: THE NUMBERS OF square_right.py (Sep 15 15:47) ----
// 🔑 TRANSFERRED AS IS from the script that closed two laps with 0.21 degrees
// of error (bench_tests/14_square_circuit/best_right_2026-09-15_1547). There
// they lived on the Pi sending manual pushes; here they do the same inside
// the cell advance. Each constant carries the name from the script.
//   V_CRUCERO 102/255 of vmanual 0.30 = 0.12 m/s   V_MINIMA 30/255 = 0.035
//   T_RAMPA_INI 1.5 s   EMPIEZA_FRENAR 330   LATENCIA (smaller here: no network)
//   taps: VY_MIN_EFECT 40/255 = 0.047 m/s, BANDA_LAT 5, ERR_PLENO 18,
//   SESGO_LAT 3.8, MIN_LATERAL 32 -> VY_URGENTE 45/255 = 0.053,
//   CONSIGNA_E 52.5, CONSIGNA_O 43.0, SIN_TRASLADAR 150, LIMITE_LATERAL 140.
//   Stopped re-centering after the turn: VY_PEGAR 65/255 = 0.076 m/s, pulse
//   ms = |err|*6 bounded 35..250, INERCIA_LAT 12, BANDA_DER 5, TOPE_PEGAR 9 s.
// 🔴 "A SMALL vy DOES NOT MOVE THE ROBOT": the strength of the tap is fixed
// (the minimum executable one) and what is modulated is HOW OFTEN it is
// applied. A proportional with a large vy gives shoves and the robot goes in
// a zigzag (measured on Sep 15, and again tonight).
float CENTRAR_ACTIVO   = 1.0f;     // '#centrar=0' turns the taps off
float V_MINIMA_MS      = 0.035f;   // V_MINIMA
float T_RAMPA_INI_S    = 1.5f;     // T_RAMPA_INI
float EMPIEZA_FRENAR_MM= 330.0f;   // EMPIEZA_FRENAR (start braking)
float LATENCIA_FRENO_S = 0.20f;    // LATENCIA (0.35 with the network; here the ToF alone)
float VY_TOQUE_MS      = 0.047f;   // VY_MIN_EFECT
float BANDA_LAT_MM     = 5.0f;     // BANDA_LAT
float ERR_PLENO_MM     = 18.0f;    // ERR_PLENO
float SESGO_LAT_MM     = 3.8f;     // SESGO_LAT (dE-dO while centered)
float MIN_LATERAL_MM   = 32.0f;    // MIN_LATERAL
float VY_URGENTE_MS    = 0.053f;   // VY_URGENTE
float CONSIGNA_E_MM    = 52.5f;    // CONSIGNA_E (setpoint E)
float CONSIGNA_O_MM    = 43.0f;    // CONSIGNA_O (setpoint W)
float LIMITE_LATERAL_MM= 140.0f;   // LIMITE_LATERAL: farther = there is no wall
float SIN_TRASLADAR_MM = 150.0f;   // SIN_TRASLADAR
const uint32_t TOQUE_MS = 100;     // script PERIOD: a tap lasts one cycle
float PEGAR_ACTIVO     = 1.0f;     // '#pegar=0' removes the re-centering after the turn
float VY_PEGAR_MS      = 0.076f;   // VY_PEGAR
float INERCIA_LAT_MM   = 12.0f;    // INERCIA_LAT
float BANDA_PEGAR_MM   = 5.0f;     // BANDA_DER
float TOPE_PEGAR_S     = 9.0f;     // TOPE_PEGAR
float errCentro = 0.0f;            // last lateral error, to the CSV
float acumToque = 0.0f;            // the accumulator of _toque()
uint32_t toqueHasta = 0;           // until when the current tap lasts
float vyToque = 0.0f;
unsigned long tInicioTramo = 0;
unsigned long tCicloAnterior = 0;  // to scale the accumulator to the real cycle
float yawCrudo = 0.0f;     // the last thing the sensor gave, which wraps around at +-180
bool  hayYawPrevio = false;
float gzFiltrado = 0.0f;   // degrees/s, for the derivative term
float integralYaw = 0.0f;
float biasGyroZ = 0.0f;    // drift at rest, ONLY diagnostic (see header)
bool  imu_ok = false;
uint32_t reiniciosImu = 0; // times the sensor reset by itself
uint32_t saltosImu = 0;    // impossible heading jumps, discarded

// How much heading can really change between two reports. At 20 ms, 45
// degrees are 2250 degrees/s, which this chassis cannot do even if thrown.
// Anything beyond that is a sensor discontinuity, not a turn.
const float SALTO_IMPOSIBLE = 45.0f;

// Above this the heading carry-over is forgotten: if someone moves the robot
// by hand, it cannot be spent in a start jolt.
const float RUMBO_ARRASTRE_MAX = 25.0f;

// The reports requested from the sensor. They are in a function because they
// have to be requested again every time the BNO085 resets by itself (see
// actualizarIMU).
void pedirInformesIMU(){
    bno.enableGameRotationVector(MS_REPORTE);   // fused heading, without compass
    bno.enableGyro(MS_REPORTE);                 // calibrated turn rate, for the D of the loop
}
bool inicializarIMU(){
    tcaCerrar();
    // Both possible addresses of the module are tried instead of assuming one.
    for(uint8_t dir = DIR_IMU_A; dir <= DIR_IMU_B; dir++){
        if(bno.begin(dir, Wire)){
            dirImu = dir;
            pedirInformesIMU();
            delay(100);
            return true;
        }
    }
    // If it does not answer at either: if the module has PS0/PS1 jumpers,
    // they have to be set for I2C; in UART or SPI nothing answers here.
    return false;
}
void actualizarIMU(float dt){
    (void)dt;   // the heading arrives finished: nothing is integrated. The
                // signature is kept because the caller still measures the real cycle.
    if(!imu_ok) return;
    tcaCerrar();

    // 🔴 The BNO085 resets by itself every now and then. When it does, its
    // heading goes back to zero, and two things must be done or the robot
    // goes crazy: (1) request the reports again, because the reset erases
    // them and otherwise it goes mute with the heading frozen; (2) do NOT
    // accumulate that zero: the heading we had is good, what was lost is the
    // sensor's internal reference. It is re-anchored on its new zero leaving
    // `yaw` where it was. If the jump were accumulated, the loop would
    // suddenly see an error the size of the whole heading and would send the
    // correction to the cap.
    if(bno.hasReset()){
        reiniciosImu++;
        pedirInformesIMU();
        hayYawPrevio = false;
    }

    // Whatever has arrived is drained, but with a cap: if the sensor gets
    // stuck, this lives inside the control loop and cannot be left unbounded.
    bool nuevo = false;
    for(int i=0; i<4 && bno.dataAvailable(); i++) nuevo = true;
    if(!nuevo) return;

    // Soft filter: the wheels vibrate and that vibration enters through the
    // gyroscope. Vibration is symmetric and averages out by itself, but
    // unfiltered the derivative term of the control would amplify it.
    float gz = bno.getGyroZ() * 57.29578f;
    if(isfinite(gz)) gzFiltrado = 0.3f*gz + 0.7f*gzFiltrado;

    // 🔴 The sensor has no heading until its first rotation report arrives,
    // and until then its quaternion is all zeros: the library normalizes it
    // by dividing by its own norm (0/0) and returns NaN. Since the heading
    // is accumulated, a single NaN poisons it forever. And the first report
    // that arrives is usually the gyroscope one, not the rotation one, so
    // this case happens on EVERY boot.
    float ahora = bno.getYaw() * 57.29578f;
    if(!isfinite(ahora)) return;
    float r = bno.getRoll()  * 57.29578f; if(isfinite(r)) rollActual  = r;
    float p = bno.getPitch() * 57.29578f; if(isfinite(p)) pitchActual = p;

    if(!hayYawPrevio){ yawCrudo = ahora; hayYawPrevio = true; return; }
    float salto = ahora - yawCrudo;
    if(salto >  180.0f) salto -= 360.0f;
    if(salto < -180.0f) salto += 360.0f;
    // Safety net in case a reset is not announced: an impossible jump is
    // discarded and re-anchored, instead of feeding it to the loop as if it
    // were real.
    if(fabsf(salto) > SALTO_IMPOSIBLE){ saltosImu++; yawCrudo = ahora; return; }
    yaw += salto;
    yawCrudo = ahora;
}
// With the robot STILL: checks that the sensor delivers heading, measures the
// drift at rest (diagnostic) and sets the heading to zero. It calibrates
// nothing -- the BNO085 calibrates itself -- but it keeps the name because
// the bench command 'C' and the setup call it the same as before, and
// PASOS.md names it.
bool calibrarBiasGiroscopio(uint16_t n){
    if(!imu_ok) return false;
    double s=0; uint16_t validas=0;
    hayYawPrevio = false; yaw = 0; yawMedido = 0; yawRef = 0;
    for(uint16_t i=0;i<n;i++){
        actualizarIMU(0.005f);
        if(hayYawPrevio){ s += gzFiltrado; validas++; }
        delay(5);
    }
    if(validas < 20) return false;      // no heading arrived: the sensor is not healthy
    biasGyroZ = (float)(s/validas);     // degrees/s seen at rest
    // The boot itself counts as a reset (the sensor announces its reset after
    // the library wakes it up). That one is not interesting: the counter is
    // an alarm for resets WHILE the robot is moving.
    reiniciosImu = 0; saltosImu = 0;
    // Re-anchored at zero: yawCrudo stays at the last sensor reading and the
    // next jump is measured against it, so the heading starts at 0.
    yaw = 0; yawMedido = 0; yawRef = 0;
    yawBase = 0; cuartos = 0;          // the re-anchoring is also the base
    return true;
}

// The heading loop, COPIED from carro_v08 (Sep 14). Returns rpm of difference
// between sides. On Sep 12 it had been turned off "on request" so the robot
// would go with the speed PID alone; the firmware tested while driving has it
// on, so it is turned on again as is.
//
// 🔑 The pending heading is carried over only after going STRAIGHT, not after
// a turn. Measured in carro_v08 over two full laps: the car follows the
// reference with a lag of about 11 degrees and when released it owes
// between 6 and 11 that it never closes. Carrying that debt over would make
// the next straight stretch veer sideways right from the start. After a
// turn, the heading reached IS the right one: the reference sticks there and
// it starts clean.
bool huboGiro = false;

// 🔴 It waits for the car to stop TURNING, not for the wheels to stop. When
// released, the wheels stop in half a second but the chassis keeps sliding
// and turning for a couple more seconds. If the reference were decided right
// when the wheels stop, the degrees that arrive afterwards would be absorbed
// and nobody would ever see them.
const float         ASENTAR_GZ_QUIETO = 2.0f;   // degrees/s: below this, it no longer turns
const unsigned long ASENTAR_MS        = 500;    // and it also has to stay like that for a while
const unsigned long ASENTAR_TOPE_MS   = 4000;   // maximum time to settle
bool          esperandoParada = false;
unsigned long asentarHasta = 0, asentarQuietoDesde = 0;

void calcularCorreccion(float dt){
    yawMedido = SIGNO_YAW * yaw;
    if(!imu_ok){ correccion = 0; return; }

    // 🔴 With the car stopped the heading loop does NOT act. A heading loop
    // makes sense while the car advances, not as a parking brake: without
    // this, when released it kept correcting with the wheels turning in
    // opposite directions until the motors were turned off.
    bool quieto = fabsf(pedidoVx) < 1e-3f && fabsf(pedidoVy) < 1e-3f
                                          && fabsf(pedidoW)  < 1e-3f;
    if(!quieto){
        esperandoParada = false;
        if(fabsf(pedidoW) > 1e-3f) huboGiro = true;
    } else {
        unsigned long ahora = millis();
        correccion = 0;
        integralYaw = 0;

        if(!esperandoParada){
            esperandoParada = true;
            asentarHasta = ahora + ASENTAR_TOPE_MS;
            asentarQuietoDesde = 0;
        }
        bool ruedasQuietas = true;
        for(int i=0;i<N_RUEDAS;i++) if(fabsf(ruedas[i].rpm) > 8.0f) ruedasQuietas = false;
        bool asentado = ruedasQuietas && fabsf(gzFiltrado) < ASENTAR_GZ_QUIETO;
        if(!asentado) asentarQuietoDesde = 0;
        else if(!asentarQuietoDesde) asentarQuietoDesde = ahora;

        bool listo = asentarQuietoDesde && (ahora - asentarQuietoDesde) >= ASENTAR_MS;
        if(!listo && ahora < asentarHasta) return;

        // Settled. The reference does NOT stick to the current heading: it is
        // kept so that whatever ended up tilted is straightened in the next
        // stretch, while rolling. It is released in two cases: if the stretch
        // had a requested turn (the heading reached is the right one) or if
        // the error went so far (moved by hand) that correcting it would be a
        // jolt.
        // 🔑 After a GRID turn, the right heading is not the one reached but
        // the grid one: whatever the turn left unclosed (< TOL_GIRO_DEG) is
        // straightened while rolling in the next stretch, which is exactly
        // what made the error stop accumulating in the Sep 15 circuits. A
        // manual turn (control) stays as before.
        if(huboGiro){
            yawRef = huboGiroCuadricula ? rumboCuadricula() : yawMedido;
            huboGiro = false; huboGiroCuadricula = false;
        }
        if(!cuadriculaActiva && fabsf(yawRef - yawMedido) > RUMBO_ARRASTRE_MAX)
            yawRef = yawMedido;
        if(cuadriculaActiva) yawRef = rumboCuadricula();
        return;
    }

    // If a turn is requested, the heading reference advances with it; if not,
    // it stays where it was and the loop defends it. This way the same loop
    // serves to go straight and to turn a specific amount.
    yawRef += pedidoW * 180.0f / PI * dt;
    float error = yawRef - yawMedido;

    float sinLimitar = KP_YAW*error + KI_YAW*integralYaw
                       - KD_YAW*(SIGNO_YAW*gzFiltrado);

    // 🔑 The authority grows with speed. The loop starts each stretch with a
    // carried-over error to correct; if it could use its whole cap with the
    // car almost stopped, that start would be a jerk. Tied to speed, the
    // correction comes in by itself as the car picks up speed.
    float vPedida = fabsf(pedidoVx) + fabsf(pedidoVy) + LSUM*fabsf(pedidoW);
    float rpmPedidas = vPedida / R_RUEDA * 60.0f / (2.0f*PI);
    // 🔴 THE CAP WAS 40% OF WHAT WAS REQUESTED, and that is not "correcting
    // the heading": it is another speed command on top of the one requested.
    // With the distribution below (one side +correction, the other
    // -correction), 40% leaves one side running at 140% and the other at 60%
    // -- and if the fast one hits the maximum PWM, the slow one has nothing
    // to compensate with and the car also loses forward speed. It is the
    // "the wheels do not keep the speed I set, it is compensating badly" of
    // Sep 14. With FRAC_CORR the maximum imbalance between sides is bounded
    // and the mean speed is respected.
    // 🔴 AT LOW SPEED, A PERCENTAGE DOES NOT GIVE ENOUGH AUTHORITY.
    // The cap was only FRAC_CORR times the requested rpm: when the speed was
    // halved, the cap went down with it to 6 rpm, and the correction spent
    // between 67% and 91% of the time SATURATED without managing to
    // straighten. The percentage still rules when the car goes fast -- there
    // it is what prevents the jerk -- but below that there is a fixed
    // minimum so that at low speed the heading still has something to work
    // with.
    float tope = fmin(CORR_MAX, fmax(CORR_MIN, FRAC_CORR*rpmPedidas));
    float deseada = constrain(sinLimitar, -tope, tope);

    // 🔴 AND THE CORRECTION NOW COMES IN THROUGH A RAMP, like the speed.
    // Before, it jumped to its new value in a single cycle while the advance
    // setpoint rose slowly through ACEL_MAX: the result was that what one
    // felt in the car was not the acceleration but the heading steps -- "the
    // corrections are more abrupt than the speed change". Limiting how much
    // it can move per second, the heading straightens the same but pushing,
    // not jerking.
    limitar(&correccion, deseada, ACEL_CORR, dt);

    // Anti-windup: while the correction is saturated, the integral can only
    // move in the direction that takes it out of saturation. It is compared
    // against `deseada`, not against `correccion`: the ramp above makes the
    // two almost never coincide, and with `correccion` the loop would always
    // believe it is saturated and would stop integrating.
    bool topado = (deseada != sinLimitar);
    if(!topado || (error > 0) != (sinLimitar > 0)) integralYaw += error*dt;
    if(KI_YAW > 0.0001f){
        float topeI = CORR_MAX / KI_YAW;
        integralYaw = constrain(integralYaw, -topeI, topeI);
    }
}

// ######################################################################
// BLOCK 15 -- ToF SENSORS
// COST: 4 x (tcaSelect + rangingTest) ~= 20-30 ms. It is the dominant cost
//   of the slow loop; that is why the control loop does NOT depend on them.
// ######################################################################
float leerToF(uint8_t c){
    tcaSelect(c);
    VL53L0X_RangingMeasurementData_t m;
    lox.rangingTest(&m,false);
    tcaCerrar();
    if(m.RangeStatus==4) return MAX_DIST;
    float d=(float)m.RangeMilliMeter;
    return (d>8000.0f)?MAX_DIST:d;
}
// 🔴 IT TAKES THE BUS ONE SENSOR AT A TIME, NOT ALL FOUR AT ONCE.
// The four ToF together cost 20-30 ms, and while the mutex is taken core 0
// cannot read encoders: requested every 20 ms, two piled up or the window
// was left empty. Releasing between sensors the bus is free four times per
// loop and the encoder sampling slips into those gaps. The four are still
// read per loop: the refresh rate of the ToF, their 5-sample average filter
// and the neurocontroller do not change at all.
// 🔑 The mutex is NOT recursive: whoever calls this cannot already hold it
// (that is why the loop stopped taking it, and that is why purgarFiltros,
// which is called with the bus taken, stayed as it was).
// 🔴 ONE SENSOR PER LOOP, NOT ALL FOUR. Measured on Sep 14 against the robot:
// the telemetry came out every 157 ms even though the firmware asked for it
// every 50, and the reason is that rangingTest() is a single-shot
// measurement and BLOCKS until the sensor has it -- about 40 ms each, 160
// the four. That was the real period of the whole loop, and with the mutex
// taken for 40 ms in a row core 0 could not read encoders, which it
// requests every 20.
//
// Reading one per loop, the loop goes from ~160 ms to ~40 and the bus is
// free between measurements. 🔑 EACH SENSOR IS STILL REFRESHED AS FAST AS
// BEFORE: four 40 ms loops are the same 160 ms the full round took. Nothing
// is lost in perception and everything else is gained -- commands, CSV and
// state machine at 40 ms instead of 160.
int  toca = 0;             // sensor to measure in this loop
bool rondaCompleta = false;  // true on the loop in which the round closes

void leerUnSensor(float out[4]){
    const uint8_t canal[4] = {CANAL_N, CANAL_E, CANAL_S, CANAL_O};
    I2C_TOMAR();
    float d = leerToF(canal[toca]);
    I2C_SOLTAR();
    out[toca] = filtros[toca].agregar(corregirToF(toca, d));
    toca = (toca + 1) % 4;
    rondaCompleta = (toca == 0);
}
void purgarFiltros(){
    filtros[0].purgar(corregirToF(0, leerToF(CANAL_N)));
    filtros[1].purgar(corregirToF(1, leerToF(CANAL_E)));
    filtros[2].purgar(corregirToF(2, leerToF(CANAL_S)));
    filtros[3].purgar(corregirToF(3, leerToF(CANAL_O)));
}

// ######################################################################
// BLOCK 16 -- PERMANENT MEMORY (NVS)
// WHAT IT DOES: saves what was learned -- each wheel's take-off PWM and the
//   measured encoder signs.
// IMPORTANCE: HIGH. Without this, the first stretch after each power-up
//   searches for the take-off blindly again: measured, the rear right
//   started from 39 and took two seconds to reach the right value while the
//   two on the left were already rolling. That is the start deviation.
// ######################################################################
void guardarDespegues(){
    memoria.begin("maindef", false);
    for(int i=0;i<N_RUEDAS;i++){
        char clave[8]; snprintf(clave,sizeof(clave),"d%d",i);
        memoria.putInt(clave, (int)(ruedas[i].pwmDespegue+0.5f));
    }
    memoria.end();
}
void recuperarDespegues(){
    memoria.begin("maindef", true);
    int rec=0;
    for(int i=0;i<N_RUEDAS;i++){
        char clave[8]; snprintf(clave,sizeof(clave),"d%d",i);
        int v = memoria.getInt(clave, 0);
        if(v>=20 && v<=DESPEGUE_MAX){ ruedas[i].pwmDespegue=v; rec++; }
        else if(v>DESPEGUE_MAX){ ruedas[i].pwmDespegue=DESPEGUE_INICIAL[i]; }  // learned value gone wrong
    }
    memoria.end();
    Serial2.printf("  take-off PWM: %d of %d learned ->", rec, N_RUEDAS);
    for(int i=0;i<N_RUEDAS;i++)
        Serial2.printf(" %s %d", ruedas[i].corto, (int)ruedas[i].pwmDespegue);
    Serial2.println();
}
// 🔴 POLARITY VERSION (Sep 12, audit; same mechanism as the "pol" key of
// carro_v08). A saved encoder sign is only valid for the motor polarity it
// was measured with: changing invertirMotor of a wheel changes what 'V'
// calls "forward" and its sign ends up reversed. And the automatic flip of
// BLOCK 13 also saved on the fly signs that could come from a transient. So
// the saved signs are tagged with the version of the polarity table; if it
// does not match, THEY ARE NOT LOADED: the robot drives with the table's
// starting ones (those carro_v08 measured with this same polarity) and warns
// that they must be measured again with 'V'.
// Raise this number every time an invertirMotor is touched.
//   1 = table of Sep 12 15:59 (TI=true, the one that turned out wrong)
//   2 = TI=false, like carro_v08
//   3 = DI<->TD swapped (DI=M2, TD=M3) and both false (Sep 14, floor)
// 4: DD and TI signs fixed with the Sep 14 batch. When this is raised, the
// signs saved in NVS (the old, bad ones) are ignored and the table ones are
// used. The warning 'signs_from_other_polarity' shows up at boot: it is
// expected, and it says the old memory was NOT reused.
const int VERSION_POLARIDAD = 6;

void guardarSignos(){
    memoria.begin("maindef", false);
    for(int i=0;i<N_RUEDAS;i++){
        char clave[8]; snprintf(clave,sizeof(clave),"s%d",i);
        memoria.putInt(clave, ruedas[i].signoEncoder);
    }
    memoria.putInt("pol", VERSION_POLARIDAD);   // with which polarity they were measured
    memoria.end();
}
void recuperarSignos(){
    memoria.begin("maindef", true);
    int pol = memoria.getInt("pol", 0);
    int rec=0;
    if(pol == VERSION_POLARIDAD){
        for(int i=0;i<N_RUEDAS;i++){
            char clave[8]; snprintf(clave,sizeof(clave),"s%d",i);
            int v = memoria.getInt(clave, 0);
            if(v==1 || v==-1){ ruedas[i].signoEncoder=v; rec++; }
        }
    }
    memoria.end();
    if(pol != VERSION_POLARIDAD){
        // The saved ones were measured (or flipped on the fly) with ANOTHER
        // polarity: they are useless. The table stays, and it is reported on
        // both links so it can be seen from the Pi interface.
        Serial2.printf("  encoder signs: the saved ones belong to polarity %d and "
                       "this is %d. THEY ARE IGNORED; the table ones are used. Robot "
                       "on its stand and 'V' to measure them with this polarity.\n",
                       pol, VERSION_POLARIDAD);
        Serial.printf("WARN:signs_from_other_polarity,saved=%d,current=%d,measure_with_V\n",
                      pol, VERSION_POLARIDAD);
    } else if(rec == N_RUEDAS)
        Serial2.println("  encoder signs: all four, measured in another session");
    else
        Serial2.printf("  encoder signs: %d measured, %d still assumed. "
                       "Measure them with 'V' before driving: with one reversed the "
                       "motor runs away.\n", rec, N_RUEDAS-rec);
}

// ######################################################################
// BLOCK 17 -- LOCOMOTION STATE AND CORE ASSIGNMENT
// WHAT IT DOES: the only state machine left on the ESP, and the split of
//   the work between the two cores of the ESP32.
//
// 🔑 WHY TWO CORES. The speed loop has to run every 100 ms with the encoder
//   counts read every 20; the four VL53L0X cost 20-30 ms per loop on their
//   own. In a single loop, the control loop would be at the mercy of the
//   ToF: it is exactly the problem that in carro_v08 made a cycle take a
//   second and the integral add ten times what it should all at once.
//     core 0 -> tareaControl(): encoders, IMU, heading, speed, PWM.
//     core 1 -> loop(): ToF, neurocontroller, protocol, CSV.
//   The I2C bus is shared by both, so EVERY transaction goes inside the
//   mtxI2C mutex. Without it, a half-done tcaSelect between the two tasks
//   leaves the mux open on two channels and the readings come out mixed.
// ######################################################################
enum EstadoLoco { LOCO_IDLE, LOCO_AVANZANDO, LOCO_GIRANDO,
                  LOCO_CENTRANDO, LOCO_BLOQUEADO, LOCO_CALIBRANDO,
                  LOCO_MANUAL };   // 6: the Pi sends vx/vy/w by hand
volatile EstadoLoco estadoLoco = LOCO_IDLE;

// The bus mutex is declared above, with the bus itself (BLOCK 03): it is used
// both by core 0 (encoders, IMU) and by core 1 (ToF).

bool    avanzando       = false;
uint8_t comandoActivo   = CMD_PARAR;
uint8_t eventoPendiente = EV_NADA;
int     confirma_fuera  = 0;
float   err_giro        = 0.0f;
volatile bool calibrando = false;    // the control task does not touch the motors

// ######################################################################
// MANUAL MODE (Sep 14: through the LOOP, no longer by raw PWM). The Pi sends
// a '#m=vx,vy,w' line (each one -255..255, a fraction of the cap) and that
// becomes ordenVx/ordenVy/ordenW: the SAME input navigation uses, with the
// same per-wheel speed loop, the same ramp and the same heading loop. It is
// exactly how carro_v08 was driven from its station, and it is what makes
// the four wheels go together even though DD is a motor twice as fast.
//
// WHY THE RAW PWM WENT AWAY: it sent the same PWM to the four, DD gave twice
// the rpm of the other three, and "forward" came out curved and "turn" came
// out sideways. It was not a useful wiring test: it was a mecanum chassis
// driven as if it were differential and with an unbalanced wheel.
//
// It is entered with any non-zero '#m=' (it turns the motors on if they were
// off), it is left with any navigation command, and BY ITSELF: if no other
// '#m=' line arrives within HOMBRE_MUERTO_MS, it brakes and goes back to
// IDLE. A dropped link cannot leave the robot moving. '#m=0,0,0' brakes
// immediately (request to zero, no down ramp) and stays in manual.
// ######################################################################
volatile bool manual = false;
uint32_t tManual = 0;
const uint32_t HOMBRE_MUERTO_MS = 600;

// 🔴 PUSHES: THE ESP SUSTAINS THE MOTION ON ITS OWN.
// The page control repeated '#m=' every 250 ms and the ESP braked if 600 ms
// passed without receiving one. For a 5 s push that means TWENTY consecutive
// messages that have to go through browser -> WiFi -> Pi -> HTTP -> serial,
// and it is enough for ONE to be delayed for the ESP to brake. And what
// comes after is worse than the braking: the next '#m=' ENTERS manual mode
// again, and entering resets the ramp from zero, so the car starts again
// from standstill. Measured on Sep 14: "it starts for half a second, stops,
// comes back and advances, and so on". It was not the control: it was asking
// the network for twenty punctual deliveries in a row for something the ESP
// can sustain on its own.
//
// '#t=vx,vy,w,ms' requests a push: the command is kept until the time is up,
// without needing a single further message. The network can no longer cut it.
//
// 🔑 SAFETY IS NOT LOST, IT CHANGES FORM: the dead man's switch protected
// against a dropped link, and a push has its own guaranteed end. A link that
// drops halfway leaves the car moving for whatever is left of the push and
// never more -- that is why there is a hard cap, and that is why '0', 'H' and
// '#m=0,0,0' cut it immediately.
uint32_t tiradaHasta = 0;                 // 0 = no push: the dead man's switch rules
// Closing braking: the push completed its time and is ramping down with the
// loop still active (see the end of the push in the loop).
bool     frenando = false;
uint32_t tFrenado = 0;
const uint32_t FRENADA_TOPE_MS = 1500;    // maximum time to finish braking
uint32_t tAtascado = 0;                   // since when it is requested and nothing turns
const uint32_t ATASCO_MS = 1200;          // before declaring it stalled
const uint32_t TIRADA_MAX_MS = 10000;     // hard cap, no matter what
// Control caps: at 255 on the slider these are requested; with the slider at
// 140 (the page default) that gives 0.16 m/s and 49 degrees/s, in the order
// of the cruise speed (0.228) and the 90-degree turn (60). They are changed
// live with '#vmanual=' and '#wmanual='.
// 🔴 0.30 m/s = 99 rpm per wheel, AND THE LIMIT IS THE PWM MARGIN, not the
// speed cap. With 0.42 (140 rpm) the three long-gearbox wheels sat at PWM 255
// in steady state: saturated, the loop has nothing left to correct upwards.
// Measured in the air; under load there will be even less margin.
float VEL_MANUAL_MAX = 0.30f;      // m/s (vx and vy)
float W_MANUAL_MAX_DPS = 90.0f;    // degrees/s

void salirManual(){
    tiradaHasta = 0;               // any exit cancels the push
    frenando = false;
    if(!manual) return;
    manual = false;
    ordenVx = ordenVy = ordenW = 0;
    pedidoVx = pedidoVy = pedidoW = 0;
    if(estadoLoco == LOCO_MANUAL) estadoLoco = LOCO_IDLE;
}
void aplicarManual(int vx, int vy, int w){
    cuadriculaActiva = false;
    vx = constrain(vx, -255, 255); vy = constrain(vy, -255, 255);
    w  = constrain(w,  -255, 255);
    if(!manual){
        if(vx == 0 && vy == 0 && w == 0) return;   // a stray "stop" does not enter
        // Entering: whatever the state machine was doing is cut.
        avanzando = false; comandoActivo = CMD_PARAR;
        estadoLoco = LOCO_MANUAL; manual = true;
        habilitado = true;
        // 🔴 THEY ARE ALWAYS RE-ARMED ON ENTERING, not only if the motors were
        // off. A cut-off wheel stayed cut off for the rest of the session: in
        // the Sep 14 batch, DD and TI were cut off 37 s into the first push
        // and the THREE following pushes were done with two wheels, without
        // anything saying so except the `cortadas` field of the CSV.
        // Each push is a new test and starts with the four wheels.
        // If the cut-off is real, it happens again within half a second and
        // is recorded just the same -- but it does not drag the next test along.
        for(int i=0;i<N_RUEDAS;i++){
            ruedas[i].desbocada = false; ruedas[i].sospechas = 0;
        }
    }
    tManual = millis();
    // A new command cancels the closing braking: if another push arrives while
    // it was stopping, the new one rules.
    if(vx || vy || w) frenando = false;
    ordenVx = vx / 255.0f * VEL_MANUAL_MAX;
    ordenVy = vy / 255.0f * VEL_MANUAL_MAX;
    ordenW  = w  / 255.0f * W_MANUAL_MAX_DPS * PI / 180.0f;
    if(vx == 0 && vy == 0 && w == 0){
        // Releasing = braking now. The ramp exists to start without a jerk; for
        // stopping it adds nothing (same decision as pararGiroYa).
        pedidoVx = pedidoVy = pedidoW = 0;
        tiradaHasta = 0;           // and cuts any push in progress
    }
}

// '#t=vx,vy,w,ms': like manual, but the ESP sustains it on its own until the
// time is up. Not a single further message is needed (see above).
void aplicarTirada(int vx, int vy, int w, long ms){
    cuadriculaActiva = false;
    aplicarManual(vx, vy, w);
    if(vx == 0 && vy == 0 && w == 0) return;     // a "stop" does not open a push
    if(ms <= 0) { tiradaHasta = 0; return; }     // 0 = classic pulsed control
    if(ms > (long)TIRADA_MAX_MS) ms = TIRADA_MAX_MS;
    tiradaHasta = millis() + (uint32_t)ms;
}
// Current ToF readings and their wall reading. They are global because each
// loop refreshes only ONE (see leerUnSensor): the other three keep their last
// good value. They start as "no wall in sight" so that the first cycle,
// before anything has been measured, does not brake the robot believing it
// is boxed in.
float dists[4] = {MAX_DIST, MAX_DIST, MAX_DIST, MAX_DIST};
bool  pared[4] = {false, false, false, false};

unsigned long peorCiclo = 0;         // the slowest control cycle, in ms

// Stall watchdog: there is a setpoint but the encoders do not move.
const unsigned long VENTANA_BLOQUEO_MS = 900;
const float CM_MIN_VIVO = 0.5f;
unsigned long t_ultimo_avance = 0;
float cm_en_ultimo_chequeo = 0;

// ######################################################################
// BLOCK 18 -- THE CONTROL TASK (core 0)
// WHAT IT DOES: the whole loop, at a fixed rate, whatever happens on the
//   other core. It is the only one that writes to the motors during
//   operation.
// IMPORTANCE: CRITICAL. Hard rule: nothing is printed to Serial here (a
//   printf of floats at 115200 takes ~1.5 ms and would eat the rate) and it
//   waits for nothing except the bus mutex.
// ######################################################################
void tareaControl(void *){
    unsigned long tMuestra=0, tControl=0, tSalud=0, tImu=0;
    unsigned long tPrevUs = micros();
    for(;;){
        unsigned long ahora = millis();

        // --- IMU: the BNO085 sends a report every MS_REPORTE; asking it more
        // often only spends bus, which encoders and ToF share.
        if(ahora - tImu >= (unsigned long)MS_REPORTE){
            tImu = ahora;
            unsigned long ahoraUs = micros();
            float dt = (ahoraUs - tPrevUs)/1e6f;   // informative only: the heading arrives finished
            tPrevUs = ahoraUs;
            I2C_TOMAR(); actualizarIMU(dt); I2C_SOLTAR();
        }

        if(ahora - tMuestra >= (unsigned long)MS_MUESTRA){
            tMuestra = ahora;
            I2C_TOMAR(); muestrearEncoders(); I2C_SOLTAR();
        }

        if(ahora - tControl >= (unsigned long)MS_CONTROL){
            unsigned long transcurrido = ahora - tControl;
            tControl = ahora;
            unsigned long t0 = millis();

            rampa(transcurrido/1000.0f);
            calcularCorreccion(transcurrido/1000.0f);
            // During the direction calibration the motors are moved by someone
            // else with raw PWM: the loop neither writes NOR evaluates runaway,
            // because with a target of 0 any turning wheel would look shot up.
            if(!calibrando){
                corregirVelocidad(transcurrido);
                for(int i=0;i<N_RUEDAS;i++) aplicar(i);
            }

            unsigned long dur = millis()-t0;
            if(dur > peorCiclo) peorCiclo = dur;
        }

        if(ahora - tSalud >= (unsigned long)MS_SALUD){
            tSalud = ahora;
            I2C_TOMAR(); muestrearSalud(); I2C_SOLTAR();
        }
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}

// ######################################################################
// BLOCK 19 -- TURN ON THE SPOT, CLOSED BY THE IMU (BLOCKING)
// WHAT IT DOES: rotates the chassis 90 or 180 degrees and does not return
//   until it is done.
//
// CHANGE FROM THE PREVIOUS VERSION: the turn NO LONGER writes raw PWM. It
// asks the SAME loop as the advance for a turn rate and closes on yaw. It is
// what in carro_v08 gave 724 degrees turned out of 721 requested over two
// full laps (100.5%), with the turn rate at 59 degrees/s out of the 60
// requested. A calibrated loop follows a reference better than a fixed PWM.
//
// WHY IT IS BLOCKING (the user's decision): while turning, core 1 does not
// handle commands or emit CSV -- the radio silence of the field test. Core 0
// KEEPS running the loop: the motors are never left without control. An
// emergency 'H' during a turn is not handled until it ends or
// TIMEOUT_GIRO_MS expires.
//
// 🔑 WHEN IT ENDS, THE REFERENCE STICKS TO THE HEADING REACHED. After a turn,
// the heading reached IS the right heading: otherwise the robot is left
// owing the 6-11 degrees it did not close and the next straight stretch
// spends them veering sideways right from the start.
// ######################################################################
const float    GIRO_DEG        = 90.0f;
// 🔴 THE TOLERANCE HAS TO BE LARGER THAN THE STEP THE CHASSIS CAN MAKE.
// carro_v08 measured it: the fastest wheel does not go below 54 rpm ~ 65
// degrees/s, so the finest motion this robot can make is about 6 degrees,
// and that is why the dead band there was 3 and not 1. Asking for 2 is asking
// for something the hardware cannot resolve: the robot enters the band,
// overshoots, comes back, overshoots -- the left/right back-and-forth --
// spends the three passes and closes with EV_ERR_GIRO, which leaves the
// state in LOCO_BLOQUEADO even though the turn ended 2.5 degrees off.
// Editable live ('#tolgiro=') to fine-tune it on the floor without
// reflashing.
float          TOL_GIRO_DEG    = 4.4f;
const uint32_t T_ASENTADO_MS   = 150;
const uint32_t TIMEOUT_GIRO_MS = 8000;
const uint32_t T_INERCIA_MS    = 350;
const int      MAX_PASADAS     = 3;
// Gain of the outer turn loop: degrees of error -> requested degrees/s.
// Deliberately soft and capped at VEL_GIRO_DPS: what closes the turn
// precisely is the speed loop below, not the aggressiveness of this one.
const float KP_GIRO_EXT = 2.0f;

// 🔴 THE RIGHT TURN GOES WITH ITS OWN NUMBERS (Sep 14, measured on the floor:
// to the left it closes cleanly and to the right it oscillates at the end).
//
// The turn code is SYMMETRIC -- right and left are the same function with
// the sign changed -- so a measured asymmetry cannot come from the control:
// it comes from the chassis. The candidate is in the wheel table itself: DD
// delivers 189.4 rpm at PWM 160 and its three companions 94.9, 82.3 and
// 85.2. It is TWICE as much. With the old model DD was given half the PWM of
// the others for the same requested rpm; near the end of the turn, when few
// rpm are requested, that PWM dropped below what the wheel needs to break
// the friction, the wheel stuck, the integral pushed it, it broke free
// suddenly and the chassis overshot. That was the oscillation.
//
// 🔑 THE BLOCK 07 LINE ATTACKS THAT DIRECTLY: DD's pwmBase is 53.4, so however
// few rpm it is asked for it never receives less than that. If the right
// turn stops oscillating, these two numbers below can be equalized again
// with the left ones. They have not been touched yet: one change, one batch.
//
// Until DD's rpmA160 is measured again, the right direction is requested
// more slowly and with a more gradual approach. Both are editable live
// ('#vgiroder=', '#kpgiroder=') to fine-tune them without reflashing.
// 🔑 THE LEFT DIRECTION USES NONE OF THIS: it keeps VEL_GIRO_DPS and
// KP_GIRO_EXT, exactly as they were.
// 🔑 THE RIGHT ONE GOES SLOWER THAN THE LEFT ONE, and that is NOT a patch: it
// is what Sebas approved at the start of the session ("the turns look good,
// they oscillate much less") and it comes from a real asymmetry of the
// chassis. That split is restored. If the right one oscillates again only
// this one is lowered ('#vgiroder='): the left one is unaffected.
// 🔑 SAME AS THE LEFT ONE: the asymmetry is no longer needed. It existed
// because the right turn oscillated at the end, and that came from the front
// right wheel sticking and the integral releasing it suddenly -- the start
// pulse and the anti-windup fixed it. Measured at 60 both sides give the
// same error.
float VEL_GIRO_DER_DPS = 60.0f;   // speed cap turning to the right
float KP_GIRO_EXT_DER  = 1.2f;    // softer: it reaches the end with less momentum
// 🔴 THE MINIMUM HAS TO BE REACHABLE. 12 degrees/s are 10 rpm per wheel, below
// the take-off floor of the four: the loop asks for 12 and the floor delivers
// ~65, so in the final stretch of the turn the command and the motion have
// nothing to do with each other and the robot systematically overshoots.
// Raising it does not make the turn more abrupt -- the chassis already went
// at that speed -- but it does make what is requested and what happens
// coincide, which is what the yaw closing lives on. Editable live
// ('#vgiromin=').
float VEL_GIRO_MIN_DPS = 25.0f;

// 🔴 STOPPING THE TURN MEANS STOPPING NOW (Sep 12, audit). ordenW=0 only tells
// the ramp (ACEL_GIRO=2.5 rad/s2) to start going down: from 60 degrees/s it
// takes 0.42 s to reach zero, and meanwhile the loop keeps asking the wheels
// to turn -- and they cannot go slowly anyway (take-off floor), so the
// chassis keeps rotating at 20-60 degrees/s with the command already at zero.
// Measurable consequences: inside the +-2 degree band ordenW=0 is set and
// the robot leaves the band by itself, between passes the "achieved" angle
// is measured with the robot still turning (T_INERCIA_MS does not cover the
// ramp), and EV_FIN_GIRO is emitted with the wheels on one side still
// pushing backwards, which is what the next advance receives.
// The ramp exists to START without a jerk; for stopping it adds nothing.
// Here the request is cut at once: target -> 0, the wheels coast.
static inline void pararGiroYa(){ ordenW = 0; pedidoW = 0; }

// `derecha` selects the parameter set: the left direction uses the usual
// ones and the right one its own (see above). It is the ONLY thing that
// distinguishes one direction from the other; the rest of the loop is
// identical.
// Declared here because the turn emits CSV and lives before BLOCK 22.
void enviarCSV(float d[4], bool pared[4]);

// `tol` is the closing band: TOL_GIRO_DEG for the 90-degree turns and
// TOL_ALINEAR_DEG for the fine alignment ('Y').
// 🔴 FORCED STOP (Sep 15 night): 'H' or '0' arriving in the middle of a turn
// or of the wall sticking -- which are blocking and do not read the serial
// -- brake and disable the motors immediately. Everything available on the
// serial is read; whatever is not a stop is discarded (during a turn nothing
// else is handled).
bool paroForzoso = false;
bool hayParoSerie(){
    bool paro = false;
    while(Serial.available()){
        int c = Serial.read();
        // Only 'H': a stray '0' (the soft brake the Pi sends when a timeout
        // expires, or the one from "#m=0,0,0") does NOT turn the motors off.
        if(c=='H' || c=='h') paro = true;
    }
    if(paro){
        frenarTodo(); habilitado = false; paroForzoso = true;
        ordenVx = ordenVy = ordenW = 0; pedidoVx = pedidoVy = pedidoW = 0;
        Serial.println("WARN:forced_stop,motors_disabled");
    }
    return paro;
}

bool accionGiro(float objetivo_abs, bool derecha, float tol){
    paroForzoso = false;
    const float vTope = derecha ? VEL_GIRO_DER_DPS : VEL_GIRO_DPS;
    const float TOL_GIRO_DEG = tol;   // local shadow: the rest of the loop does not change

    bool logrado = false;
    uint32_t t0 = millis();
    // The turn emits CSV, as in batch 10. 🔴 WATCH OUT IF IT EVER JERKS: a line
    // is ~330 bytes against a 128-byte output buffer, so Serial BLOCKS until
    // they fit -- about 30 ms at 115200 baud -- and this loop should run every
    // 5. Removing it is the first thing to try, but it is kept because it is
    // what was there when the turn measured best.
    uint32_t tCsvGiro = 0;

    for(int pasada=0; pasada<MAX_PASADAS && !logrado; pasada++){
        uint32_t t_dentro = 0;
        bool dentro = false;

        while(millis() - t0 < TIMEOUT_GIRO_MS){
            if(hayParoSerie()) return false;
            float e = objetivo_abs - yawMedido;      // degrees left
            if(fabsf(e) < TOL_GIRO_DEG){
                // Zero is requested and it ramps down, as at the end of a
                // straight push. It already comes slowly because of the
                // anticipated braking, so the hard cut is not needed.
                ordenW = 0;
                if(!dentro){ dentro = true; t_dentro = millis(); }
                else if(millis() - t_dentro >= T_ASENTADO_MS) break;
                if(millis() - tCsvGiro >= PERIODO_CSV_MS){
                    tCsvGiro = millis(); enviarCSV(dists, pared);
                }
                delay(5);
                continue;
            }
            dentro = false;
            // 🔴 THE TURN GOES LIKE DRIVING: CONSTANT CRUISE AND ANTICIPATED
            // BRAKING (Sep 14). Before, the speed came from multiplying the
            // error by a gain, and with 90 degrees to go that asks for the cap
            // from the first instant: a jolt. Worse still when the chassis
            // does not manage to turn -- the error does not drop, so the
            // command stays stuck at the maximum and the loop pushes harder
            // and harder. It is the "if it cannot turn it gives it full speed".
            //
            // Now there is a fixed, moderate cruise speed, just like forward/
            // backward, and it only goes down in the last degrees: it enters
            // the band already slowly, instead of arriving at full speed and
            // having to brake hard. The up ramp is set by rampa() with
            // ACEL_GIRO, so the start is progressive and the four wheels have
            // time to take off together.
            float dps = vTope;
            if(fabsf(e) < GRADOS_FRENADA_GIRO){
                dps = fmax(VEL_GIRO_MIN_DPS, vTope * fabsf(e) / GRADOS_FRENADA_GIRO);
            }
            ordenW = (e > 0 ? +1.0f : -1.0f) * dps * PI / 180.0f;
            if(millis() - tCsvGiro >= PERIODO_CSV_MS){
                tCsvGiro = millis(); enviarCSV(dists, pared);
            }
            delay(5);
        }
        pararGiroYa();

        // The chassis inertia moves the yaw after cutting. It waits, measures
        // again, and if it ended outside another pass is spent: that is what
        // avoids reporting an optimistic end of turn.
        delay(T_INERCIA_MS);
        logrado = fabsf(objetivo_abs - yawMedido) < TOL_GIRO_DEG;
        if(millis() - t0 >= TIMEOUT_GIRO_MS) break;
    }
    err_giro = objetivo_abs - yawMedido;
    return logrado;
}

// pegarse_a_la_pared() from square_right.py, as is: after the turn, stopped,
// translate sideways until reaching the setpoint of the wall it sees (the
// closest one if it sees both). Fixed force VY_PEGAR (when stopped a LARGE
// translation is needed: with +-27 it does not break free) and what is
// modulated is the pulse TIME, anticipating what slides when cutting
// (INERCIA_LAT). Blocking, like the turn it belongs to. The heading loop
// keeps defending the grid.
void pegarseALaPared(){
    uint32_t t0 = millis();
    uint32_t tCsv = 0;
    int lado0 = 0;
    while(millis() - t0 < (uint32_t)(TOPE_PEGAR_S * 1000.0f)){
        I2C_TOMAR();
        float dE_ = corregirToF(1, leerToF(CANAL_E));
        float dO_ = corregirToF(3, leerToF(CANAL_O));
        I2C_SOLTAR();
        bool d = dE_ > 10.0f && dE_ < 200.0f, i = dO_ > 10.0f && dO_ < 200.0f;
        int lado = (d && i) ? (dE_ <= dO_ ? 1 : 3) : d ? 1 : i ? 3 : 0;   // 1 = E, 3 = O (W)
        if(lado == 0) break;                       // I see no wall to the side
        if(lado0 == 0) lado0 = lado;
        if(lado != lado0) break;                   // wall change: as in the script
        // 🔴 Safety (Sep 16 afternoon): if a side falls below its threshold, a
        // WITHDRAWAL pulse (not just stopping: it ended up 16 mm from the wall
        // and the advance started scraping) and the sticking is ended.
        {
            int cerca = (dE_ > 1.0f && dE_ < SEG_E) ? 1 : (dO_ > 1.0f && dO_ < SEG_O) ? 3 : 0;
            if(cerca){
                Serial.printf("WARN:safety,side=%s,mm=%.0f,stick_withdraws\n",
                              cerca == 1 ? "E" : "O", cerca == 1 ? dE_ : dO_);
                ordenVx = 0; ordenVy = (cerca == 1) ? +VY_PEGAR_MS : -VY_PEGAR_MS;   // move away
                uint32_t tr = millis();
                while(millis() - tr < 120){ if(hayParoSerie()) return; delay(5); }
                ordenVy = 0; pedidoVy = 0;
                delay(300);
                break;
            }
        }
        float dist = (lado == 1) ? dE_ : dO_;
        float consigna = (lado == 1) ? CONSIGNA_E_MM : CONSIGNA_O_MM;
        float err = dist - consigna;               // > 0 = far from that wall
        float restante = err - (err > 0 ? INERCIA_LAT_MM : -INERCIA_LAT_MM);
        if(err * restante <= 0.0f || fabsf(restante) <= BANDA_PEGAR_MM) break;
        // vy > 0 goes to the LEFT: moving away from the right asks for vy > 0
        float sentido = (lado == 1) ? -1.0f : +1.0f;
        float vy = sentido * (err > 0 ? VY_PEGAR_MS : -VY_PEGAR_MS);
        uint32_t ms = (uint32_t)constrain(fabsf(err) * 6.0f, 35.0f, 150.0f);   // cap 150 (was 250)
        ordenVx = 0; ordenVy = vy;
        uint32_t tp = millis();
        while(millis() - tp < ms){
            if(hayParoSerie()) return;
            if(millis() - tCsv >= PERIODO_CSV_MS){ tCsv = millis(); enviarCSV(dists, pared); }
            delay(5);
        }
        ordenVy = 0; pedidoVy = 0;                 // hard cut, like parar()
        uint32_t te = millis();
        while(millis() - te < 300){
            if(millis() - tCsv >= PERIODO_CSV_MS){ tCsv = millis(); enviarCSV(dists, pared); }
            delay(5);
        }
    }
    ordenVy = 0; pedidoVy = 0;
    delay(500);
    I2C_TOMAR();
    float dE2 = corregirToF(1, leerToF(CANAL_E)), dO2 = corregirToF(3, leerToF(CANAL_O));
    I2C_SOLTAR();
    // Visible in the interface messages: what was executed and where it ended.
    Serial.printf("OK:stick,side=%s,dE=%.0f,dO=%.0f,%.1fs\n",
                  lado0 == 1 ? "E" : lado0 == 3 ? "O" : "none", dE2, dO2,
                  (millis() - t0) / 1000.0f);
}

// Single entry point to the turn. pasos90: +1 right, -1 left, +-2 half turn.
void ejecutarGiro(int pasos90){
    estadoLoco = LOCO_GIRANDO;
    girando = true;            // the loop gives more margin before cutting off
    ordenVx = 0; ordenVy = 0;
    // pasos90 > 0 is "to the right" for the Pi (N->E); in the yaw convention
    // (positive = left) those are NEGATIVE degrees. This is the only place
    // where it is translated: the loop below already works in degrees.
    // 🔑 THE TARGET BELONGS TO THE GRID, not "what I measure minus 90": this
    // way what a turn left unclosed is recovered by the next one instead of
    // being dragged along forever (Sep 15: four turns added up to 7-8 degrees).
    cuartos += pasos90;
    float objetivo = rumboCuadricula();

    // pasos90 > 0 is right (and the half turn, +2, also rotates to the right).
    bool ok = accionGiro(objetivo, pasos90 > 0, TOL_GIRO_DEG);

    ordenVx = 0; pararGiroYa();
    if(ok){
        yawRef = objetivo;       // the right heading is the grid one
        huboGiroCuadricula = true;
    } else {
        // It was not reached: the Pi does NOT confirm the turn and keeps
        // believing the previous heading, so it is undone here too. The next
        // 'Y' takes it back to the heading the Pi believes, and the two match
        // again.
        cuartos -= pasos90;
        yawRef = rumboCuadricula();   // the grid rules, not the sensor
    }
    integralYaw = 0;
    // 🔴 AFTER EACH TURN, STICK TO THE WALL BEFORE ADVANCING (square test): the
    // turn does not leave the robot centered in the new corridor (22 mm to
    // the left on average, measured), and if it starts like that it goes into
    // the wall. It is done STOPPED: correcting it while moving forces shoves
    // right when leaving the turn and each 20 mm translation tilts the
    // heading 1.3 degrees.
    if(ok && PEGAR_ACTIVO > 0.5f && habilitado && !paroForzoso) pegarseALaPared();
    if(paroForzoso) ok = false;
    I2C_TOMAR(); purgarFiltros(); I2C_SOLTAR();   // the world turned
    iniciarTramo();

    girando = false;
    estadoLoco = ok ? LOCO_IDLE : LOCO_BLOQUEADO;
    comandoActivo = CMD_PARAR;
    eventoPendiente = ok ? EV_FIN_GIRO : EV_ERR_GIRO;
}

// Command 'Y': turns only what is missing to sit on the grid heading.
// Blocking like the turn. If it is already within TOL_ALINEAR_DEG it moves
// nothing and answers EV_FIN_GIRO immediately.
void ejecutarAlineacion(){
    float objetivo = rumboCuadricula();
    float e = objetivo - yawMedido;
    if(fabsf(e) < TOL_ALINEAR_DEG){
        yawRef = objetivo; err_giro = e;
        eventoPendiente = EV_FIN_GIRO;
        return;
    }
    estadoLoco = LOCO_GIRANDO;
    girando = true;
    ordenVx = 0; ordenVy = 0;
    bool ok = accionGiro(objetivo, e < 0, TOL_ALINEAR_DEG);   // e<0: it is missing to the right
    ordenVx = 0; pararGiroYa();
    yawRef = objetivo;            // the grid rules even if it did not close
    huboGiroCuadricula = ok;
    integralYaw = 0;
    I2C_TOMAR(); purgarFiltros(); I2C_SOLTAR();
    iniciarTramo();
    girando = false;
    estadoLoco = ok ? LOCO_IDLE : LOCO_BLOQUEADO;
    comandoActivo = CMD_PARAR;
    eventoPendiente = ok ? EV_FIN_GIRO : EV_ERR_GIRO;
}

// 🔴 Command '9' (Sep 16): SETTLE BEFORE EACH DECISION. It aligns the heading
// to the grid's multiple of 90 (the same as 'Y', only if needed) and THEN
// sticks to the side wall while stopped (pegarseALaPared, the one from the
// square test), always. Blocking; it answers EV_FIN_GIRO (7) or EV_ERR_GIRO
// (8). The Pi neither decides nor sends anything until it closes: this way
// the '5' always starts from a straight and centered robot, which is what
// avoids crashing into the wall.
void ejecutarAcomodo(){
    paroForzoso = false;
    float objetivo = rumboCuadricula();
    float e = objetivo - yawMedido;
    bool ok = true;
    estadoLoco = LOCO_GIRANDO;
    girando = true;
    ordenVx = 0; ordenVy = 0;
    if(fabsf(e) >= TOL_ALINEAR_DEG){
        ok = accionGiro(objetivo, e < 0, TOL_ALINEAR_DEG);
        ordenVx = 0; pararGiroYa();
    }
    yawRef = objetivo; err_giro = objetivo - yawMedido;
    huboGiroCuadricula = ok;
    integralYaw = 0;
    if(PEGAR_ACTIVO > 0.5f && habilitado && !paroForzoso) pegarseALaPared();
    if(paroForzoso) ok = false;
    I2C_TOMAR(); purgarFiltros(); I2C_SOLTAR();
    iniciarTramo();
    girando = false;
    estadoLoco = ok ? LOCO_IDLE : LOCO_BLOQUEADO;
    comandoActivo = CMD_PARAR;
    eventoPendiente = ok ? EV_FIN_GIRO : EV_ERR_GIRO;
}

// ######################################################################
// BLOCK 20 -- ENCODER DIRECTION CALIBRATION (command 'V')
// WHAT IT DOES: moves each wheel forward and looks at which direction its
//   encoder counts. Blocking, with the robot ON A STAND (wheels in the air).
// IMPORTANCE: CRITICAL before driving for the first time. With a reversed
//   sign the loop is positive feedback and the motor goes to the cap in
//   less than a second; the runaway protection cuts it, but it is a safety
//   net, not a substitute for measuring it.
// ######################################################################
// Editable live ('#calpwm=', '#calms='): if a wheel does not take off with
// the usual value, raising them tells "it lacks push" from "it cannot turn".
float CAL_PWM   = 150.0f;
float CAL_MS    = 600.0f;
const float CAL_GRADOS_MIN = 40.0f;

void calibrarSentidos(){
    estadoLoco = LOCO_CALIBRANDO;
    calibrando = true;                // the control task does not touch the motors
    frenarTodo();
    delay(200);

    for(int i=0;i<N_RUEDAS;i++){
        Rueda *r = &ruedas[i];
        // It is measured with the sign at +1 to read the raw encoder direction.
        int signoPrevio = r->signoEncoder;
        r->signoEncoder = +1;
        r->gradosTramo = 0.0f;
        r->anguloPrevio = -1;
        I2C_TOMAR(); muestrearEncoders(); I2C_SOLTAR();

        // 🔴 WHAT HAPPENS IS MEASURED, NOT ONLY WHETHER IT MOVED. When this says
        // "did_not_move" there are at least three different causes -- the
        // motor does not push, the encoder does not answer, or the encoder
        // answers but does not see the wheel -- and the old message did not
        // tell them apart. Now it records how many readings went well, how
        // many failed, whether the wheel was declared dead and between which
        // angles the magnet moved.
        int fallosAntes = r->fallosI2C;
        int lecturas = 0, muerta = 0, angIni = -1, angFin = -1;

        moverCrudo(i, (int)CAL_PWM, true);
        uint32_t t0 = millis();
        while(millis()-t0 < (uint32_t)CAL_MS){
            // 🔴 IT IS RE-APPLIED ON EVERY ITERATION. Set only once, the PWM of
            // three of the four wheels got lost: DI turned almost three
            // revolutions with PWM 220 and DD, TI and TD did not move a degree,
            // with the encoders reading perfectly (54 readings, zero I2C
            // failures) -- i.e. someone turned their motor off between being
            // turned on and being measured. Insisting every 20 ms is cheap and
            // makes the calibration immune to that.
            moverCrudo(i, (int)CAL_PWM, true);
            I2C_TOMAR(); muestrearEncoders(); I2C_SOLTAR();
            if(r->vivo){ lecturas++; if(angIni < 0) angIni = r->anguloPrevio;
                         angFin = r->anguloPrevio; }
            else muerta++;
            delay(MS_MUESTRA);
        }
        moverCrudo(i, 0, true);
        delay(300);
        I2C_TOMAR(); muestrearEncoders(); I2C_SOLTAR();

        float g = r->gradosTramo;
        Serial.printf("CALDET:%s,readings=%d,mute=%d,I2Cfailures=%d,ang=%d..%d,degrees=%.1f\n",
                      r->corto, lecturas, muerta, r->fallosI2C - fallosAntes,
                      angIni, angFin, g);
        if(fabsf(g) < CAL_GRADOS_MIN){
            r->signoEncoder = signoPrevio;   // it did not move: nothing is touched
            Serial.printf("CAL:%s,turn=%.1f,ERROR=did_not_move\n", r->corto, g);
        } else {
            r->signoEncoder = (g > 0) ? +1 : -1;
            Serial.printf("CAL:%s,turn=%.1f,sign=%+d\n", r->corto, g, r->signoEncoder);
        }
        r->gradosTramo = 0.0f;
        delay(300);
    }
    guardarSignos();
    calibrando = false;
    estadoLoco = LOCO_IDLE;
    eventoPendiente = EV_ACK;
    Serial.println("OK:direction_calibration_saved");
}

// ######################################################################
// BLOCK 21 -- LINK WITH THE Pi: RECEPTION
// WHAT IT DOES: NON-blocking parser. Two forms coexist:
//   - ONE CHARACTER = a primitive or a bench command (as always).
//   - ONE LINE '#name=value\n' = change a gain LIVE. It is how the control
//     was calibrated in carro_v08: moving the numbers from the interface
//     without uploading the firmware again. The Pi interface sends these
//     lines when a slider is touched.
// IMPORTANCE: CRITICAL. Hard rule: NEVER readStringUntil or any while that
//   waits for data.
// ######################################################################
bool hayComandoNuevo    = false;
int  comandoRecibido    = CMD_PARAR;
bool pedidoCsvInmediato = false;
bool pedidoCalibrar     = false;
char lineaParam[48];
int  nLinea = 0;
bool enLinea = false;

void atenderComandoBanco(int c);

struct Param { const char* nombre; float* destino; float minimo; float maximo; };
const Param PARAMS[] = {
    {"kp",       &KP,          0.0f,  5.0f},
    {"ki",       &KI,          0.0f,  5.0f},
    {"kd",       &KD,          0.0f,  5.0f},
    {"kpy",      &KP_YAW,      0.0f, 20.0f},
    {"kiy",      &KI_YAW,      0.0f, 10.0f},
    {"kdy",      &KD_YAW,      0.0f, 10.0f},
    {"corrmax",  &CORR_MAX,    0.0f, 80.0f},
    {"signoyaw", &SIGNO_YAW,  -1.0f,  1.0f},
    {"crucero",  &VEL_CRUCERO, 0.03f, 0.45f},
    // --- ONLY THE TURN ON THE SPOT -----------------------------------------
    {"vgiro",    &VEL_GIRO_DPS, 10.0f, 180.0f},
    {"vgiroder", &VEL_GIRO_DER_DPS, 10.0f, 180.0f},
    {"vgiromin", &VEL_GIRO_MIN_DPS, 10.0f, 90.0f},
    {"acelgiro", &ACEL_GIRO,      0.2f,    6.0f},
    {"gradfreno",&GRADOS_FRENADA_GIRO, 5.0f, 90.0f},
    {"factecho", &FACTOR_TECHO,   1.0f,   5.0f},   // PWM threshold per wheel
    {"margtecho",&MARGEN_TECHO,   0.0f, 120.0f},   // threshold margin
    {"tolgiro",  &TOL_GIRO_DEG, 1.0f,  10.0f},
    {"tolalin",  &TOL_ALINEAR_DEG, 1.0f, 10.0f},   // 'Y' band (align)
    {"tolavance",&TOL_AVANCE_DEG,  1.0f, 30.0f},   // '5' refuses beyond this
    {"parada",   &DIST_STOP_N,     30.0f, 150.0f}, // front stopping distance (and rear in reverse)
    {"arrimar",  &DIST_ARRIMAR,    50.0f, 200.0f}, // where 'A' stops: center of the crossing cell
    {"paradaS",  &DIST_STOP_S,     30.0f, 150.0f}, // rear stop in reverse
    {"segN",     &SEG_N,           5.0f,  120.0f}, // safety: hard brake per side
    {"segE",     &SEG_E,           5.0f,  120.0f},
    {"segS",     &SEG_S,           5.0f,  120.0f},
    {"segO",     &SEG_O,           5.0f,  120.0f},
    {"fracrev",  &FRAC_REVERSA,    0.2f,  1.0f},   // reverse speed / cruise
    {"fuera",    &UMBRAL_FUERA,   150.0f, 600.0f}, // open field: N, E, O above
    {"umbralN",  &UMBRAL_N,       100.0f, 500.0f}, // wall in front if <
    {"umbralE",  &UMBRAL_E,       100.0f, 500.0f},
    {"umbralS",  &UMBRAL_S,       100.0f, 500.0f},
    {"umbralO",  &UMBRAL_O,       100.0f, 500.0f},
    // --- advance and centering: those of square_right.py -----------------
    {"centrar",  &CENTRAR_ACTIVO,   0.0f,  1.0f},
    {"vmin",     &V_MINIMA_MS,      0.01f, 0.20f},
    {"rampaini", &T_RAMPA_INI_S,    0.0f,  5.0f},
    {"frenadesde",&EMPIEZA_FRENAR_MM,100.0f,600.0f},
    {"latfreno", &LATENCIA_FRENO_S, 0.0f,  1.0f},
    {"vytoque",  &VY_TOQUE_MS,      0.01f, 0.20f},
    {"bandalat", &BANDA_LAT_MM,     0.0f, 60.0f},
    {"errpleno", &ERR_PLENO_MM,     1.0f, 100.0f},
    {"sesgolat", &SESGO_LAT_MM,   -30.0f, 30.0f},
    {"minlat",   &MIN_LATERAL_MM,   0.0f, 100.0f},
    {"vyurg",    &VY_URGENTE_MS,    0.01f, 0.20f},
    {"consE",    &CONSIGNA_E_MM,   20.0f, 120.0f},
    {"consO",    &CONSIGNA_O_MM,   20.0f, 120.0f},
    {"limlat",   &LIMITE_LATERAL_MM,60.0f, 400.0f},
    {"sintras",  &SIN_TRASLADAR_MM, 0.0f, 400.0f},
    {"pegar",    &PEGAR_ACTIVO,     0.0f,  1.0f},
    {"vypegar",  &VY_PEGAR_MS,      0.02f, 0.20f},
    {"inerlat",  &INERCIA_LAT_MM,   0.0f, 40.0f},
    {"bandapegar",&BANDA_PEGAR_MM,  0.0f, 30.0f},
    {"topepegar",&TOPE_PEGAR_S,     0.0f, 20.0f},
    {"acel",     &ACEL_MAX,       0.05f,  1.50f},   // start ramp, m/s2
    {"acelcorr", &ACEL_CORR,      2.0f,  200.0f},   // heading correction ramp
    {"calpwm",   &CAL_PWM,       60.0f,  255.0f},   // calibration push
    {"calms",    &CAL_MS,       200.0f, 3000.0f},   // calibration duration
    {"fraccorr", &FRAC_CORR,      0.02f,   0.60f},   // maximum imbalance between sides
    {"corrmin",  &CORR_MIN,       0.0f,   40.0f},   // minimum heading authority
    {"celda",    &PASO_CELDA_CM, 5.0f, 60.0f},
    {"odom",     &FACTOR_ODOM, 0.5f,  2.0f},
    {"vmanual",  &VEL_MANUAL_MAX, 0.05f, 0.45f},
    {"wmanual",  &W_MANUAL_MAX_DPS, 10.0f, 180.0f},
};
const int N_PARAMS = sizeof(PARAMS)/sizeof(PARAMS[0]);

void aplicarParametro(const char* linea){
    if(linea[0]=='m' && linea[1]=='='){          // '#m=vx,vy,w' -> manual
        int a = 0, b = 0, c = 0;
        int n = sscanf(linea+2, "%d,%d,%d", &a, &b, &c);
        if(n == 3){
            aplicarManual(a, b, c);
            Serial.printf("OK:manual,vx=%d,vy=%d,w=%d\n", a, b, c);
        } else if(n == 2){                       // '#m=vx,w', the old format
            aplicarManual(a, 0, b);
            Serial.printf("OK:manual,vx=%d,vy=0,w=%d\n", a, b);
        } else Serial.printf("ERR:manual_unreadable,%s\n", linea);
        return;
    }
    if(linea[0]=='t' && linea[1]=='='){          // '#t=vx,vy,w,ms' -> push
        int a=0,b=0,c=0; long ms=0;
        if(sscanf(linea+2, "%d,%d,%d,%ld", &a,&b,&c,&ms) == 4){
            aplicarTirada(a,b,c,ms);
            Serial.printf("OK:push,vx=%d,vy=%d,w=%d,ms=%ld\n", a,b,c,ms);
        } else Serial.printf("ERR:push_unreadable,%s\n", linea);
        return;
    }
    char nombre[24]; float valor;
    if(sscanf(linea, "%23[^=]=%f", nombre, &valor) != 2){
        Serial.printf("ERR:param_unreadable,%s\n", linea);
        return;
    }
    for(int i=0;i<N_PARAMS;i++){
        if(strcmp(nombre, PARAMS[i].nombre) == 0){
            *PARAMS[i].destino = constrain(valor, PARAMS[i].minimo, PARAMS[i].maximo);
            Serial.printf("OK:%s=%.4f\n", nombre, *PARAMS[i].destino);
            return;
        }
    }
    Serial.printf("ERR:param_unknown,%s\n", nombre);
}

void leerComandoPi(){
    while(Serial.available()){
        int c = Serial.read();

        if(enLinea){                       // inside a '#name=value'
            if(c=='\n' || c=='\r'){
                lineaParam[nLinea] = 0;
                aplicarParametro(lineaParam);
                enLinea = false; nLinea = 0;
            } else if(nLinea < (int)sizeof(lineaParam)-1){
                lineaParam[nLinea++] = (char)c;
            }
            continue;
        }
        if(c=='#'){ enLinea = true; nLinea = 0; continue; }
        if(c=='\n' || c=='\r' || c==' ') continue;
        if(c=='?'){ pedidoCsvInmediato = true; continue; }
        if(c!='S' && c!='s' && c!='N' && c!='n') salirManual();   // everything else rules
        // 🔴 'H' IS THE FORCED STOP: it brakes AND disables the motors,
        // immediately and idempotently (Sep 15 night: the interface stop used
        // 'E', which TOGGLES, and with stale telemetry two presses in a row
        // turned them back on). '0' is still the soft stop.
        if(c=='H' || c=='h'){
            frenarTodo(); habilitado = false;
            ordenVx = ordenVy = ordenW = 0; pedidoVx = pedidoVy = pedidoW = 0;
            comandoRecibido = CMD_PARAR; hayComandoNuevo = true;
            Serial.println("WARN:forced_stop,motors_disabled");
            continue;
        }
        if(c=='0'){ comandoRecibido = CMD_PARAR;   hayComandoNuevo = true; continue; }
        if(c=='5'){ comandoRecibido = CMD_AVANZAR; hayComandoNuevo = true; continue; }
        if(c=='9'){ comandoRecibido = CMD_CENTRAR; hayComandoNuevo = true; continue; }
        if(c=='A' || c=='a'){ comandoRecibido = CMD_HASTA_PARED; hayComandoNuevo = true; continue; }
        if(c=='D' || c=='d'){ comandoRecibido = CMD_REVERSA;     hayComandoNuevo = true; continue; }
        if(c=='Y' || c=='y'){ comandoRecibido = CMD_ALINEAR;     hayComandoNuevo = true; continue; }
        if(c==CMD_GIRO_DER || c==CMD_GIRO_IZQ || c==CMD_GIRO_180){
            comandoRecibido = c; hayComandoNuevo = true; continue;
        }
        // The digits 1..4 and 6..8 were the absolute and relative commands of
        // the mecanum chassis. They no longer exist: explicit NACK so that an
        // outdated link is noticed right away instead of failing silently.
        if(c>='1' && c<='8'){ comandoRecibido = -1; hayComandoNuevo = true; continue; }
        atenderComandoBanco(c);
    }
}

// ######################################################################
// BLOCK 22 -- LINK WITH THE Pi: STATUS CSV
// WHAT IT DOES: one line per period, fixed fields, always the same order.
//
// 🔑 THE FIELD NAMES ARE PUBLISHED. At boot (and when the Pi sends 'N') a
// line 'C,<name>,<name>,...' with the header goes out. The Pi builds its
// parser from that list, so adding a field here does NOT force touching the
// parser: it only has to be added in THE TWO places of this file (the list
// and the printf), which are next to each other on purpose.
// IMPORTANCE: CRITICAL -- it is the Pi's only sensory input.
// ######################################################################
const char* CAMPOS_CSV[] = {
    "t_ms",
    "dN","dE","dS","dO",            // filtered ToF, chassis frame, mm
    "pN","pE","pS","pO",            // 1 = there is a wall
    "nr0","nr1","nr2","nr3",        // Naka-Rushton
    "mr0","mr1","mr2","mr3",        // gaussian layer
    "ret","mem",
    "roll","pitch","yaw","yaw_ref","corr","cuad","ecen",   // cuad = grid heading, ecen = centering error
    "cm_tramo","celdas",
    "estado","cmd","ev","err_giro","imu_ok","hab",
    "rpm_DI","rpm_DD","rpm_TI","rpm_TD",
    "obj_DI","obj_DD","obj_TI","obj_TD",
    "pwm_DI","pwm_DD","pwm_TI","pwm_TD",
    "desp_DI","desp_DD","desp_TI","desp_TD",
    "agc_DI","agc_DD","agc_TI","agc_TD",
    "vivas","cortadas","ciclo_ms",
};
const int N_CAMPOS_CSV = sizeof(CAMPOS_CSV)/sizeof(CAMPOS_CSV[0]);

// PERIODO_CSV_MS is declared above, with the other rates (BLOCK 05).
unsigned long t_csv = 0;

void enviarCabecera(){
    Serial.print("C");
    for(int i=0;i<N_CAMPOS_CSV;i++) Serial.printf(",%s", CAMPOS_CSV[i]);
    Serial.println();
}

void enviarCSV(float d[4], bool pared[4]){
    int vivas=0, cortadas=0;
    for(int i=0;i<N_RUEDAS;i++){
        if(ruedas[i].vivo)      vivas    |= (1<<i);
        if(ruedas[i].desbocada) cortadas |= (1<<i);
    }
    Serial.printf("D,%lu,%.0f,%.0f,%.0f,%.0f,%d,%d,%d,%d,",
        millis(), d[0], d[1], d[2], d[3],
        pared[0]?1:0, pared[1]?1:0, pared[2]?1:0, pared[3]?1:0);
    Serial.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
        neuro.naka_act[0], neuro.naka_act[1], neuro.naka_act[2], neuro.naka_act[3],
        neuro.gauss_act[0],neuro.gauss_act[1],neuro.gauss_act[2],neuro.gauss_act[3],
        neuro.ret_act, neuro.mem_act);
    Serial.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,",
        rollActual, pitchActual, yawMedido, yawRef, correccion, rumboCuadricula(), errCentro);
    Serial.printf("%.1f,%d,%d,%d,%d,%.2f,%d,%d,",
        avanzando ? distanciaTramoCm() : 0.0f,
        avanzando ? celdasTramo() : 0,
        (int)estadoLoco, (int)comandoActivo, (int)eventoPendiente,
        err_giro, imu_ok?1:0, habilitado?1:0);
    for(int i=0;i<4;i++) Serial.printf("%.1f,", ruedas[i].rpm);
    for(int i=0;i<4;i++) Serial.printf("%.1f,", ruedas[i].objetivo);
    for(int i=0;i<4;i++) Serial.printf("%.0f,", ruedas[i].salida);
    for(int i=0;i<4;i++) Serial.printf("%.0f,", ruedas[i].pwmDespegue);
    for(int i=0;i<4;i++) Serial.printf("%d,", ruedas[i].agc);
    Serial.printf("%d,%d,%lu\n", vivas, cortadas, peorCiclo);

    eventoPendiente = EV_NADA;   // each event is reported only ONCE
    peorCiclo = 0;               // the worst cycle is "since the last CSV"
}

// ######################################################################
// BLOCK 23 -- BENCH COMMANDS (floor, not part of the protocol)
//   S readable status       C recalibrate the gyroscope bias
//   Z set the current yaw as the reference
//   V measure the encoder directions (robot ON A STAND)
//   E enable / disable the motors
//   G save the learned take-off PWMs
//   N resend the CSV header
// ######################################################################
void atenderComandoBanco(int c){
    switch(c){
        case 'S': case 's':
            Serial.printf("ST:yaw=%.2f,ref=%.2f,bias=%.4f,imu=%d,addr=0x%02X,resets=%lu,"
                          "jumps=%lu,hab=%d,state=%d,"
                          "kp=%.2f,ki=%.2f,kpy=%.2f,kdy=%.2f,corrmax=%.0f,syaw=%+.0f,fw=%s\n",
                          yawMedido, yawRef, biasGyroZ, imu_ok?1:0, dirImu,
                          (unsigned long)reiniciosImu, (unsigned long)saltosImu,
                          habilitado?1:0,
                          (int)estadoLoco, KP, KI, KP_YAW, KD_YAW, CORR_MAX, SIGNO_YAW,
                          FW_VERSION);
            for(int i=0;i<N_RUEDAS;i++)
                Serial.printf("ST_WHEEL:%s,sign=%+d,takeoff=%.0f,rpm=%.1f,agc=%d,"
                              "alive=%d,cut=%d\n",
                              ruedas[i].corto, ruedas[i].signoEncoder,
                              ruedas[i].pwmDespegue, ruedas[i].rpm, ruedas[i].agc,
                              ruedas[i].vivo?1:0, ruedas[i].desbocada?1:0);
            break;
        case 'C': case 'c':
            // With the BNO085 this no longer calibrates: it re-anchors the
            // heading at zero and measures the drift at rest. The letter and
            // the format are kept.
            frenarTodo(); habilitado = false;
            I2C_TOMAR();
            if(calibrarBiasGiroscopio(400)) Serial.printf("OK:calib,bias=%.4f\n", biasGyroZ);
            else                            Serial.println("ERR:calib_failed");
            I2C_SOLTAR();
            break;
        case 'Z': case 'z':
            // 🔑 RE-ANCHORS THE YAW AT ZERO (Sep 15 afternoon, requested on the
            // floor): the current heading becomes 0, reference 0, grid base 0
            // and quarters = 0. This way each run starts at yaw=0 and the
            // multiples of 90 are read directly in the interface (0, -90,
            // -180...). It is safe because `yaw` is integrated from the jumps
            // of yawCrudo, just like in calibrarBiasGiroscopio.
            I2C_TOMAR();
            yaw = 0; yawMedido = 0; yawRef = 0; yawBase = 0; cuartos = 0;
            integralYaw = 0; huboGiro = false; huboGiroCuadricula = false;
            cuadriculaActiva = true;
            I2C_SOLTAR();
            Serial.println("OK:ref,yaw=0.00,grid_base");
            break;
        case 'V': case 'v':
            pedidoCalibrar = true;      // done in the loop, not in here
            break;
        case 'E': case 'e':
            habilitado = !habilitado;
            if(!habilitado) frenarTodo();
            else for(int i=0;i<N_RUEDAS;i++){    // they are given another chance
                ruedas[i].desbocada = false; ruedas[i].sospechas = 0;
            }
            Serial.printf("OK:motors=%d\n", habilitado?1:0);
            break;
        case 'B': case 'b':
            // Factory take-offs. For when the learned one went wrong (an
            // encoder that did not see the wheel raises it to the cap).
            for(int i=0;i<N_RUEDAS;i++){ ruedas[i].pwmDespegue = DESPEGUE_INICIAL[i]; ruedas[i].suelo = 0; }
            guardarDespegues();
            Serial.printf("OK:factory_takeoffs,%.0f,%.0f,%.0f,%.0f\n",
                          ruedas[0].pwmDespegue, ruedas[1].pwmDespegue,
                          ruedas[2].pwmDespegue, ruedas[3].pwmDespegue);
            break;
        case 'G': case 'g':
            guardarDespegues();
            Serial.println("OK:takeoffs_saved");
            break;
        case 'N': case 'n':
            enviarCabecera();
            break;
        default: break;                 // garbage: silently ignored
    }
}

// ######################################################################
// BLOCK 24 -- DEBUG TELEMETRY (Serial2, towards the base)
// It does NOT take part in the control: it can be disconnected without the
// robot changing its behavior. It is also where the control task writes when
// it cuts off a wheel, so as not to put a long printf on the link with the Pi.
// ######################################################################
const unsigned long PERIODO_DEBUG_MS = 400;
unsigned long t_debug = 0;

const char* nombreEstado(){
    if(manual) return "MANUAL";
    switch(estadoLoco){
        case LOCO_IDLE:      return "IDLE";
        case LOCO_AVANZANDO: return "AVANZA";
        case LOCO_GIRANDO:   return "GIRA";
        case LOCO_CENTRANDO: return "CENTRA";
        case LOCO_CALIBRANDO:return "CALIBRA";
        case LOCO_MANUAL:    return "MANUAL";
        default:             return "BLOQUEADO";
    }
}
void enviarDebug(float d[4]){
    Serial2.printf("[%s] hab=%d F:%.0f R:%.0f B:%.0f L:%.0f yaw=%.1f ref=%.1f "
                   "corr=%.1f rpm %.0f/%.0f/%.0f/%.0f cm=%.1f\n",
        nombreEstado(), habilitado?1:0, d[0],d[1],d[2],d[3],
        yawMedido, yawRef, correccion,
        ruedas[0].rpm, ruedas[1].rpm, ruedas[2].rpm, ruedas[3].rpm,
        avanzando ? distanciaTramoCm() : 0.0f);
}

// ######################################################################
// BLOCK 27 -- REPROGRAMMING OVER WiFi (OTA)
// WHAT IT DOES: connects to the network with a fixed IP and leaves the ESP
//   listening for firmware uploads. Nothing else. The link with the Pi is
//   still the USB cable and the CSV; the network carries neither commands
//   nor telemetry.
//
// WHY: without this, each gain change forces disconnecting the battery,
//   connecting the USB, uploading, disconnecting the USB and putting the
//   battery back. With the car assembled that is half a session, and tuning
//   the control means twenty uploads.
//
// 🔴 THE FIRST THING IT DOES WHEN AN UPLOAD STARTS IS BRAKE AND DISABLE THE
//   MOTORS. Reprogramming with the wheels turning would leave the car moving
//   by itself during the ~15 s the write takes, with nobody running the loop.
//
// 🔑 FIXED IP ON PURPOSE (11.11.41.14, free on Sep 14; the Pi is .17 and the
//   laptop .13). With DHCP the address changes and each upload starts by
//   finding it out; this way 'pio run -e wifi -t upload' always works the
//   same.
//
// 🔴 THE WiFi LIVES ON CORE 0, the same as the control loop (BLOCK 18). That
//   is why power saving is turned off, which adds delays of tens of ms. Keep
//   an eye on `ciclo_ms` in the CSV: if the worst cycle rises compared with
//   what was measured without network, the network is stealing time from
//   the loop and one has to choose between the two. That is also why the
//   connection has an 8 s limit: if the network does not connect, the robot
//   starts anyway, without OTA.
// ######################################################################
const char* OTA_NOMBRE = "carro";
IPAddress   IP_CARRO (11, 11, 41, 14);
IPAddress   IP_PUERTA(11, 11, 41,  1);
IPAddress   IP_MASCARA(255, 255, 255, 0);
bool wifi_ok = false;

void conectarWifi(){
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);          // power saving delays the loop
    if(!WiFi.config(IP_CARRO, IP_PUERTA, IP_MASCARA))
        Serial2.println("[BOOT] could not set the static IP: continuing with the DHCP one");
    WiFi.begin(WIFI_RED, WIFI_CLAVE);

    // Bounded wait that does NOT block the boot longer than needed: the robot
    // has to be able to work with the network down.
    unsigned long limite = millis() + 8000;
    while(WiFi.status() != WL_CONNECTED && millis() < limite) delay(200);

    if(WiFi.status() != WL_CONNECTED){
        Serial.println("WARN:no_wifi,cannot_upload_by_ota");
        Serial2.println("[BOOT] no WiFi: the robot works the same, but it can only be flashed by cable");
        return;
    }
    wifi_ok = true;

    ArduinoOTA.setHostname(OTA_NOMBRE);
    ArduinoOTA.onStart([](){
        habilitado = false;
        ordenVx = ordenVy = ordenW = 0;
        pedidoVx = pedidoVy = pedidoW = 0;
        frenarTodo();
        Serial2.println("[OTA] WiFi upload: motors braked and disabled");
    });
    ArduinoOTA.onError([](ota_error_t e){
        frenarTodo();
        Serial2.printf("[OTA] upload failed (%u)\n", (unsigned)e);
    });
    ArduinoOTA.begin();

    // Through the Pi link with the WARN: prefix, which the interface already
    // knows how to show; the long detail goes through the debug link.
    Serial.printf("WARN:wifi_ok,ip=%s,rssi=%d\n",
                  WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
    Serial2.printf("[BOOT] WiFi %s, signal %d dBm -- it can be flashed with "
                   "'pio run -e wifi -t upload'\n",
                   WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
}

// ######################################################################
// BLOCK 25 -- SETUP
// IMPORTANCE: CRITICAL. The heading check takes ~2 s with the robot STILL:
// do not move it at boot.
// ######################################################################
void setup(){
    Serial.begin(115200);                                  // link with the Pi
    Serial2.begin(115200,SERIAL_8N1,RX_S3,TX_S3);          // debug

    for(int i=0;i<N_RUEDAS;i++){
        pinMode(ruedas[i].in1, OUTPUT); digitalWrite(ruedas[i].in1, LOW);
        pinMode(ruedas[i].in2, OUTPUT); digitalWrite(ruedas[i].in2, LOW);
        pinMode(ruedas[i].pwm, OUTPUT); digitalWrite(ruedas[i].pwm, LOW);
        ledcSetup(ruedas[i].canalPwm, FREQ_PWM, BITS_PWM);
        ledcAttachPin(ruedas[i].pwm, ruedas[i].canalPwm);
        ledcWrite(ruedas[i].canalPwm, 0);
    }

    mtxI2C = xSemaphoreCreateMutex();
    Wire.begin(SDA_BUS,SCL_BUS); Wire.setClock(400000);
    tcaCerrar();

    const int CANALES_TOF[4] = {CANAL_N, CANAL_E, CANAL_S, CANAL_O};
    for(int i=0;i<4;i++){ tcaSelect((uint8_t)CANALES_TOF[i]); lox.begin(); }
    tcaCerrar();

    recuperarSignos();
    recuperarDespegues();

    imu_ok = inicializarIMU();
    if(imu_ok) imu_ok = calibrarBiasGiroscopio(400);
    if(!imu_ok) Serial2.println("[BOOT] IMU BNO085 DOES NOT ANSWER at 0x4A or 0x4B: no heading, turns will fail");
    yaw = 0; yawMedido = 0; yawRef = 0;

    muestrearEncoders();           // first reading: sets anguloPrevio
    iniciarTramo();
    habilitado = false;            // the motors start OFF, always
    estadoLoco = LOCO_IDLE;

    // The network BEFORE starting the control task: connecting takes up to 8 s
    // and during that time the loop must not be running, so that the worst
    // cycle measured in the CSV is that of the robot working and not that of
    // the boot.
    conectarWifi();

    // The control task to core 0; loop() stays on core 1.
    xTaskCreatePinnedToCore(tareaControl, "control", 4096, NULL, 2, NULL, 0);

    enviarCabecera();
    Serial.printf("READY imu=%d bias=%.4f fields=%d fw=%s\n",   // bias = drift at rest, deg/s
                  imu_ok?1:0, biasGyroZ, N_CAMPOS_CSV, FW_VERSION);
    Serial2.println("[BOOT] ESP32 locomotion slave ready -- motors OFF ('E' enables them)");
}

// ######################################################################
// BLOCK 26 -- LOOP (core 1)
// WHAT IT DOES, IN A FIXED ORDER:
//   1. commands from the Pi    2. ToF + walls
//   3. neurocontroller         4. locomotion machine
//   5. CSV to the Pi / debug to the base
// The motor loop is NOT here: it lives on core 0 (BLOCK 18).
// IMPORTANCE: CRITICAL. Hard rule: no waiting while and no long delay inside
//   a state -- with two documented exceptions, blocking by design: the turn
//   (BLOCK 19) and the direction calibration (BLOCK 20).
// ######################################################################
void loop(){
    unsigned long ahora = millis();

    // --- 0. WiFi uploads ------------------------------------------------
    // Cheap when nobody is uploading. It goes FIRST in the cycle so that an
    // upload gets in even if the rest of the loop gets longer. During the
    // turn and the direction calibration, which are blocking, it is not
    // handled: one has to wait for them to end (at most 8 s) before flashing.
    if(wifi_ok) ArduinoOTA.handle();

    // --- 1. commands ----------------------------------------------------
    leerComandoPi();
    if(pedidoCalibrar){ pedidoCalibrar = false; calibrarSentidos(); }
    if(pedirGuardarDespegues){ pedirGuardarDespegues = false; guardarDespegues(); }
    if(ruedaCortada >= 0){ ruedaCortada = -1; eventoPendiente = EV_RUEDA_CORTADA; }

    // --- 2. perception (chassis frame) -----------------------------------
    leerUnSensor(dists);   // one per loop, rotating (BLOCK 15)
    for(int i=0;i<4;i++) pared[i]=esPared(DIRS4[i],dists[i]);

    // --- 3. neurocontroller (5 sub-steps) --------------------------------
    // Only when the round of the four ToF closes: this way its rate is
    // EXACTLY the one it had before (one pass per full round) even though the
    // loop runs 4x faster.
    if(rondaCompleta){
        bool hay_salida=false;
        for(int i=0;i<4;i++) if(!pared[i]){ hay_salida=true; break; }
        for(int s=0;s<5;s++) neuro.step(dists, hay_salida);
    }

    // --- 4. locomotion machine ---------------------------------------------
    if(hayComandoNuevo){
        hayComandoNuevo = false;
        int cmd = comandoRecibido;

        if(cmd == -1){                                   // retired command
            eventoPendiente = EV_NACK;
        }
        else if(cmd==CMD_PARAR){
            frenarTodo(); avanzando=false; comandoActivo=CMD_PARAR;
            estadoLoco = LOCO_IDLE; eventoPendiente = EV_ACK;
        }
        else if(cmd==CMD_CENTRAR){
            // '9' = settle (align + stick to the wall), see ejecutarAcomodo().
            // Without IMU or with the motors off: ERR_GIRO.
            if(!imu_ok || !habilitado) eventoPendiente = EV_ERR_GIRO;
            else { avanzando = false; ejecutarAcomodo(); ahora = millis(); }   // BLOCKS
        }
        else if(cmd==CMD_GIRO_DER || cmd==CMD_GIRO_IZQ || cmd==CMD_GIRO_180){
            if(!imu_ok || !habilitado){
                eventoPendiente = EV_ERR_GIRO;
            } else {
                comandoActivo = (cmd==CMD_GIRO_DER) ? CODCSV_GIRO_DER :
                                (cmd==CMD_GIRO_IZQ) ? CODCSV_GIRO_IZQ : CODCSV_GIRO_180;
                avanzando = false;
                // The turn ACK is not sent: it is blocking and the Pi only
                // waits for the closing event (7 or 8).
                int pasos90 = (cmd==CMD_GIRO_DER) ? +1 :
                              (cmd==CMD_GIRO_IZQ) ? -1 : +2;
                ejecutarGiro(pasos90);                   // BLOCKS here
                ahora = millis();
            }
        }
        else if(cmd==CMD_HASTA_PARED){
            // it refuses only if it is ALREADY where it would have to stop.
            // DIRECT reading: the filtered one drags the wall from before the
            // turn.
            I2C_TOMAR(); float dfA = corregirToF(0, leerToF(CANAL_N)); I2C_SOLTAR();
            // In a T (both sides open) 'A' stops at DIST_ARRIMAR: if it is
            // already there it refuses, instead of accepting and closing in the
            // same cycle (Sep 16: the PARED overwrote the ACK and the Pi kept
            // waiting 4 s, sent '0' and everything fell apart).
            bool enTA = dists[1] > UMBRAL_E && dists[3] > UMBRAL_O;
            float stopA = enTA ? DIST_ARRIMAR : DIST_STOP_N;
            if(dfA <= stopA + 5.0f || !habilitado){
                frenarTodo(); avanzando=false; comandoActivo=CMD_PARAR;
                estadoLoco = LOCO_IDLE; eventoPendiente = EV_NACK;
            } else {
                comandoActivo = CMD_HASTA_PARED;
                avanzando = true;
                iniciarTramo();
                t_ultimo_avance = ahora;
                cm_en_ultimo_chequeo = 0;
                confirma_fuera = 0;
                estadoLoco = LOCO_AVANZANDO;
                eventoPendiente = EV_ACK;
                tInicioTramo = ahora; tCicloAnterior = ahora; acumToque = 0; toqueHasta = 0;
                ordenVx = V_MINIMA_MS;      // the T_RAMPA_INI ramp rises from here
            }
        }
        else if(cmd==CMD_ALINEAR){
            if(!imu_ok || !habilitado) eventoPendiente = EV_ERR_GIRO;
            else { avanzando = false; ejecutarAlineacion(); ahora = millis(); }   // BLOCKS
        }
        else if(cmd==CMD_REVERSA){
            // it refuses only if the back is ALREADY at the stopping distance
            if(dists[2] <= DIST_STOP_S || !habilitado){
                frenarTodo(); avanzando=false; comandoActivo=CMD_PARAR;
                estadoLoco = LOCO_IDLE; eventoPendiente = EV_NACK;
            } else {
                comandoActivo = CMD_REVERSA;
                avanzando = true;
                iniciarTramo();
                t_ultimo_avance = ahora;
                cm_en_ultimo_chequeo = 0;
                confirma_fuera = 0;
                estadoLoco = LOCO_AVANZANDO;
                eventoPendiente = EV_ACK;
                tInicioTramo = ahora; tCicloAnterior = ahora; acumToque = 0; toqueHasta = 0;
                ordenVx = -V_MINIMA_MS;
            }
        }
        else if(cmd==CMD_AVANZAR){
            // 🔴 DIRECT READING OF THE FRONT SENSOR TO DECIDE THE REJECTION (Sep
            // 15 night): with pared[] (filtered, one sample per round) the '5'
            // the Pi sends right after the turn closes was rejected because of
            // the wall the robot had in front BEFORE turning. Measured: nack
            // after the turn and fin_celda on the retry, in three steps.
            I2C_TOMAR(); float df5 = corregirToF(0, leerToF(CANAL_N)); I2C_SOLTAR();
            if(df5 <= UMBRAL_N || !habilitado){
                frenarTodo(); avanzando=false; comandoActivo=CMD_PARAR;
                estadoLoco = LOCO_IDLE; eventoPendiente = EV_NACK;
            } else if(imu_ok && fabsf(rumboCuadricula() - yawMedido) > TOL_AVANCE_DEG){
                // 🔴 Tilted: no advance. The Pi sends 'Y' and asks again.
                frenarTodo(); avanzando=false; comandoActivo=CMD_PARAR;
                estadoLoco = LOCO_IDLE; eventoPendiente = EV_DESALINEADO;
                err_giro = rumboCuadricula() - yawMedido;
            } else {
                comandoActivo = CMD_AVANZAR;
                avanzando = true;
                iniciarTramo();
                t_ultimo_avance = ahora;
                cm_en_ultimo_chequeo = 0;
                confirma_fuera = 0;
                estadoLoco = LOCO_AVANZANDO;
                eventoPendiente = EV_ACK;
                tInicioTramo = ahora; tCicloAnterior = ahora; acumToque = 0; toqueHasta = 0;
                ordenVx = V_MINIMA_MS;      // the T_RAMPA_INI ramp rises from here
            }
        }
        else eventoPendiente = EV_NACK;
    }

    switch(estadoLoco){

        case LOCO_IDLE:
        case LOCO_CENTRANDO:
        case LOCO_GIRANDO:
        case LOCO_CALIBRANDO:
            ordenVx = 0; ordenVy = 0;
            break;

        case LOCO_MANUAL:
            // The commands are written by aplicarManual(); they are not touched
            // here.
            break;

        case LOCO_AVANZANDO: {
            float cm = distanciaTramoCm();

            // 🔴 4-0. SAFETY: a ToF below its threshold = hard brake. It closes
            // as PARED (or FIN_CELDA if it already passed half a cell), just
            // like the normal wall braking, and reports on which side.
            {
                int ls = ladoSeguridad(dists);
                if(ls){
                    frenarTodo();
                    static const char* NOM[5] = {"", "N", "E", "S", "O"};
                    Serial.printf("WARN:safety,side=%s,mm=%.0f,cm=%.1f\n", NOM[ls], dists[ls-1], cm);
                    bool media = (comandoActivo != CMD_HASTA_PARED) && (cm >= 0.5f * PASO_CELDA_CM);
                    eventoPendiente = media ? EV_FIN_CELDA : EV_PARED;
                    avanzando = false; comandoActivo = CMD_PARAR; estadoLoco = LOCO_IDLE;
                    break;
                }
            }

            // 4a. open field: it left the maze.
            if(dists[0]>=UMBRAL_FUERA && dists[1]>=UMBRAL_FUERA &&
               dists[3]>=UMBRAL_FUERA){                     // N, E, O (the S does not count)
                if(++confirma_fuera >= N_CONFIRMA_FUERA){
                    frenarTodo(); confirma_fuera=0; avanzando=false;
                    estadoLoco = LOCO_IDLE; comandoActivo = CMD_PARAR;
                    eventoPendiente = EV_CAMPO_ABIERTO;
                    break;
                }
            } else confirma_fuera = 0;

            // 4b. stall: there is a setpoint but the odometry does not advance.
            if(cm - cm_en_ultimo_chequeo > CM_MIN_VIVO){
                cm_en_ultimo_chequeo = cm;
                t_ultimo_avance = ahora;
            } else if(ahora - t_ultimo_avance > VENTANA_BLOQUEO_MS){
                frenarTodo(); avanzando=false;
                estadoLoco = LOCO_BLOQUEADO; comandoActivo = CMD_PARAR;
                eventoPendiente = EV_BLOQUEO;
                break;
            }

            // 4c. front wall: dedicated reading, without the filter delay.
            I2C_TOMAR();
            // 🔴 Corrected here by hand: this reading does NOT go through
            // leerUnSensor, so without this it would be the only raw one left
            // in the firmware and DIST_STOP_N, which is in real distance, would
            // not mean here the same as in the rest of the code.
            // In reverse the "wall in front" is the one BEHIND: same threshold.
            bool reversa = (comandoActivo == CMD_REVERSA);
            float df = reversa ? corregirToF(2, leerToF(CANAL_S))
                               : corregirToF(0, leerToF(CANAL_N));
            I2C_SOLTAR();
            // 🔑 ANTICIPATED STOP (tramo() of the script): it brakes when the
            // distance PREDICTED for LATENCIA from now falls below the
            // threshold, computed with the REQUESTED speed (exact), not
            // measured with the ToF (which gives zero between two refreshes).
            // In a T (both sides open) '5' and 'A' stop at DIST_ARRIMAR: the
            // center of the crossing cell. In a corner, at DIST_STOP_N. The
            // reverse looks at the rear sensor: DIST_STOP_S.
            bool enT = !reversa && dists[1] > UMBRAL_E && dists[3] > UMBRAL_O;
            float distStop = reversa ? DIST_STOP_S : (enT ? DIST_ARRIMAR : DIST_STOP_N);
            bool pared_adelante = (df - fabsf(pedidoVx) * 1000.0f * LATENCIA_FRENO_S <= distStop);
            // 🔑 the per-cell cut belongs to the cell advance; CMD_HASTA_PARED
            // goes up to the wall, with a safety cap in case there never is one.
            bool celda_lista = (comandoActivo == CMD_HASTA_PARED)
                             ? (cm >= TOPE_HASTA_PARED_CM)
                             : (cm >= PASO_CELDA_CM);

            if(pared_adelante || celda_lista){
                frenarTodo();
                // 🔑 Braking for a wall after traveling MORE THAN HALF A CELL
                // counts as a cell: the robot did move to the next one (the
                // last of the corridor, against the wall). PARED only if it
                // barely moved (Sep 15 night: the map fell behind).
                bool media = (comandoActivo != CMD_HASTA_PARED) && (cm >= 0.5f * PASO_CELDA_CM);
                eventoPendiente = (pared_adelante && !celda_lista && !media) ? EV_PARED : EV_FIN_CELDA;
                avanzando = false;
                comandoActivo = CMD_PARAR;
                estadoLoco = LOCO_IDLE;
                break;
            }

            // 4d. speed: tramo() of square_right.py. Cruise, T_RAMPA_INI ramp
            // when leaving, and PROGRESSIVE braking from EMPIEZA_FRENAR to the
            // stop (it arrives slowly, not in two steps). The neuro can still
            // brake more (the smaller of the two is taken).
            {
                float v = reversa ? fmaxf(V_MINIMA_MS, VEL_CRUCERO * FRAC_REVERSA) : VEL_CRUCERO;
                if(df < EMPIEZA_FRENAR_MM){
                    float frac = (df - distStop) / (EMPIEZA_FRENAR_MM - distStop);
                    frac = constrain(frac, 0.0f, 1.0f);
                    v = V_MINIMA_MS + (VEL_CRUCERO - V_MINIMA_MS) * frac;
                }
                float trans = (ahora - tInicioTramo) / 1000.0f;
                if(trans < T_RAMPA_INI_S){
                    float tope = V_MINIMA_MS + (VEL_CRUCERO - V_MINIMA_MS) * (trans / T_RAMPA_INI_S);
                    v = fmin(v, tope);
                }
                float g = neuro.gauss_act[IDX_FRENTE];
                float f = constrain(g, 0.35f, 1.0f);
                v = fmin(v, VEL_CRUCERO * f);
                v = fmax(v, V_MINIMA_MS);
                ordenVx = reversa ? -v : v;
            }

            // 4e. centering by TAPS: correccion() and _toque() of the script.
            // Direct readings of the side sensors (without the 800 ms filter).
            {
                float vy = 0.0f;
                float cicloMs = (float)(ahora - tCicloAnterior);
                tCicloAnterior = ahora;
                if(cicloMs > 300.0f) cicloMs = 100.0f;    // first cycle of the stretch
                errCentro = 0.0f;
                // 🔑 NO TAPS UNTIL AT CONSTANT SPEED (after the ramp):
                // correcting during the start made the robot oscillate between
                // walls and hesitate. Soft start, then it centers.
                bool enCrucero = (ahora - tInicioTramo) >= (unsigned long)(T_RAMPA_INI_S * 1000.0f);
                if(CENTRAR_ACTIVO > 0.5f && df > SIN_TRASLADAR_MM && enCrucero){
                    I2C_TOMAR();
                    float dE_ = corregirToF(1, leerToF(CANAL_E));
                    float dO_ = corregirToF(3, leerToF(CANAL_O));
                    I2C_SOLTAR();
                    bool d = dE_ < LIMITE_LATERAL_MM, i = dO_ < LIMITE_LATERAL_MM;
                    float err = 0.0f; int sentido = 0; bool urgente = false;
                    // first: move away if it is right on top of a wall
                    if(i && dO_ < MIN_LATERAL_MM){ vy = -VY_URGENTE_MS; urgente = true; errCentro = dO_ - MIN_LATERAL_MM; }
                    else if(d && dE_ < MIN_LATERAL_MM){ vy = +VY_URGENTE_MS; urgente = true; errCentro = MIN_LATERAL_MM - dE_; }
                    else if(d && i){ err = ((dE_ - dO_) - SESGO_LAT_MM) * 0.5f; sentido = (err > 0) ? -1 : +1; }
                    else if(d){ err = dE_ - CONSIGNA_E_MM; sentido = (err > 0) ? -1 : +1; }
                    else if(i){ err = dO_ - CONSIGNA_O_MM; sentido = (err > 0) ? +1 : -1; }
                    if(!urgente){
                        errCentro = err;
                        if(sentido != 0 && fabsf(err) > BANDA_LAT_MM){
                            // _toque(): the FREQUENCY is proportional to the
                            // error. The script added once per 100 ms cycle;
                            // here it is scaled to the real cycle.
                            acumToque += fmin(1.0f, fabsf(err) / ERR_PLENO_MM) * (cicloMs / 100.0f);
                            if(acumToque >= 1.0f){
                                acumToque -= 1.0f;
                                vyToque = VY_TOQUE_MS * sentido;
                                toqueHasta = ahora + TOQUE_MS;
                            }
                        }
                        if((int32_t)(toqueHasta - ahora) > 0) vy = vyToque;
                    }
                }
                ordenVy = vy;
            }
            break;
        }

        case LOCO_BLOQUEADO:
            // Sticky on purpose: it only leaves with a new command from the Pi.
            ordenVx = 0; ordenVy = 0;
            break;
    }

    // --- manual: dead man's switch ------------------------------------------
    if(manual){
        // 🔴 THE STOPPING THRESHOLD ALWAYS APPLIES, ALSO IN MANUAL (Sep 15
        // night): the pushes (exit confirmation, control, sticking) did not
        // look at the wall and the robot ran into it. Direct reading of the
        // sensor in the direction of motion; if it is closer than DIST_STOP_N
        // it brakes and leaves manual mode.
        if(habilitado && (fabsf(pedidoVx) > 1e-3f || fabsf(ordenVx) > 1e-3f)){
            bool haciaAdelante = (ordenVx > 0) || (fabsf(ordenVx) < 1e-3f && pedidoVx > 0);
            I2C_TOMAR();
            float dm = haciaAdelante ? corregirToF(0, leerToF(CANAL_N))
                                     : corregirToF(2, leerToF(CANAL_S));
            I2C_SOLTAR();
            if(dm <= (haciaAdelante ? DIST_STOP_N : DIST_STOP_S)){
                frenarTodo(); salirManual(); tiradaHasta = 0; frenando = false;
                Serial.printf("WARN:manual_wall,%s=%.0f\n", haciaAdelante ? "front" : "back", dm);
            }
        }
        // Per-side safety also in manual (lateral translations included).
        if(habilitado && estadoLoco == LOCO_MANUAL){
            int ls = ladoSeguridad(dists);
            if(ls){
                static const char* NOM[5] = {"", "N", "E", "S", "O"};
                frenarTodo(); salirManual(); tiradaHasta = 0; frenando = false;
                Serial.printf("WARN:safety,side=%s,mm=%.0f,manual\n", NOM[ls], dists[ls-1]);
            }
        }
        // With a push in progress its clock rules, not the dead man's switch:
        // it is exactly what prevents the network from cutting the motion
        // halfway.
        bool viva = tiradaHasta ? (int32_t)(tiradaHasta - ahora) > 0
                                : (ahora - tManual <= HOMBRE_MUERTO_MS);
        if(!viva){
            bool eraTirada = tiradaHasta != 0;
            // 🔑 IT DOES NOT LEAVE YET: zero is requested and the ramp is left
            // to go down while the loop keeps equalizing the four wheels.
            // Leaving here would turn the loop off with the car still rolling,
            // which is exactly what made it veer at the end.
            if(eraTirada && !frenando && (fabsf(pedidoVx) > 1e-3f ||
                                          fabsf(pedidoVy) > 1e-3f)){
                ordenVx = ordenVy = ordenW = 0;
                tiradaHasta = 0;
                frenando = true;
                tFrenado = ahora;
            } else {
                salirManual();
                Serial.println(eraTirada ? "OK:manual,end_of_push"
                                         : "OK:manual,dead_man");
            }
        }

        // 🔴 STALL IN MANUAL. The "there is a setpoint but the encoders do not
        // turn" watchdog only existed for the cell advance, so with the manual
        // control a car stuck against the wall kept pushing for the five
        // seconds: without any warning, and with the motors at locked rotor
        // drawing current. It is what happened on Sep 14 in three pushes in a
        // row, and from outside it looked like the software was failing.
        if(!frenando && habilitado){
            float pedidas = 0, reales = 0;
            for(int i=0;i<N_RUEDAS;i++){
                pedidas = fmax(pedidas, fabsf(ruedas[i].objetivo));
                reales  = fmax(reales,  fabsf(ruedas[i].rpm));
            }
            if(pedidas > 8.0f && reales < 3.0f){
                if(!tAtascado) tAtascado = ahora;
                else if(ahora - tAtascado > ATASCO_MS){
                    frenarTodo();
                    salirManual();
                    tAtascado = 0;
                    Serial.println("WARN:manual_stalled,setpoint_active_and_wheels_not_turning");
                    Serial2.println("[MANUAL] stalled: speed is requested and no wheel turns -- braking");
                }
            } else tAtascado = 0;
        } else tAtascado = 0;

        // Braking: it leaves when the request has reached zero (the four
        // wheels have come down together) or if the time runs out, so as not
        // to stay in manual because of a request that never finishes going down.
        if(frenando){
            bool parado = fabsf(pedidoVx) < 1e-3f && fabsf(pedidoVy) < 1e-3f;
            if(parado || ahora - tFrenado > FRENADA_TOPE_MS){
                frenando = false;
                salirManual();
                Serial.println("OK:manual,end_of_push");
            }
        }
    }

    // --- 5. links ---------------------------------------------------------------
    if(pedidoCsvInmediato || ahora - t_csv >= PERIODO_CSV_MS){
        pedidoCsvInmediato = false; t_csv = ahora;
        enviarCSV(dists, pared);
    }
    if(ahora - t_debug >= PERIODO_DEBUG_MS){ t_debug = ahora; enviarDebug(dists); }
}
