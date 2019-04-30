//#include <BasicLinearAlgebra.h>
#include <SPI.h>
#include <TDS_Sensors.h>
#include <bluefruit.h>
#include <Nffs.h>

//-----------file system file names---------//
#define FILENAME0    "/model0.txt"
#define FILENAME1    "/model1.txt"
#define FILENAME2    "/model2.txt"
#define FILENAME3    "/model3.txt"
#define FILENAME4    "/model4.txt"
#define FILENAME5    "/model5.txt"

//-----------erasing address definition---------//
#define STARTING_ADDR     0x06D000
#define SECTOR_SIZE       4096
#define ERASE_SIZE        (7*SECTOR_SIZE)

//----------all defines---------------------------//
#define BATT_VOLT      31     // ADC for the battery voltage P0.31  //31
#define CS1            27      //P0_1 sensor 1 left back      P0.11
#define CS2            30      //P0_2 sensor 2 left front     P0.07
#define CS3            15     //P0_3 sensor 3 right front    P0.15
#define CS4            16     //P0_4 sensor 4 right back     P0.16
#define CS5            7     // sensor LSM9DS1 magnetometer P0.27
#define CS6            11     // sensor LSM9DS1 accelerometer and gyroscope P0.30
#define DEN_AG         2     // LSM9DS1 DEN_AG pin //        P0.02  
#define Packet_Size    20
#define Sensor_Data_Packet 21
#define Sensor_Packet 42
#define cmd_packet 3
# define MODEL_BUFFER_SIZE 209

//--------------model values + calibration ----------------//

float b1[9];// = {0};    
float b2[9];// = {0};   
float b3[9];// = {0}; 
float b4[9];// = {0};  
float Beta[21];// = {0}; 
float W[12][21];// = {0};

/*
float b1[9] = {0.5703,-0.5324,0.5872,0.8664,-0.1442,-0.747,0.2421,1.0005,0.0492};

float b2[9] = {0.6224,-0.5109,0.5962,0.985,-0.0868,-0.7336,0.2485,0.9595,0.0244};

float b3[9] = {0.6018,-0.1272,-0.8779,-0.8382,0.1169,-1.1201,-0.1693,0.733,0.1215};

float b4[9] = {0.5975,-0.1323,-0.8627,-0.8127,0.1055,-1.0272,-0.1571,0.7203,0.1166};

float Beta[21] = {-4.58312,0.202345,-11.2836,11.6548,-3.03382,-0.538665,2.32483,4.63947,3.54282,-2.47223,0.399257,-0.59753,1.41276,-1.99017,-0.889193,7.3044,-3.44843,-0.361732,-2.57868,-0.814616,0.589099};

float W[12][21] = {{-0.119873,-0.483477,0.0828011,4.31748,-0.0402124,-0.216497,-0.0646153,0.264833,0.25067,0.026306,-0.0878456,0.437042,0.99329,0.0364468,-0.268301,1.48939,-0.0766191,-0.193391,-0.0888305,-0.297744,-0.105618},
{-0.397826,0.293947,-2.05232,5.90818,-0.171461,0.440849,0.274612,0.247178,0.507863,-0.0745556,0.298317,-0.430385,0.738313,-0.12452,0.604858,3.2407,-0.0787478,0.406841,-0.231743,0.360825,0.267626},
{0.192626,0.236207,1.99851,-5.39104,0.106577,-0.58154,0.00894673,-0.0810676,-0.35614,0.0364611,-0.377619,-0.00683516,-1.16448,-0.0225816,-1.05531,-2.70805,0.0535891,-0.517122,0.189474,-0.541626,-0.319575},
{-0.545146,-1.11713,-5.18116,4.88619,-0.18829,1.02723,-0.159464,0.488408,0.597215,0.0879984,0.781517,0.691337,1.53797,0.0817023,1.96728,3.43146,-0.125562,0.890114,-0.212594,1.0665,0.539748},
{-0.689316,5.48624,-12.891,0.486103,-0.382916,1.5828,1.71593,0.0255959,0.49137,-0.285255,1.08333,-4.0855,-5.3374,-0.949655,1.51518,4.36596,-0.0984744,1.45796,-0.29035,1.71824,0.883639},
{0.287504,-2.6073,5.74427,2.22645,0.175796,-0.817116,-0.880337,0.137028,-0.030357,0.185472,-0.554751,2.15319,2.8551,0.479066,-0.967332,-0.75425,0.0474211,-0.761076,0.0850837,-0.963261,-0.461072},
{1.15436,0.491195,-7.16101,0.230568,-0.758545,0.0330269,-0.477892,-1.66025,-0.795195,-2.91114,-0.213995,-0.628607,-0.745528,-0.634813,-0.016502,1.5567,-0.819475,0.0733188,-0.574401,0.0416114,0.227291},
{3.99727,-1.34947,6.39774,-11.6418,1.92645,-0.254343,-2.34954,-4.27958,-3.16549,0.65141,-0.77229,0.997138,-0.250604,1.45695,-0.232955,-5.76783,2.23769,-0.24665,1.72133,-0.0674965,-0.659654},
{-1.59088,0.370022,0.0719361,8.01381,-1.20993,0.137825,0.900028,1.5795,1.38803,-1.17572,0.398432,-0.466659,0.537972,-0.809052,0.0983601,3.50685,-1.38082,0.148592,-1.08709,0.0175826,0.419239},
{0.537768,0.144073,-1.65926,-2.77723,0.551454,-0.0090167,-0.22405,-0.642113,-0.469007,0.698298,-0.0858589,-0.104753,-0.624887,0.284834,-0.0132399,-0.819976,0.654153,-0.0123814,0.474447,0.0281787,-0.161722},
{0.716595,-0.992637,9.92016,-5.4873,0.860054,-0.156543,-0.581534,-0.368628,-0.716309,1.21344,-0.223589,0.978136,0.25013,0.698905,-0.138079,-4.52916,0.884487,-0.198552,0.784259,-0.0714149,-0.317709},
{-0.199639,0.187073,-6.74502,3.20599,-1.23371,0.0842176,0.109871,-0.288266,0.284084,-2.55698,0.0324044,-0.63381,0.260274,-0.805328,0.0218468,3.60203,-1.25277,0.153864,-1.02946,0.016275,0.398582}};
*/

//------------- end of model values + calibration -------------------//

//-----------------------HT parameters------------------------------------//


float Rotation[9] = { 0.10443,1.0123,-0.024483,1.0417,-0.17053,0.04383,0.097113,-0.072682,-0.97983};
float GyroOffset[3] = { 0.77101,1.7381,-1.2952};
float NoiseCovptr[16] = {0.038452,0.0026869,0.0026869,0.042119,
                        0.15096,-0.026302,-0.026302,0.34993,
                        0.0047394,5.1747e-05,5.1747e-05,0.0055787,
                        0.0031379,-0.00023904,-0.00023904,0.011377
};
float PCov[8] = {0, 1, 0, 1, 1, 0, 1, 0};


float previousdatasampleptr[4] = {0};

//---------------HT parameters end here--------------------------------//

//--------some global variable--------------//
//BLEUart bleuart;
//BLEService TDSservice = BLEService(0x1234);
//BLECharacteristic sensorPack1 = BLECharacteristic(0x0001);
//BLECharacteristic sensorPack2 = BLECharacteristic(0x0002);
//BLECharacteristic Parameter = BLECharacteristic(0x0004);

//uint8_t buf1[Packet_Size];                      // packet to be transmitted
//uint8_t buf2[Packet_Size];                      // packet to be transmitted
int8_t cmd[cmd_packet];
uint8_t ucmd [cmd_packet];
//uint16_t sensor[Sensor_Data_Packet];
int16_t SEN_OUT[Sensor_Data_Packet];
//uint16_t sensor1[10];
//uint16_t sensor2[10];
uint8_t state = 0;
uint8_t senserial[Sensor_Packet];

//---------------variables for model transfer------------------------------//
int PktNumber = 0;
int countM = 0;
// Packet buffer
uint8_t ModelPkt0[MODEL_BUFFER_SIZE];
uint8_t ModelPkt1[MODEL_BUFFER_SIZE];
uint8_t ModelPkt2[MODEL_BUFFER_SIZE];
uint8_t ModelPkt3[MODEL_BUFFER_SIZE];
uint8_t ModelPkt4[MODEL_BUFFER_SIZE];
uint8_t ModelPkt5[MODEL_BUFFER_SIZE];

union {
  float f;        
  uint8_t b[4];
} u;

NffsFile file;
//---------------global variable ends here---------//

//------------ local function definition----------//
void calibrate(float *test_Data);
uint8_t vote (float *dec_values);
float dot(float* px, float* py, uint8_t len);
uint8_t classification (float *sensordata);


//-----------state machine functions-----------//
void StateData(void);
void StateCmd(void);


void serialData(void);
void serialCmd(void);
void receiveModel(void);
void convertModel(void);
void eraseModel(void);
void saveModel(void);
void(* resetFunc) (void) = 0;//declare reset function at address 0

//-----------BLE functions----------//
void setupAdv(void);
void setupTDS(void);

//-----------HT functions-------------------------//
void mtdsScale (float *datasampleptr);
void mtdsRotate(float *imudata);
void matmul21(float *res, float *a, float *b);
void matmul22(float *res, float *a, float *b);
void matinv22(float *res, float *a);
void mattp22(float *res, float *a);
void matmul31(float *res, float *a, float *b);
void mtdsMouseMovementModel(uint8_t *mouse, float *pitchroll);
void mtdsRemoveGyroOffset(float *datasampleptr);
void mtdsGetPitchRollUnfiltered(float *datasampleptr);
void mtdsGetPitchRollKalman(float *datasampleptr, float *previousdatasampleptr);
void mTDSHT(float *imudata, uint8_t *mouse);
/*****************local function definition ends here************/


void setup() {
  Serial.begin(115200);
  //Serial.begin(921600);
  //Serial.println("Starting mTDS...\n");
  setupPins();      // setup TDS pins for the data captures
  pinMode(LED_BUILTIN, OUTPUT);

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI.begin();  // initialize SPI:

  Init_Mag_Sensors();     // initialize magnetometers
  InitLSM9DS1();          // initialize LSM9DS1

  // for file system
  Bluefruit.begin();

  // Initialize Nffs
  Nffs.begin();

  convertModel();
  
  /*
    Bluefruit.begin();
    Bluefruit.setName("mTDS_BLE");

    // setup TDS service
    setupTDS();
    //bleuart.begin();

    // Set up the advertising packet
    setupAdv();

    // Start advertising
    Bluefruit.Advertising.start();
  */
}

void loop() {

  //-------------all testing codes goes here----------------------------------------//

  /*
    // for LED testing
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(10);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(10);                       // wait for a second
  */


  /*
    // for classification testing
     //float sensor1[15] = {1641,1269,1648,-113,409,1770,-947,280,447,-1365,84,551,-141,1336,-4099}; //1
    // float sensor2[15] = {52,76,1660,-280,281,1453,-1075,530,512,-1927,1213,352,-173,1339,-4114}; //2
    // float sensor3[15] = {-267,-964,2214,-602,-186,1410,-954,170,280,-1199,-120,336,-165,1362,-4121}; //3
    // float sensor4[15] = {-56,-144,1288,-212,171,1222,-559,53,369,-566,-442,1650,-158,1365,-3872}; //4
     float sensor5[15] = {-6915,-1059,-13573,2696,-276,248,-891,229,399,-1208,-30,535,-180,1361,-4123}; //5
    // float sensor6[15] = {-21,-83,1537,-292,209,1387,-2692,-593,-1127,2303,-3096,-5108,-144,1321,-4106}; //6
    // float sensor7[15] = {-171,-368,1593,-318,52,1326,-859,116,360,-1071,-219,635,-176,1329,-4116}; //7

    for (uint8_t i = 0; i < 15; i++)
    {
      sensordata[i] = ((float)sensor5[i] * 10) / 32768;
    }

    // classification printout and testing
    Serial.printf("%d\n", (int)classification (sensordata));    // classification result printing
  */

  /*
    // wireless data TX testing
    for (uint8_t t = 0; t < 10; t++)
    {
     sensor1[t] = t;
     sensor2[t] = t + 10;
    }

    uint8_t j = 0;
    for (uint8_t i = 0; i < Packet_Size; i = i + 2)
    {
     buf1[i] = (byte)(sensor1[j] >> 8);
     buf1[i + 1] =  (byte)(sensor1[j]);
     buf2[i] = (byte)(sensor2[j] >> 8);
     buf2[i + 1] =  (byte)(sensor2[j]);
     j++;

    }

      sensorPack1.write(buf1,Packet_Size);
      sensorPack2.write(buf2,Packet_Size);
  */

  //-------------all testing codes ends here----------------------------------------//



  //----------debugging-------------//
  //Serial.flush();
  //Serial.printf("%.2f %.2f %.2f %.2f %.2f %.2f\n", imudata[0], imudata[1], imudata[2], imudata[3], imudata[4], imudata[5]);
  //Serial.printf("ax = %.2f ay = %.2f az = %.2f\n", imudata[0], imudata[1], imudata[2]);
  //Serial.printf("Pitch = %.2f Roll = %.2f\n", imudata[6], imudata[7]);
  //Serial.printf("x = %d y = %d\n", (int8_t)mouse[0], (int8_t)mouse[1]);
  //delay(50);

  // Serial.printf("TC = %d Roll = %d Pitch = %d\n", cmd[0], cmd[1], cmd[2]);
  // delay(50);
  //-----------debugging ends here-----------//

  state = Serial.read();

  

  // let's run the state machine

  switch (state)
  {
    case 11:
    {
      
      StateData();          // collecting data from sensors
      serialData();         // sending data to serial monitor
      state = 0;
      break;
    }
      

    case 22:
    {
      StateData();        // command needs data collection first
      StateCmd();         // then data processing
      serialCmd();        // then sending commands
      state = 0;
      break;
    }
      

    case 33:
    {
      //eraseModel();
      Serial.flush();
      receiveModel();
      saveModel();
     // resetFunc(); //call reset
    //  Serial.flush();
     // convertModel();       // Load transferred model values
      
      // debugging
//      Serial.printf("%f %f %f %f %f %f %f %f %f\n", b1[0],b1[1],b1[2],b1[3],b1[4],b1[5],b1[6],b1[7],b1[8]);
//      Serial.printf("%f %f %f %f %f %f %f %f %f\n", b2[0],b2[1],b2[2],b2[3],b2[4],b2[5],b2[6],b2[7],b2[8]);
//      Serial.printf("%f %f %f %f %f %f %f %f %f\n", b3[0],b3[1],b3[2],b3[3],b3[4],b3[5],b3[6],b3[7],b3[8]);
//      Serial.printf("%f %f %f %f %f %f %f %f %f\n", b4[0],b4[1],b4[2],b4[3],b4[4],b4[5],b4[6],b4[7],b4[8]);
//      Serial.printf("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", 
//      Beta[0],Beta[1],Beta[2],Beta[3],Beta[4],Beta[5],Beta[6],Beta[7],Beta[8],Beta[9],
//      Beta[10],Beta[11],Beta[12],Beta[13],Beta[14],Beta[15],Beta[16],Beta[17],Beta[18],Beta[19],Beta[20]);
//      for (int i = 0; i< 12; i++)
//      {
//        Serial.printf("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", 
//      W[i][0],W[i][1],W[i][2],W[i][3],W[i][4],W[i][5],W[i][6],W[i][7],W[i][8],W[i][9],
//      W[i][10],W[i][11],W[i][12],W[i][13],W[i][14],W[i][15],W[i][16],W[i][17],W[i][18],W[i][19],W[i][20]);
//      }
      // debugging ends here
      
      state = 0;
      break;
    }
      

    default:
    break;
  }


}


//-----------Data processing state machine---------------------//
void StateData(void)
{
  // get all the sensor data
  GetSensorOutput();

  // prepare sensor data to USB for calibration and training
  uint8_t j = 0;
  for (uint8_t i = 0; i < Sensor_Packet; i = i + 2)
  {
    senserial[i] = (byte)(SEN_OUT[j] >> 8);  //sensordata[i]); MSB
    senserial[i + 1] = (byte)SEN_OUT[j];   // LSB

    // making the wireless packet to send out the data

    //    if (j < 10)
    //    {
    //      buf1[i] = senserial[i];
    //      buf1[i + 1] = senserial[i + 1];
    //    }
    //    else //if (j>=10 & j<20)
    //    {
    //      buf2[i - 20] = senserial[i];
    //      buf2[i + 1 - 20] = senserial[i + 1];
    //    }

    j++;
  }

  //  sensorPack1.write(buf1, Packet_Size);   // wireless packet transfer
  //  sensorPack2.write(buf2, Packet_Size);   // wireless packet transfer

}

//------------command processing state machine--------------------//

void StateCmd(void)
{
  float sensordata[15] = {0};
  float imudata[12] = {0};      // for HT processing
  uint8_t mouse[2] = {0};

  // scale all the sensor data
  for (uint8_t i = 0; i < 21; i++)
  {
    if (i < 15)
    {
      sensordata[i] = (float)SEN_OUT[i] * 0.000305;
      //sensordata[i] = ((float)sensor[i] * 10) / 32768;
    }
    else if (i >= 15 & i < 18)
    {
      imudata[i - 15] = (float) SEN_OUT[i] * 0.000061;
    }
    else
    {
      imudata[i - 15] = (float) SEN_OUT[i] * 0.0076;
    }
  }

  ucmd[0] = classification (sensordata);    // machine learning to find the tongue commands
  mTDSHT(imudata, mouse);

  ucmd[1] = (uint8_t)cmd[1];
  ucmd[2] = (uint8_t)cmd[2];

}


//-------------Data send after collecting state machine---------------//

void serialData(void)
{
  Serial.write(senserial, Sensor_Packet);
  delay(10);  // wait for 10ms to get the next data

}

//------------command sending after processing------//
void serialCmd(void)
{
  Serial.write(ucmd, cmd_packet);
  delay(10);

}



//----------setup BLE---------------------//
/*
  void setupAdv(void)
  {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(TDSservice);
  // Bluefruit.Advertising.addService(bleuart);

  // There is no room for 'Name' in the Advertising packet
  // Use the optional secondary Scan Response packet for 'Name' instead
  Bluefruit.ScanResponse.addName();
  }
*/
//------------end of BLE setup----------------------------//


//----------setup BLE---------------------//
/*
  void setupTDS(void)
  {
  TDSservice.begin();

  // setup sensor packet properties 1
  sensorPack1.setProperties(CHR_PROPS_READ);
  sensorPack1.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  sensorPack1.setFixedLen(Packet_Size);
  //sensorPack1.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  sensorPack1.begin();
  // uint8_t sensor1[Packet_Size] = { 0 }; // Set the characteristic to use 8-bit values, with the sensor connected and detected
  sensorPack1.write(buf1, Packet_Size);                   // Use .notify instead of .write!

  // setup sensor packet properties 2
  sensorPack2.setProperties(CHR_PROPS_READ);
  sensorPack2.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  sensorPack2.setFixedLen(Packet_Size);
  //sensorPack2.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  sensorPack2.begin();
  // uint8_t sensor2[20] = { 0 }; // Set the characteristic to use 8-bit values, with the sensor connected and detected
  sensorPack2.write(buf2, Packet_Size);                   // Use .notify instead of .write!


  //    // setup parameter properties
  //    Parameter.setProperties(CHR_PROPS_WRITE);
  //    Parameter.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  //    Parameter.setFixedLen(3);
  //    //sensorPack2.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  //    Parameter.begin();
  //    //  Parameter.read(sensor2, 20);


  }
*/
//------------end of BLE setup----------------------------//



/******************* calibration********************************/

void calibrate(float *test_Data)
{
//      Serial.printf("%f %f %f %f %f %f %f %f %f\n", b1[0],b1[1],b1[2],b1[3],b1[4],b1[5],b1[6],b1[7],b1[8]);
//      Serial.printf("%f %f %f %f %f %f %f %f %f\n", b2[0],b2[1],b2[2],b2[3],b2[4],b2[5],b2[6],b2[7],b2[8]);
//      Serial.printf("%f %f %f %f %f %f %f %f %f\n", b3[0],b3[1],b3[2],b3[3],b3[4],b3[5],b3[6],b3[7],b3[8]);
//      Serial.printf("%f %f %f %f %f %f %f %f %f\n", b4[0],b4[1],b4[2],b4[3],b4[4],b4[5],b4[6],b4[7],b4[8]);

  // new EMF cancellation
  *(test_Data + 0) = *(test_Data + 0) - (*(test_Data + 12) * (*(b1 + 0)) + * (test_Data + 13) * (*(b1 + 3)) + * (test_Data + 14) * (*(b1 + 6)));
  *(test_Data + 1) = *(test_Data + 1) - (*(test_Data + 12) * (*(b1 + 1)) + * (test_Data + 13) * (*(b1 + 4)) + * (test_Data + 14) * (*(b1 + 7)));
  *(test_Data + 2) = *(test_Data + 2) - (*(test_Data + 12) * (*(b1 + 2)) + * (test_Data + 13) * (*(b1 + 5)) + * (test_Data + 14) * (*(b1 + 8)));

  *(test_Data + 3) = *(test_Data + 3) - (*(test_Data + 12) * (*(b2 + 0)) + * (test_Data + 13) * (*(b2 + 3)) + * (test_Data + 14) * (*(b2 + 6)));
  *(test_Data + 4) = *(test_Data + 4) - (*(test_Data + 12) * (*(b2 + 1)) + * (test_Data + 13) * (*(b2 + 4)) + * (test_Data + 14) * (*(b2 + 7)));
  *(test_Data + 5) = *(test_Data + 5) - (*(test_Data + 12) * (*(b2 + 2)) + * (test_Data + 13) * (*(b2 + 5)) + * (test_Data + 14) * (*(b2 + 8)));

  *(test_Data + 6) = *(test_Data + 6) - (*(test_Data + 12) * (*(b3 + 0)) + * (test_Data + 13) * (*(b3 + 3)) + * (test_Data + 14) * (*(b3 + 6)));
  *(test_Data + 7) = *(test_Data + 7) - (*(test_Data + 12) * (*(b3 + 1)) + * (test_Data + 13) * (*(b3 + 4)) + * (test_Data + 14) * (*(b3 + 7)));
  *(test_Data + 8) = *(test_Data + 8) - (*(test_Data + 12) * (*(b3 + 2)) + * (test_Data + 13) * (*(b3 + 5)) + * (test_Data + 14) * (*(b3 + 8)));

  *(test_Data + 9) =  *(test_Data + 9) - (*(test_Data + 12) * (*(b4 + 0)) + * (test_Data + 13) * (*(b4 + 3)) + * (test_Data + 14) * (*(b4 + 6)));
  *(test_Data + 10) = *(test_Data + 10) - (*(test_Data + 12) * (*(b4 + 1)) + * (test_Data + 13) * (*(b4 + 4)) + * (test_Data + 14) * (*(b4 + 7)));
  *(test_Data + 11) = *(test_Data + 11) - (*(test_Data + 12) * (*(b4 + 2)) + * (test_Data + 13) * (*(b4 + 5)) + * (test_Data + 14) * (*(b4 + 8)));

}


/******************************************************************************/

/************* voting mechanism******************************/
uint8_t vote (float *dec_values)
{
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t nr_class = 7;
  // uint8 p = 0;
  uint8_t vote[7] = {0};
  uint8_t vote_max_idx = 0;
  //for(i=1;i<nr_class;i++)

  /*       for(i=0;i<nr_class;i++)
        {
            vote[i] = 0;
        }

  */
  for (i = 0; i < nr_class; i++)
  {
    for ( j = i + 1; j < nr_class; j++)
    {

      if ( *(dec_values) > 0)
      {
        ++vote[i];
      }
      else
      {
        ++vote[j];
      }
      dec_values++;
    }
  }


  for (i = 1; i < nr_class; i++)
  {
    if (vote[i] > vote[vote_max_idx])
    {
      vote_max_idx = i;
    }
  }

  return vote_max_idx;

}



/*************************end of voting**********************************/

/******************** dot product of the values********************/

float dot(float* px, float* py, uint8_t len)
{
  uint8_t i;
  float sum = 0;
  for (i = 0; i < len; i++)
  {
    sum += (*px) * (*py);
    ++px;
    ++py;
  }
  return sum;
}

/***************end of the dot product**********************/

/***************** classification**********************/

uint8_t classification (float *sensordata)
{

  //calibrate sensor//
  calibrate(sensordata);
  /*
    // debugging
    for (uint8_t i=0; i<15; i++){
      Serial.printf("%f ", sensordata[i]);
    }
    Serial.printf("\n");

    // end of debugging
  */

  //dot product and remove bias/

  uint8_t a, b;
  float res[21] = {0};
  float wt[12] = {0};

  for (b = 0; b < 21 ; b++)
  {

    for (a = 0; a < 12; a++)
    {
      wt[a] = W[a][b];
    }

//    Serial.printf("%f %f %f %f %f %f %f %f %f %f %f %f\n", 
//      wt[0],wt[1],wt[2],wt[3],wt[4],wt[5],wt[6],wt[7],wt[8],wt[9],wt[10],wt[11]);

    res[b] = dot(wt, sensordata, 12) - Beta[b];
  }

  //voting//


  //decision to the output//
  return vote (res);

}

/********** end of classification************/

void setupPins(void)
{
  pinMode(CS1, OUTPUT);       // sensor 1
  pinMode(CS2, OUTPUT);       // sensor 2
  pinMode(CS3, OUTPUT);       // sensor 3
  pinMode(CS4, OUTPUT);       // sensor 4
  pinMode(CS5, OUTPUT);       // sensor 5
  pinMode(CS6, OUTPUT);       // sensor 6
  pinMode(DEN_AG, OUTPUT);    // den for accelerometer/gyroscope
  pinMode(BATT_VOLT, INPUT);  // battery voltage information capture
}

void Init_Mag_Sensors(void)
{
  /**********************************Configure SPI******************************************/

  /***********************************end of Configure SPI****************************************/
  CS_DISABLED(CS1);
  CS_DISABLED(CS2);
  CS_DISABLED(CS3);
  CS_DISABLED(CS4);
  CS_DISABLED(CS5);
  CS_DISABLED(CS6);
  // CNTRL register for the magnetometer values


  mgtWriteReg(MGT_CTRL1 , MGT_CTRL1_VAL, 1);
  mgtWriteReg(MGT_CTRL1 , MGT_CTRL1_VAL, 2);
  mgtWriteReg(MGT_CTRL1 , MGT_CTRL1_VAL, 3);
  mgtWriteReg(MGT_CTRL1 , MGT_CTRL1_VAL, 4);


  mgtWriteReg(MGT_CTRL2 , MGT_CTRL2_VAL, 1);
  mgtWriteReg(MGT_CTRL2 , MGT_CTRL2_VAL, 2);
  mgtWriteReg(MGT_CTRL2 , MGT_CTRL2_VAL, 3);
  mgtWriteReg(MGT_CTRL2 , MGT_CTRL2_VAL, 4);


  mgtWriteReg(MGT_CTRL3 , MGT_CTRL3_VAL, 1);
  mgtWriteReg(MGT_CTRL3 , MGT_CTRL3_VAL, 2);
  mgtWriteReg(MGT_CTRL3 , MGT_CTRL3_VAL, 3);
  mgtWriteReg(MGT_CTRL3 , MGT_CTRL3_VAL, 4);

  mgtWriteReg(MGT_CTRL4 , MGT_CTRL4_VAL, 1);
  mgtWriteReg(MGT_CTRL4 , MGT_CTRL4_VAL, 2);
  mgtWriteReg(MGT_CTRL4 , MGT_CTRL4_VAL, 3);
  mgtWriteReg(MGT_CTRL4 , MGT_CTRL4_VAL, 4);

  mgtWriteReg(MGT_CTRL5 , MGT_CTRL5_VAL, 1);
  mgtWriteReg(MGT_CTRL5 , MGT_CTRL5_VAL, 2);
  mgtWriteReg(MGT_CTRL5 , MGT_CTRL5_VAL, 3);
  mgtWriteReg(MGT_CTRL5 , MGT_CTRL5_VAL, 4);

  mgtWriteReg(MGT_CTRL6 , MGT_CTRL6_VAL, 1);
  mgtWriteReg(MGT_CTRL6 , MGT_CTRL6_VAL, 2);
  mgtWriteReg(MGT_CTRL6 , MGT_CTRL6_VAL, 3);
  mgtWriteReg(MGT_CTRL6 , MGT_CTRL6_VAL, 4);

  mgtWriteReg(MGT_CTRL7 , MGT_CTRL7_VAL, 1);
  mgtWriteReg(MGT_CTRL7 , MGT_CTRL7_VAL, 2);
  mgtWriteReg(MGT_CTRL7 , MGT_CTRL7_VAL, 3);
  mgtWriteReg(MGT_CTRL7 , MGT_CTRL7_VAL, 4);

  mgtWriteReg(MGT_INT_CTRL_M , MGT_INT_CTRL_M_VAL, 1);
  mgtWriteReg(MGT_INT_CTRL_M , MGT_INT_CTRL_M_VAL, 2);
  mgtWriteReg(MGT_INT_CTRL_M , MGT_INT_CTRL_M_VAL, 3);
  mgtWriteReg(MGT_INT_CTRL_M , MGT_INT_CTRL_M_VAL, 4);
}



/*******************************LSM9DS1 initialization********************************/
void InitLSM9DS1(void)
{

  CS_DISABLED(CS1);
  CS_DISABLED(CS2);
  CS_DISABLED(CS3);
  CS_DISABLED(CS4);
  CS_DISABLED(CS5);
  CS_DISABLED(CS6);


  /* init for the Gyroscope configuration*/
  LSM9DWriteReg(LSM9DS1_CTRL_REG1_G, LSM9DS1_CTRL_REG1_G_VAL, 2);
  LSM9DWriteReg(LSM9DS1_CTRL_REG2_G, LSM9DS1_CTRL_REG2_G_VAL, 2);
  LSM9DWriteReg(LSM9DS1_CTRL_REG3_G, LSM9DS1_CTRL_REG3_G_VAL, 2);
  LSM9DWriteReg(LSM9DS1_CTRL_REG4, LSM9DS1_CTRL_REG4_VAL, 2);
  LSM9DWriteReg(LSM9DS1_ORIENT_CFG_G, LSM9DS1_ORIENT_CFG_G_VAL, 2);

  /* end of initialization gyroscoper*/

  /* init for accelerometer */
  LSM9DWriteReg(LSM9DS1_CTRL_REG5_XL, LSM9DS1_CTRL_REG5_XL_VAL, 2);
  LSM9DWriteReg(LSM9DS1_CTRL_REG6_XL, LSM9DS1_CTRL_REG6_XL_VAL, 2);
  LSM9DWriteReg(LSM9DS1_CTRL_REG7_XL, LSM9DS1_CTRL_REG7_XL_VAL, 2);

  /* end of accelerometer initialization */

  /* initialize LSM9DS1 magnetometer */
  DISABLED(DEN_AG);
  LSM9DWriteReg(LSM9DS1_CTRL_REG1_M, LSM9DS1_CTRL_REG1_M_VAL, 1);
  LSM9DWriteReg(LSM9DS1_CTRL_REG2_M, LSM9DS1_CTRL_REG2_M_VAL, 1);
  LSM9DWriteReg(LSM9DS1_CTRL_REG3_M, LSM9DS1_CTRL_REG3_M_VAL, 1);
  LSM9DWriteReg(LSM9DS1_CTRL_REG4_M, LSM9DS1_CTRL_REG4_M_VAL, 1);
  LSM9DWriteReg(LSM9DS1_CTRL_REG5_M, LSM9DS1_CTRL_REG5_M_VAL, 1);

  /* end of init magnetometer  */

}




void GetSensorOutput(void)
{
  uint8_t k = 0;
  int16_t *p, *q, *r, *s, *v, *w, *x, *y, *t, *u, *z ;


  // delay_us(500);
  p = mgtReadAcc(1);
  //delay_us(10);
  SEN_OUT[k++] = p[0];
  SEN_OUT[k++] = p[1];
  SEN_OUT[k++] = p[2];


  q = mgtReadAcc(2);
  //delay_us(10);
  SEN_OUT[k++] = q[0];
  SEN_OUT[k++] = q[1];
  SEN_OUT[k++] = q[2];


  r = mgtReadAcc(3);
  //delay_us(10);
  SEN_OUT[k++] = r[0];
  SEN_OUT[k++] = r[1];
  SEN_OUT[k++] = r[2];


  s = mgtReadAcc(4);
  //delay_us(10);
  SEN_OUT[k++] = s[0];
  SEN_OUT[k++] = s[1];
  SEN_OUT[k++] = s[2];

  z = ReadLSM9DS1(1, LSM9DS1_OUT_X_L_M);                  // reading LSM9D magnetometer
  //delay_us(10);
  SEN_OUT[k++] = z[0];
  SEN_OUT[k++] = z[1];
  SEN_OUT[k++] = z[2];

  /*
       v = LSM3DAcc(1);
       //delay_us(10);
       SEN_OUT[k++]= (int) v[0];
       SEN_OUT[k++]= (int) v[1];
       SEN_OUT[k++]= (int) v[2];

       w = LSM3DAcc(2);
       //delay_us(10);
       SEN_OUT[k++]= (int) w[0];
       SEN_OUT[k++]= (int) w[1];
       SEN_OUT[k++]= (int) w[2];


       x = LSM3DAcc(3);
       //delay_us(10);
       SEN_OUT[k++]= (int) x[0];
       SEN_OUT[k++]= (int) x[1];
       SEN_OUT[k++]= (int) x[2];


       y = LSM3DAcc(4);
       //delay_us(10);
       SEN_OUT[k++]= (int) y[0];
       SEN_OUT[k++]= (int) y[1];
       SEN_OUT[k++]= (int) y[2];
  */

  t = ReadLSM9DS1(2, LSM9DS1_OUT_X_L_XL);                 // reading accelerometer
  //delay_us(10);
  SEN_OUT[k++] = t[0];
  SEN_OUT[k++] = t[1];
  SEN_OUT[k++] = t[2];

  u = ReadLSM9DS1(2, LSM9DS1_OUT_X_L_G);                  // reading gyroscope
  //delay_us(10);
  SEN_OUT[k++] = u[0];
  SEN_OUT[k++] = u[1];
  SEN_OUT[k] = u[2];




}



/**************************************************************************//**
  @fn       mgtWriteReg(uint8 reg, uint8 val)

  @brief    Write one byte to a sensor register

  @param    reg     Register address
  @param    val     Value to write
  @param    sennum  choose the sensor

  @return   void
******************************************************************************/
void mgtWriteReg(uint8_t reg, uint8_t val, uint8_t sennum)
{
  switch (sennum) {
    case 1:
      CS_ENABLED(CS1);
      break;
    case 2:
      CS_ENABLED(CS2);
      break;
    case 3:
      CS_ENABLED(CS3);
      break;
    case 4:
      CS_ENABLED(CS4);
      break;
    default:
      break;
  }
  spiWriteByte(reg);      // Write address
  spiWriteByte(val);      // Write value
  switch (sennum) {
    case 1:
      CS_DISABLED(CS1);
      break;
    case 2:
      CS_DISABLED(CS2);
      break;
    case 3:
      CS_DISABLED(CS3);
      break;
    case 4:
      CS_DISABLED(CS4);
      break;
    default:
      break;
  }
}



/**************************************************************************//**
  @fn       LSM9DWriteReg(uint8 reg, uint8 val, uint8 sennum)

  @brief    Write one byte to a sensor register

  @param    reg     Register address
  @param    val     Value to write
  @param    sennum  choose the sensor 1 = magnetometer; 2 = accel & gyro;

  @return   void
******************************************************************************/
void LSM9DWriteReg(uint8_t reg, uint8_t val, uint8_t sennum)
{
  switch (sennum) {
    case 1:
      CS_ENABLED(CS5);
      break;
    case 2:
      ENABLED(DEN_AG);
      CS_ENABLED(CS6);
      break;
    default:
      break;
  }
  spiWriteByte(reg);      // Write address
  spiWriteByte(val);      // Write value
  switch (sennum) {
    case 1:
      CS_DISABLED(CS5);
      break;
    case 2:
      CS_DISABLED(CS6);
      DISABLED(DEN_AG);
      break;
    default:
      break;
  }
}


/**************************************************************************//**
  @fn       mgtReadAcc( uint8 sennum)

  @brief    Read each sensor

  @param    reg     Register address
  @param    val     Pointer to destination of read value

  @return   void
******************************************************************************/

int16_t * mgtReadAcc( uint8_t sennum)
{
  static int16_t pBUF[3];
  uint8_t readout[6] = {0, 0, 0, 0, 0, 0};

  // Read all data from accelerometer
  switch (sennum) {
    case 1:
      CS_ENABLED(CS1);
      break;
    case 2:
      CS_ENABLED(CS2);
      break;
    case 3:
      CS_ENABLED(CS3);
      break;
    case 4:
      CS_ENABLED(CS4);
      break;
    default:
      break;
  }


  spiWriteByte(0xC0 | MGT_X_LSB);   // Write start address
  for (uint8_t i = 0; i < 6; i++)
  {
    spiReadByte((uint8_t *)&readout[i], 0xFF); // Read byte
  }
  switch (sennum) {
    case 1:
      CS_DISABLED(CS1);          // left back
      break;
    case 2:
      CS_DISABLED(CS2);          // left front
      break;
    case 3:
      CS_DISABLED(CS3);          // right front
      break;
    case 4:
      CS_DISABLED(CS4);          // right back
      break;
    default:
      break;
  }
  //   sennum = CS_DISABLED;

  // Use only most significant byte of each channel.
  pBUF[0] = (((uint8_t)readout[0]) | ((int16_t)(readout[1]) << 8) );
  pBUF[1] = (((uint8_t)readout[2]) | ((int16_t)(readout[3]) << 8) );
  pBUF[2] = (((uint8_t)readout[4]) | ((int16_t)(readout[5]) << 8) );

  return pBUF;

} // mgtReadAcc



/**************************************************************************//**
  @fn       LSM303DAcc( uint8 sennum)

  @brief    Read each sensor

  @param    reg     Register address
  @param    val     Pointer to destination of read value

  @return   void
******************************************************************************/

int16_t * LSM3DAcc( uint8_t sennum)
{
  static int16_t pBUF[3];
  uint8_t readout[6] = {0, 0, 0, 0, 0, 0};

  // Read all data from accelerometer
  switch (sennum) {
    case 1:
      CS_ENABLED(CS1);
      break;
    case 2:
      CS_ENABLED(CS2);
      break;
    case 3:
      CS_ENABLED(CS3);
      break;
    case 4:
      CS_ENABLED(CS4);
      break;
    default:
      break;
  }


  spiWriteByte(0xC0 | ACC_X_LSB);   // Write start address
  for (uint8_t i = 0; i < 6; i++)
  {
    spiReadByte((uint8_t *)&readout[i], 0xFF); // Read byte
  }
  switch (sennum) {
    case 1:
      CS_DISABLED(CS1);
      break;
    case 2:
      CS_DISABLED(CS2);
      break;
    case 3:
      CS_DISABLED(CS3);
      break;
    case 4:
      CS_DISABLED(CS4);
      break;
    default:
      break;
  }
  //   sennum = CS_DISABLED;

  // Use only most significant byte of each channel.
  pBUF[0] = (((uint8_t)readout[0]) | ((int16_t)(readout[1]) << 8) );
  pBUF[1] = (((uint8_t)readout[2]) | ((int16_t)(readout[3]) << 8) );
  pBUF[2] = (((uint8_t)readout[4]) | ((int16_t)(readout[5]) << 8) );

  return pBUF;

} // LSM303DAcc


/**************************************************************************//**
  @fn       ReadLSM9DS1(uint8 sennum, uint8 reg_add)

  @brief    Read each sensor

  @param    reg     Register address
  @param    val     Pointer to destination of read value

  @return   void
******************************************************************************/

int16_t * ReadLSM9DS1(uint8_t sennum, uint8_t reg_add)
{
  static int16_t pBUF[3];
  uint8_t readout[6] = {0, 0, 0, 0, 0, 0};

  // Read all data from accelerometer
  switch (sennum) {
    case 1:
      CS_ENABLED(CS5);

      spiWriteByte(0xC0 | reg_add);   // Write start address
      for (uint8_t j = 0; j < 6; j++)
      {
        spiReadByte((uint8_t *)&readout[j], 0xFF); // Read byte
      }
      break;
    case 2:
      ENABLED(DEN_AG);
      CS_ENABLED(CS6);
      spiWriteByte(0x80 | reg_add);   // Write start address
      for (uint8_t i = 0; i < 6; i++)
      {
        spiReadByte((uint8_t *)&readout[i], 0xFF); // Read byte
      }
      break;
    default:
      break;
  }

  switch (sennum) {
    case 1:
      CS_DISABLED(CS5);
      break;
    case 2:
      CS_DISABLED(CS6);
      DISABLED(DEN_AG);
      break;
    default:
      break;
  }


  // Use only most significant byte of each channel.
  pBUF[0] = (((uint8_t)readout[0]) | ((int16_t)(readout[1]) << 8) );
  pBUF[1] = (((uint8_t)readout[2]) | ((int16_t)(readout[3]) << 8) );
  pBUF[2] = (((uint8_t)readout[4]) | ((int16_t)(readout[5]) << 8) );

  return pBUF;

}


/**************************************************************************//**
  @fn       spiWriteByte(uint8 write)

  @brief    Write one byte to SPI interface

  @param    write   Value to write
******************************************************************************/
void spiWriteByte(uint8_t write)
{
  SPI.transfer(write);

}


/**************************************************************************//**
  @fn       spiReadByte(uint8 *read, uint8 write)

  @brief    Read one byte from SPI interface

  @param    read    Read out value
  @param    write   Value to write
******************************************************************************/
void spiReadByte(uint8_t *read, uint8_t write)
{
  *read = SPI.transfer(write);                 // Save returned value
}


/******************************************************************************/

//--------------mTDS mouse movement computation-------------------------//
void mTDSHT(float *imudata, uint8_t *mouse)
{
  //  uint8_t mouse[2];
  //mtdsScale(imudata);
  mtdsRotate(imudata);
  mtdsRemoveGyroOffset(imudata);
  mtdsGetPitchRollUnfiltered(imudata);
  mtdsGetPitchRollKalman(imudata, previousdatasampleptr);

  cmd[1] =  (int8_t) * (imudata + 7);
  cmd[2] =  (int8_t) * (imudata + 6);

  mtdsMouseMovementModel(mouse, imudata);

  // update prvious data sample
  *(previousdatasampleptr + 0) = *(imudata + 8);    // pitch_kf
  *(previousdatasampleptr + 1) = *(imudata + 9);    // ry_kf
  *(previousdatasampleptr + 2) = *(imudata + 10);   // roll_kf
  *(previousdatasampleptr + 3) = *(imudata + 11);   // rx_kf


}

//---------------scaling--------------------------------------------//

void mtdsScale (float *datasampleptr)
{
  *(datasampleptr + 0) = (*(datasampleptr + 0));// / 16384;
  *(datasampleptr + 1) = (*(datasampleptr + 1));// / 16384;
  *(datasampleptr + 2) = (*(datasampleptr + 2));// / 16384;
  *(datasampleptr + 3) = (*(datasampleptr + 3)) / 131;
  *(datasampleptr + 4) = (*(datasampleptr + 4)) / 131;
  *(datasampleptr + 5) = (*(datasampleptr + 5)) / 131;

  /*
    // Scale the accelerometer data
     (datasampleptr + 0) = *(datasampleptr + 0) * 2;
     (datasampleptr + 1) = *(datasampleptr + 1) * 2;    // work on combining the maths
     (datasampleptr + 2) = *(datasampleptr + 2) * 2;

    // Scale the gyroscope data
     (datasampleptr + 3) = *(datasampleptr + 3) * 250;
     (datasampleptr + 4) = *(datasampleptr + 4) * 250;
     (datasampleptr + 5) = *(datasampleptr + 5) * 250;
  */
}

//--------------rotation------------------------------//

void mtdsRotate(float *imudata)
{

  // Rotate Accelerometer in 3D // temp_accel = Rotation*accel_in_t
  float temp_accel[3];
  float temp_gyro[3];

  // get the input
  float accel_in[3] = { *(imudata + 0), *(imudata + 1), *(imudata + 2) };
  // do 3D rotation
  matmul31(temp_accel, Rotation, accel_in);


  // end of modification
  *(imudata + 0) = -temp_accel[0];
  *(imudata + 1) = temp_accel[1];
  *(imudata + 2) = temp_accel[2];


  // get the input
  float gyro_in[3] = { *(imudata + 3), *(imudata + 4), *(imudata + 5) };
  // do 3D rotation
  matmul31(temp_gyro, Rotation, gyro_in);


  *(imudata + 3) = -temp_gyro[0];
  *(imudata + 4) = temp_gyro[1];
  *(imudata + 5) = temp_gyro[2];

}

//----------------Remove Gyro Offset--------------//

void mtdsRemoveGyroOffset(float *datasampleptr)
{ // Fix the gyroscope offset
  *(datasampleptr + 3) = *(datasampleptr + 3) - *(GyroOffset + 0);
  *(datasampleptr + 4) = *(datasampleptr + 4) - *(GyroOffset + 1);
  *(datasampleptr + 5) = *(datasampleptr + 5) - *(GyroOffset + 2);
  //cout<<"Test: Gyro Offset Corrected rx = "<<*(datasampleptr + rx)<<" ry = "<<*(datasampleptr + ry)<<" rz ="<<*(datasampleptr + rz)<<endl;
}

//-------------Pitch Roll Unfiltered-----------------------//

void mtdsGetPitchRollUnfiltered(float *datasampleptr)
{
  *(datasampleptr + 6) = atan(-(*(datasampleptr + 0)) / (*(datasampleptr + 2))) * 180 / 3.1416;                                                                               // pitch
  *(datasampleptr + 7) = atan(*(datasampleptr + 1) / (*(datasampleptr + 0) * (*(datasampleptr + 0)) + * (datasampleptr + 2) * (*(datasampleptr + 2)))) * 180 / 3.1416;   // roll

  // *(datasampleptr + 6)  = (atan2(- * (datasampleptr + 1), *(datasampleptr + 2)) * 180.0) / 3.1416; // pitch
  // *(datasampleptr + 7) = (atan2(*(datasampleptr + 0), sqrt(*(datasampleptr + 1)**(datasampleptr + 1) + * (datasampleptr + 2) * *(datasampleptr + 2))) * 180.0) / 3.1416; // roll


  //cout<<"Test: Unfiltered pitch = "<<*(datasampleptr + pitch)<<" roll = "<<*(datasampleptr + roll)<<endl;

}




//-------------------- Kalman filter implementation ---------------------//

void mtdsGetPitchRollKalman(float *datasampleptr, float *previousdatasampleptr)
{

  float dt = 1 / 30;
  float A[4] = { 1.0, dt , 0, 1.0};                  // A = [1 dt; 0  1];
  float At[4] = {A[0], A[2], A[1], A[3]};     // Initialize A transpose
  float H[4] = {1, 0, 0, 1};                         // H = [1 0 ; 0  1]; Identity Matrix; Ht = H

  float KalmanNoiseGain = 1;

  float Q_roll[4];
  Q_roll[0] = KalmanNoiseGain * (*(NoiseCovptr + 0)); // NoiseCov.rows(0,1);
  Q_roll[1] = KalmanNoiseGain * (*(NoiseCovptr + 1));
  Q_roll[2] = KalmanNoiseGain * (*(NoiseCovptr + 2));
  Q_roll[3] = KalmanNoiseGain * (*(NoiseCovptr + 3));

  // This is the format for use with mat
  //Q_roll[0][0] = KalmanNoiseGain*(*(NoiseCovptr + 0)); // NoiseCov.rows(0,1);
  //Q_roll[1][0] = KalmanNoiseGain*(*(NoiseCovptr + 1));
  //Q_roll[0][1] = KalmanNoiseGain*(*(NoiseCovptr + 8));
  //Q_roll[1][1] = KalmanNoiseGain*(*(NoiseCovptr + 9));

  float Q_pitch[4];
  Q_pitch[0] = KalmanNoiseGain * (*(NoiseCovptr + 4)); // NoiseCov.rows(2, 3);
  Q_pitch[1] = KalmanNoiseGain * (*(NoiseCovptr + 5));
  Q_pitch[2] = KalmanNoiseGain * (*(NoiseCovptr + 6));
  Q_pitch[3] = KalmanNoiseGain * (*(NoiseCovptr + 7));

  // This is the format for use with mat
  //Q_pitch[0][0] = KalmanNoiseGain*(*(NoiseCovptr + 2)); // NoiseCov.rows(2, 3);
  //Q_pitch[1][0] = KalmanNoiseGain*(*(NoiseCovptr + 3));
  //Q_pitch[0][1] = KalmanNoiseGain*(*(NoiseCovptr + 10));
  //Q_pitch[1][1] = KalmanNoiseGain*(*(NoiseCovptr + 11));

  float R_roll[4];

  R_roll[0] = KalmanNoiseGain * (*(NoiseCovptr + 8)); // NoiseCov.rows(4, 5);
  R_roll[1] = KalmanNoiseGain * (*(NoiseCovptr + 9));
  R_roll[2] = KalmanNoiseGain * (*(NoiseCovptr + 10));
  R_roll[3] = KalmanNoiseGain * (*(NoiseCovptr + 11));

  // This is the format for use with mat
  //R_roll[0][0] = KalmanNoiseGain*(*(NoiseCovptr + 4));  // NoiseCov.rows(4, 5);
  //R_roll[1][0] = KalmanNoiseGain*(*(NoiseCovptr + 5));
  //R_roll[0][1] = KalmanNoiseGain*(*(NoiseCovptr + 12));
  //R_roll[1][1] = KalmanNoiseGain*(*(NoiseCovptr + 13));

  float R_pitch[4];

  R_pitch[0] = KalmanNoiseGain * (*(NoiseCovptr + 12)); // NoiseCov.rows(6, 7);
  R_pitch[1] = KalmanNoiseGain * (*(NoiseCovptr + 13));
  R_pitch[2] = KalmanNoiseGain * (*(NoiseCovptr + 14));
  R_pitch[3] = KalmanNoiseGain * (*(NoiseCovptr + 15));

  // This is the format for use with mat
  //R_pitch[0][0] = KalmanNoiseGain*(*(NoiseCovptr + 6)); // NoiseCov.rows(6, 7);
  //R_pitch[1][0] = KalmanNoiseGain*(*(NoiseCovptr + 7));
  //R_pitch[0][1] = KalmanNoiseGain*(*(NoiseCovptr + 14));
  //R_pitch[1][1] = KalmanNoiseGain*(*(NoiseCovptr + 15));

  //cout << "Unpacking Noise Cov was ok" << endl;;

  // Unpack the State parameters from previous timestep
  float X_est_pitch_t_minus_1[2];
  X_est_pitch_t_minus_1[0] = *(previousdatasampleptr + 0);        //pitch_kf
  X_est_pitch_t_minus_1[1] = *(previousdatasampleptr + 1);        //ry_kf

  float X_est_roll_t_minus_1[2];
  X_est_roll_t_minus_1[0] = *(previousdatasampleptr + 2);         //roll_kf
  X_est_roll_t_minus_1[1] = *(previousdatasampleptr + 3);         //rx_kf
  //cout << "Unpacking State Variables was ok" << endl;

  // FOR NO ARMADILLO
  float P_pitch_t_minus_1[4];
  // Unpack the previoud PCov
  P_pitch_t_minus_1[0] = *(PCov + 0);
  P_pitch_t_minus_1[1] = *(PCov + 1);
  P_pitch_t_minus_1[2] = *(PCov + 4);
  P_pitch_t_minus_1[3] = *(PCov + 5);

  float P_roll_t_minus_1[4];
  P_roll_t_minus_1[0] = *(PCov + 2);
  P_roll_t_minus_1[1] = *(PCov + 3);
  P_roll_t_minus_1[2] = *(PCov + 6);
  P_roll_t_minus_1[3] = *(PCov + 7);


  //// Time update - Prediction - PITCH


  float X_estprior_pitch[2];
  float P_prior_pitch[4];

  ////// X_estprior_pitch = A*X_est_pitch_t_minus_1;

  //Mat C5 = Mat(2, 2, CV_64F, A)*Mat(2, 1, CV_64F, X_est_pitch_t_minus_1);   // X_estprior_pitch = A*X_est_pitch_t_minus_1;
  matmul21(X_estprior_pitch, A, X_est_pitch_t_minus_1);


  //////// P_prior_pitch = A*P_pitch_t_minus_1*A.t() + Q_pitch;

  //Mat C8 = Mat(2, 2, CV_64F, A)*Mat(2, 2, CV_64F, P_pitch_t_minus_1) * Mat(2, 2, CV_64F, At) + Mat(2, 2, CV_64F, Q_pitch);
  // P_prior_pitch = A*P_pitch_t_minus_1*A.t() + Q_pitch;
  float temp0[4];
  float temp1[4];
  matmul22(temp0, A, P_pitch_t_minus_1);
  matmul22(temp1, temp0, At);
  // add them
  P_prior_pitch[0] = temp1[0] + Q_pitch[0];
  P_prior_pitch[1] = temp1[1] + Q_pitch[1];
  P_prior_pitch[2] = temp1[2] + Q_pitch[2];
  P_prior_pitch[3] = temp1[3] + Q_pitch[3];


  //// Measurement Update - Correction - PITCH
  float K_pitch[4];
  float X_est_pitch[2];
  float P_pitch[4];
  float temp_pitch[2] = { *(datasampleptr + 6), *(datasampleptr + 4) };   // indexes pitch = 6 ry = 4

  ////// K_pitch = (P_prior_pitch * H.t())* inv ( H * P_prior_pitch * H.t() + R_pitch);

  //Mat C14 = (Mat(2, 2, CV_64F, P_prior_pitch)*Mat(2, 2, CV_64F, H)) * (Mat(2, 2, CV_64F, H)*Mat(2, 2, CV_64F, P_prior_pitch) * Mat(2, 2, CV_64F, H) + Mat(2, 2, CV_64F, R_pitch)).inv(DECOMP_SVD);      // K_pitch = (P_prior_pitch * H.t())* inv ( H * P_prior_pitch * H.t() + R_pitch);

  float temp2[4];
  float temp3[4];
  float temp4[4];
  float temp5[4];

  matmul22(temp2, P_prior_pitch, H); //(P_prior_pitch * H.t())
  matmul22(temp3, H, P_prior_pitch); // H * P_prior_pitch
  matmul22(temp4, temp3, H);      //H * P_prior_pitch * H.t()
  // H * P_prior_pitch * H.t() + R_pitch
  temp4[0] += R_pitch[0];
  temp4[1] += R_pitch[1];
  temp4[2] += R_pitch[2];
  temp4[3] += R_pitch[3];
  // inverse temp4
  matinv22(temp5, temp4);
  matmul22(K_pitch, temp2, temp5);


  ////// X_est_pitch = X_estprior_pitch + K_pitch*(temp_pitch - H * X_estprior_pitch);

  //Mat C18 = (Mat(2, 2, CV_64F, K_pitch) * (Mat(2, 1, CV_64F, temp_pitch) - Mat(2, 2, CV_64F, H)*Mat(2, 1, CV_64F, X_estprior_pitch))) + Mat(2, 1, CV_64F, X_estprior_pitch);    // X_est_pitch = X_estprior_pitch + K_pitch*(temp_pitch - H * X_estprior_pitch);

  float temp6[2];
  float temp7[2];
  matmul21(temp6, H, X_estprior_pitch); //H * X_estprior_pitch
  //temp_pitch - H * X_estprior_pitch
  temp_pitch[0] -= temp6[0];
  temp_pitch[1] -= temp6[1];

  matmul21(temp7, K_pitch, temp_pitch); // K_pitch*(temp_pitch - H * X_estprior_pitch)
  // X_est_pitch = X_estprior_pitch + K_pitch*(temp_pitch - H * X_estprior_pitch)
  X_est_pitch[0] = X_estprior_pitch[0] + temp7[0];
  X_est_pitch[1] = X_estprior_pitch[1] + temp7[1];



  //P_pitch     = P_prior_pitch - K_pitch * (H * P_prior_pitch * H' + R_pitch)*K_pitch';

  //Mat C21 = Mat(2, 2, CV_64F, P_prior_pitch) - Mat(2, 2, CV_64F, K_pitch)*(Mat(2, 2, CV_64F, H)*Mat(2, 2, CV_64F, P_prior_pitch)*(Mat(2, 2, CV_64F, H).t()) + Mat(2, 2, CV_64F, R_pitch))*Mat(2, 2, CV_64F, K_pitch).t();

  float temp8[4];
  float temp9[4];
  float temp10[4];
  float temp11[4];
  //double HT[4];
  float K_pitchT[4];
  matmul21(temp8, H, P_prior_pitch);  //H * P_prior_pitch
  //mattp22(HT, H);   // H'
  matmul22(temp9, temp8, H);  //H * P_prior_pitch * H' // HT = H
  // (H * P_prior_pitch * H' + R_pitch)
  temp10[0] = temp9[0] + R_pitch[0];
  temp10[1] = temp9[1] + R_pitch[1];
  temp10[2] = temp9[2] + R_pitch[2];
  temp10[3] = temp9[3] + R_pitch[3];
  mattp22(K_pitchT, K_pitch); //K_pitch'
  matmul22(temp11, K_pitch, temp10); // K_pitch * (H * P_prior_pitch * H' + R_pitch)
  matmul22(temp10, temp11, K_pitchT);//K_pitch * (H * P_prior_pitch * H' + R_pitch)*K_pitch'

  //P_pitch     = P_prior_pitch - K_pitch * (H * P_prior_pitch * H' + R_pitch)*K_pitch';
  P_pitch[0] = P_prior_pitch[0] - temp10[0];
  P_pitch[1] = P_prior_pitch[1] - temp10[1];
  P_pitch[2] = P_prior_pitch[2] - temp10[2];
  P_pitch[3] = P_prior_pitch[3] - temp10[3];

  // ROLL SECTION

  //// Time update - Prediction - ROLL

  ////// X_estprior_roll = A*X_est_roll_t_minus_1;
  float X_estprior_roll[2];
  float P_prior_roll[4];


  //Mat C22 = Mat(2, 2, CV_64F, A) * Mat(2, 1, CV_64F, X_est_roll_t_minus_1);   // X_estprior_roll = A*X_est_roll_t_minus_1;

  matmul21(X_estprior_roll, A, X_est_roll_t_minus_1);


  ////// P_prior_roll = A*P_roll_t_minus_1*A.t() + Q_roll;

  //Mat C25 = (Mat(2, 2, CV_64F, A) * Mat(2, 2, CV_64F, P_roll_t_minus_1) * Mat(2, 2, CV_64F, At)) + Mat(2, 2, CV_64F, Q_roll);       // P_prior_roll = A*P_roll_t_minus_1*A.t() + Q_roll;

  float temp12[4];
  matmul22(temp12, A, P_roll_t_minus_1);
  matmul22(temp11, temp12, At); //A*P_roll_t_minus_1*A.t()

  // P_prior_roll = A*P_roll_t_minus_1*A.t() + Q_roll;
  P_prior_roll[0] = temp11[0] + Q_roll[0];
  P_prior_roll[1] = temp11[1] + Q_roll[1];
  P_prior_roll[2] = temp11[2] + Q_roll[2];
  P_prior_roll[3] = temp11[3] + Q_roll[3];


  //// Measurement Update - Correction - ROLL
  float K_roll[4];
  float X_est_roll[2];
  float P_roll[4];
  float temp_roll[2] = { *(datasampleptr + 7), *(datasampleptr + 3) };

  // K_roll = (P_prior_roll * H.t()) * inv ( H * P_prior_roll * H.t() + R_roll);

  //Mat C31 = ((((Mat(2, 2, CV_64F, H) * Mat(2, 2, CV_64F, P_prior_roll)) * Mat(2, 2, CV_64F, H)) + Mat(2, 2, CV_64F, R_roll)).inv(DECOMP_SVD)) * (Mat(2, 2, CV_64F, P_prior_roll) * Mat(2, 2, CV_64F, H));
  // K_roll = (P_prior_roll * H.t()) * inv ( H * P_prior_roll * H.t() + R_roll);

  float temp13[4];
  float temp14[4];
  float temp15[4];
  float temp16[4];
  matmul22(temp13, P_prior_roll, H); //P_prior_roll * H.t() // HT = H
  matmul22(temp14, H, P_prior_roll);
  matmul22(temp15, temp14, H); // HT = H

  //( H * P_prior_roll * H.t() + R_roll)
  temp15[0] += R_roll[0];
  temp15[1] += R_roll[1];
  temp15[2] += R_roll[2];
  temp15[3] += R_roll[3];

  matinv22(temp16, temp15); // inv ( H * P_prior_roll * H.t() + R_roll)
  matmul22(K_roll, temp13, temp16);



  // X_est_roll = X_estprior_roll + K_roll*(temp_roll - H * X_estprior_roll);

  //Mat C35 = (Mat(2, 2, CV_64F, K_roll) * (Mat(2, 1, CV_64F, temp_roll) - (Mat(2, 2, CV_64F, H) * Mat(2, 1, CV_64F, X_estprior_roll)))) + Mat(2, 1, CV_64F, X_estprior_roll);
  // X_est_roll = X_estprior_roll + K_roll*(temp_roll - H * X_estprior_roll);

  float temp17[2];
  float temp18[2];

  matmul21(temp17, H, X_estprior_roll);   // H * X_estprior_roll
  //(temp_roll - H * X_estprior_roll)
  temp_roll[0] -= temp17[0];
  temp_roll[1] -= temp17[1];
  matmul21(temp18, K_roll, temp_roll);  //  K_roll*(temp_roll - H * X_estprior_roll)

  //X_estprior_roll + K_roll*(temp_roll - H * X_estprior_roll);
  X_est_roll[0] = X_estprior_roll[0] + temp18[0];
  X_est_roll[1] = X_estprior_roll[1] + temp18[1];


  //  P_roll = P_prior_roll - K_roll * (H * P_prior_roll * H' + R_roll) * K_roll';

  //Mat C38 = Mat(2, 2, CV_64F, P_prior_roll) - Mat(2, 2, CV_64F, K_roll) * (Mat(2, 2, CV_64F, H)* Mat(2, 2, CV_64F, P_prior_roll) * Mat(2, 2, CV_64F, H).t() + Mat(2, 2, CV_64F, R_roll)) * Mat(2, 2, CV_64F, K_roll).t();

  float K_rollT[4];

  mattp22(K_rollT, K_roll); // K_roll'
  matmul22(temp13, H, P_prior_roll); // H * P_prior_roll
  matmul22(temp14, temp13, H);  // H * P_prior_roll * H' // HT = H

  //(H * P_prior_roll * H' + R_roll)
  R_roll[0] += temp14[0];
  R_roll[1] += temp14[1];
  R_roll[2] += temp14[2];
  R_roll[3] += temp14[3];
  // K_roll * (H * P_prior_roll * H' + R_roll)
  matmul22(temp15, K_roll, R_roll);
  //K_roll * (H * P_prior_roll * H' + R_roll) * K_roll'
  matmul22(temp16, temp15, K_rollT);
  //  P_roll = P_prior_roll - K_roll * (H * P_prior_roll * H' + R_roll) * K_roll';

  P_roll[0] = P_prior_roll[0] - temp16[0];
  P_roll[1] = P_prior_roll[1] - temp16[1];
  P_roll[2] = P_prior_roll[2] - temp16[2];
  P_roll[3] = P_prior_roll[3] - temp16[3];

  // MAIN KALMAN BLOCKS ARE DONE

  // Put PCov back together in its original location
  *(PCov + 0) = P_pitch[0];
  *(PCov + 1) = P_pitch[1];
  *(PCov + 4) = P_pitch[2];
  *(PCov + 5) = P_pitch[3];

  *(PCov + 2) = P_roll[0];
  *(PCov + 3) = P_roll[1];
  *(PCov + 6) = P_roll[2];
  *(PCov + 7) = P_roll[3];

  // Put the data into datasample array
  *(datasampleptr + 8) = X_est_pitch[0];           //
  *(datasampleptr + 9) = X_est_pitch[1];
  *(datasampleptr + 10) = X_est_roll[0];
  *(datasampleptr + 11) = X_est_roll[1];

}


//-----------mouse cursor curve implement------------------------//

void mtdsMouseMovementModel(uint8_t *mouse, float *imudata)
{
  int mousex, mousey;
  int dx, dy;
  float PitchRoll[2];
  int Fs = 100;      // 1/30
  int MouseThreshold[6] = { 0, 20, 20, 40, 60, 90 };
  int MouseResponseX[5] = { 0, 5, 10, 30, 50 };

  int MouseResponseY[5] = { 0, 8, 15, 45, 75 };    // Making it 1.5 times of X per Sahadat's request


  // Pick up calculations from the filtering algorithm we prefer
  //PitchRoll[1] = *(imudata + 8);        // pitch  y movement
  // PitchRoll[0] = *(imudata + 10);        // roll  x movement

  PitchRoll[1] = *(imudata + 6);        // pitch  y movement
  PitchRoll[0] = *(imudata + 7);        // roll  x movement

  if (PitchRoll[0] < 0)
  {
    mousex = (int)(PitchRoll[0] * (-0.7));
  }
  else
  {
    mousex = (int)PitchRoll[0];
  }

  if (PitchRoll[1] < 0)
  {
    mousey = (int)(PitchRoll[1] * (-1));
  }
  else
  {
    mousey = (int)PitchRoll[1];
  }


  // Apply the 6 stage Mouse Response Function X Axis
  if (mousex > MouseThreshold[0] && mousex <= MouseThreshold[1])
  {
    dx =  MouseResponseX[0] * mousex / Fs ;
  }
  else if (mousex > MouseThreshold[1] && mousex <= MouseThreshold[2])
  {
    dx = (MouseResponseX[1] * mousex - 50) / Fs;
  }
  else if (mousex > MouseThreshold[2] && mousex <= MouseThreshold[3])
  {
    dx = (MouseResponseX[2] * mousex - 150) / Fs;
  }
  else if (mousex > MouseThreshold[3] && mousex <= MouseThreshold[4])
  {
    dx = (MouseResponseX[3] * mousex - 950) / Fs;
  }
  else if (mousex > MouseThreshold[4] && mousex <= MouseThreshold[5])
  {
    dx = (MouseResponseX[4] * mousex - 2150) / Fs;
  }
  else
  {
    dx = 0;
  }

  // Apply the 6 stage Mouse Response Function Y Axis
  if (mousey > MouseThreshold[0] && mousey <= MouseThreshold[1])
  {
    dy =  MouseResponseY[0] * mousey / Fs;
  }
  else if (mousey > MouseThreshold[1] && mousey <= MouseThreshold[2])
  {
    dy = (MouseResponseY[1] * mousey - 75) / Fs;
  }
  else if (mousey > MouseThreshold[2] && mousey <= MouseThreshold[3])
  {
    dy = (MouseResponseY[2] * mousey - 225) / Fs;
  }
  else if (mousey > MouseThreshold[3] && mousey <= MouseThreshold[4])
  {
    dy = (MouseResponseY[3] * mousey - 1425) / Fs;
  }
  else if (mousey > MouseThreshold[4] && mousey <= MouseThreshold[5])
  {
    dy = (MouseResponseY[4] * mousey - 3225) / Fs;
  }
  else
  {
    dy = 0;
  }

  if (PitchRoll[0] < 0)
  {
    *(mouse + 0) = (~(uint8_t)dx * 2 + 0x01);
  }
  else
  {
    *(mouse + 0) = (uint8_t)dx;
  }

  if (PitchRoll[1] < 0)
  {
    *(mouse + 1) = (~(uint8_t)dy + 0x01);
  }
  else
  {
    *(mouse + 1) = (uint8_t)dy;
  }

}

//-------------matrix multiplication 2x2 X 2x1-------------------//

void matmul21(float *res, float *a, float *b)
{
  *(res + 0) = a[0] * b[0] + a[1] * b[1];
  *(res + 1) = a[2] * b[0] + a[3] * b[1];
}

//-------------matrix multiplication 2x2 X 2x2-------------------//

void matmul22(float *res, float *a, float *b)
{
  *(res + 0) = a[0] * b[0] + a[1] * b[2];
  *(res + 1) = a[0] * b[1] + a[1] * b[3];
  *(res + 2) = a[2] * b[0] + a[3] * b[2];
  *(res + 3) = a[2] * b[1] + a[3] * b[3];
}

//----------matrix inverse 2x2------------------//

void matinv22(float *res, float *a)
{
  float mag = a[0] * a[3] - a[1] * a[2];
  *(res + 0) = a[3] / mag;
  *(res + 1) = -a[1] / mag;
  *(res + 2) = -a[2] / mag;
  *(res + 3) = a[0] / mag;
}

//----------matrix transpose 2x2------------------//

void mattp22(float *res, float *a)
{
  *(res + 0) = *(a + 0);
  *(res + 1) = *(a + 2);
  *(res + 2) = *(a + 1);
  *(res + 3) = *(a + 3);
}

//-------------matrix multiplication 3x3 X 3x1-------------------//

void matmul31(float *res, float *a, float *b)
{
  *(res + 0) = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  *(res + 1) = a[3] * b[0] + a[4] * b[1] + a[5] * b[2];
  *(res + 2) = a[6] * b[0] + a[7] * b[1] + a[8] * b[2];
}


//---------------receive Model Code here--------------------------//

void receiveModel(void)
{
  while (PktNumber < 6)
  {
    if (Serial.available() > 0)
    {

      if (countM == 0)
      {
        PktNumber = Serial.read();
      }

      switch (PktNumber)
      {
        case 0:
          ModelPkt0[countM] = Serial.read();
          break;

        case 1:
          ModelPkt1[countM] = Serial.read();
          break;

        case 2:
          ModelPkt2[countM] = Serial.read();
          break;

        case 3:
          ModelPkt3[countM] = Serial.read();
          break;

        case 4:
          ModelPkt4[countM] = Serial.read();
          break;

        case 5:
          ModelPkt5[countM] = Serial.read();
          break;

        default:
          break;
      }


      countM = countM + 1;
      //Serial.flush();
    }

    if (countM > (MODEL_BUFFER_SIZE - 1))
    {

      switch (PktNumber)
      {
        case 0:
          ModelPkt0[0] = PktNumber;
          Serial.write(ModelPkt0, MODEL_BUFFER_SIZE);
          break;

        case 1:
          ModelPkt1[0] = PktNumber;
          Serial.write(ModelPkt1, MODEL_BUFFER_SIZE);
          break;

        case 2:
          ModelPkt2[0] = PktNumber;
          Serial.write(ModelPkt2, MODEL_BUFFER_SIZE);
          break;

        case 3:
          ModelPkt3[0] = PktNumber;
          Serial.write(ModelPkt3, MODEL_BUFFER_SIZE);
          break;

        case 4:
          ModelPkt4[0] = PktNumber;
          Serial.write(ModelPkt4, MODEL_BUFFER_SIZE);
          break;

        case 5:
          ModelPkt5[0] = PktNumber;
          Serial.write(ModelPkt5, MODEL_BUFFER_SIZE);
          break;

        default:
          break;
      }
      //Serial.flush();

      countM = 0;
    }
  }
}

//-------------------coverting the model file to W, Beta, Proj1, Proj2, Proj3, and Proj4-------------//

void convertModel(void)
{
  //NffsFile file;
  // read the file 
  file.open(FILENAME0, FS_ACCESS_READ);
 // if (file.exists())
 // {
    file.read(ModelPkt0,MODEL_BUFFER_SIZE);
 // }
  file.close();

  file.open(FILENAME1, FS_ACCESS_READ);
  //if (file.exists())
  //{
  file.read(ModelPkt1,MODEL_BUFFER_SIZE);
 // }
  file.close();
  
  file.open(FILENAME2, FS_ACCESS_READ);
 // if (file.exists())
 // {
  file.read(ModelPkt2,MODEL_BUFFER_SIZE);
 // }
  file.close();
  
  file.open(FILENAME3, FS_ACCESS_READ);
 // if (file.exists())
 // {
  file.read(ModelPkt3,MODEL_BUFFER_SIZE);
 // }
  file.close();
  
  file.open(FILENAME4, FS_ACCESS_READ);
 // if (file.exists())
 // {
  file.read(ModelPkt4,MODEL_BUFFER_SIZE);
 // }
  file.close();
  
  file.open(FILENAME5, FS_ACCESS_READ);
 // if (file.exists())
 // {
  file.read(ModelPkt5,MODEL_BUFFER_SIZE);
 // }
  file.close();
  
  // let's convert proj1, 2, 3, 4 and Beta
  uint8_t indx1 = 0;    // for b1
  uint8_t indx2 = 0;    // for b2
  uint8_t indx3 = 0;    // for b3
  uint8_t indx4 = 0;    // for b4
  uint8_t indxB = 0;    // for beta
  uint8_t r = 0;    // for row W
  uint8_t c = 0;    // for col W

 // ModelPkt0 conversion
 for (int i=1; i<MODEL_BUFFER_SIZE; i=i+4)
  {
    
    u.b[0] = ModelPkt0[i+0];
    u.b[1] = ModelPkt0[i+1];
    u.b[2] = ModelPkt0[i+2];
    u.b[3] = ModelPkt0[i+3];
    if (i<37)
    {
    *(b1+ indx1) = u.f; 
    indx1++;
    }
    else if (i>36 && i<73) // upto 72
    {
    *(b2 + indx2) = u.f;
    indx2++;
    }
    else if (i>72 && i<109) // 
    {
    *(b3+indx3) = u.f;
    indx3++;
    }
    else if (i>108 && i<145)
    {
    *(b4 + indx4) = u.f;
    indx4++;
    }
    else
    {
      *(Beta+indxB) = u.f;    // Beta till 16
      indxB++;
    }
  }

  // ModelPkt1 conversion
  for (int i=1; i<MODEL_BUFFER_SIZE; i=i+4)
  {
    u.b[0] = ModelPkt1[i+0];
    u.b[1] = ModelPkt1[i+1];
    u.b[2] = ModelPkt1[i+2];
    u.b[3] = ModelPkt1[i+3];

    if (i < 21)
    {
      *(Beta+indxB) = u.f;      // Beta conversion done
      indxB++;
    }
    else if (i>20 && i < 105)
    {
      W[0][c] = u.f;
      c++;
      if (c > 20)
      {
        c = 0;
      }
    }
    else if (i>104 && i < 189)
    {
      W[1][c] = u.f;
      c++;
      if (c > 20)
      {
        c = 0;
      }
    }
    else 
    {
      W[2][c] = u.f;
      c++;
      if (c > 20)
      {
        c = 0;
      }
    }
  }

  // ModelPkt2 conversion
  for (int i=1; i<MODEL_BUFFER_SIZE; i=i+4)
  {
    u.b[0] = ModelPkt2[i+0];
    u.b[1] = ModelPkt2[i+1];
    u.b[2] = ModelPkt2[i+2];
    u.b[3] = ModelPkt2[i+3];

    if (i < 65)
    {
      W[2][c] = u.f;
      c++;
      if (c > 20)
      {
        c = 0;
      }
    }
    else if (i> 64 && i < 149)
    {
      W[3][c] = u.f;
      c++;
      if (c > 20)
      {
        c = 0;
      }
    }
    else
    {
      W[4][c] = u.f;
      c++;
      if (c > 20)
      {
        c = 0;
      }
    }
  }

  // ModelPkt3 conversion
  for (int i=1; i<MODEL_BUFFER_SIZE; i=i+4)
  {
    u.b[0] = ModelPkt3[i+0];
    u.b[1] = ModelPkt3[i+1];
    u.b[2] = ModelPkt3[i+2];
    u.b[3] = ModelPkt3[i+3];

    if (i < 25)
    {
      W[4][c] = u.f;
      c++;
      if (c > 20)
      {
        c = 0;
      }
    }
    else if (i> 24 && i < 109)
    {
      W[5][c] = u.f;
      c++;
      if (c > 20)
      {
        c = 0;
      }
    }
    else if (i> 108 && i < 193)
    {
      W[6][c] = u.f;
      c++;
      if (c > 20)
      {
        c = 0;
      }
    }
    else
    {
      W[7][c] = u.f;
      c++;
      if (c > 20)
      {
        c = 0;
      }
    }
  }

  // ModelPkt4 conversion
  for (int i=1; i<MODEL_BUFFER_SIZE; i=i+4)
  {
    u.b[0] = ModelPkt4[i+0];
    u.b[1] = ModelPkt4[i+1];
    u.b[2] = ModelPkt4[i+2];
    u.b[3] = ModelPkt4[i+3];

    if (i < 69)
    {
      W[7][c] = u.f;
      c++;
      if (c > 20)
      {
        c = 0;
      }
    }
    else if (i> 68 && i < 153)
    {
      W[8][c] = u.f;
      c++;
      if (c > 20)
      {
        c = 0;
      }
    }
    else
    {
      W[9][c] = u.f;
      c++;
      if (c > 20)
      {
        c = 0;
      }
    }
  }

  // ModelPkt5 conversion
  for (int i=1; i<MODEL_BUFFER_SIZE; i=i+4)
  {
    u.b[0] = ModelPkt5[i+0];
    u.b[1] = ModelPkt5[i+1];
    u.b[2] = ModelPkt5[i+2];
    u.b[3] = ModelPkt5[i+3];

    if (i < 29)
    {
      W[9][c] = u.f;
      c++;
      if (c > 20)
      {
        c = 0;
      }
    }
    else if (i> 28 && i < 113)
    {
      W[10][c] = u.f;
      c++;
      if (c > 20)
      {
        c = 0;
      }
    }
    else if (i> 112 && i < 197)
    {
      W[11][c] = u.f;
      c++;
      if (c > 20)
      {
        c = 0;
      }
    }
  }
  
}

//-------------------saving the model file to flash-------------//

void saveModel(void)
{
  
  //NffsFile file;
  file.open(FILENAME0, FS_ACCESS_WRITE);
  file.write(ModelPkt0,MODEL_BUFFER_SIZE);
  file.close();

  file.open(FILENAME1, FS_ACCESS_WRITE);
  file.write(ModelPkt1,MODEL_BUFFER_SIZE);
  file.close();
  
  file.open(FILENAME2, FS_ACCESS_WRITE);
  file.write(ModelPkt2,MODEL_BUFFER_SIZE);
  file.close();
  
  file.open(FILENAME3, FS_ACCESS_WRITE);
  file.write(ModelPkt3,MODEL_BUFFER_SIZE);
  file.close();
  
  file.open(FILENAME4, FS_ACCESS_WRITE);
  file.write(ModelPkt4,MODEL_BUFFER_SIZE);
  file.close();
  
  file.open(FILENAME5, FS_ACCESS_WRITE);
  file.write(ModelPkt5,MODEL_BUFFER_SIZE);
  file.close();
  //showcom = 1;
}

//------------------------erase model--------------------//
void eraseModel(void)
{
  for (uint32_t addr = STARTING_ADDR; addr < STARTING_ADDR + ERASE_SIZE; addr += SECTOR_SIZE)
  {
    //Serial.printf("Erasing 0x%06X ... ", addr);

    if ( NRF_SUCCESS == sd_flash_page_erase(addr/SECTOR_SIZE) )
    {
      //Serial.println("OK");
    }
    else
    {
      //Serial.println("Failed");
      //Serial.println("Exit due to ERRORs");
      return;
    }
  }
}


