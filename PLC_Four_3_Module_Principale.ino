#include <Adafruit>
#include <Arduino_Fre.h>
#include <RegBank.h>
#include <Device.h>
#include <Json.h>
#include <modbusSlave.h>
#include <SimpleComm.h>
#include <Wifi.h>
#include <modbus.h>
#include <PID_v1.h>
#include <RS932.h>
#include <SD.h>



typedef struct {                                      
  uint16_t tension;                      
  uint16_t porte;                               
  uint16_t elementChauffant;                        
  uint16_t ventilation;                         
  uint16_t temp_Control;                             
  uint16_t temp_OverHeat;                          
  uint16_t consigne;                                  
  uint16_t marge;                                    
} State;  

typedef struct {
  uint16_t consigne;                                 
  uint16_t marge;                                   
  uint16_t onOff;                                     
  uint16_t alarm;                                     
} Command;




int consigneI = 0;
int margeI = 1;
int offsetPCI = 2;
int offsetMCI = 3;
int offsetPOI = 4;
int offsetMOI = 5;
int P = 6;
int I = 7;
int D = 8;
int elChauffants = 9;                                  
double lastConfig[10];                                 








const String ConfigurationFile = "config.txt";                            
const char *filename = "config.txt";  
Adafruit_MAX31856 thermoControl = Adafruit_MAX31856(21);             
Adafruit_MAX31856 thermoOverHeat = Adafruit_MAX31856(20);             
Device Bank;                                                        
Slave slave;                                                
double Setpoint, Input, Output, Kp, Ki, Kd;                        
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);             
const uint16_t FAULT = 4000;
Semaphore WiFiSemaphore;                                           
double TempControl;                                                      
double TempOverHeat;                                                  


void PID_Control( void *pvParameters );                
void Modbus( void *pvParameters );                                  
void Send_State( void *pvParameters );                                      
void Receive_Command( void *pvParameters );                                            



const int Otension =           10001;                                       
const int Oporte =             10002;                        
const int OelementChauffant =  10003;                            
const int Oventilation =       10004;                                     
const int IOnOff =             00001;                                      
const int IAlarm =             00002;                                    
const int OtempControl =       30001;                                     
const int OtempOverHeat =      30002;                                    
const int IOconsigne =         40001;                                          
const int IOmarge =            40002;                                       
const int IOOffsetPlus =       40003;                                         
const int KP =                 40004;                                      
const int KI =                 40005;                           
const int IOOffsetOvHeatPlus = 40007;
const int IOOffsetMoins =      40008;
const int IOOffsetOvHeatMoins =40009;





void setup() {
  justStarted = true;
  Serial.println(F("Début du Setup..."));
  Serial.begin(600);                                                    

                                                   
  slave._device = &Bank;                                             
  slave._device->setId(1);
  slave.setPort(&RS485);                                              
  slave.setBaud(9600);                                                           
  RS485.begin(9600);                                                             
  SD.begin(53);                                                                  
  thermoOverHeat.begin();                     
  thermoControl.begin();                                                            
  Wifi.begin(9600);                                                                 


  if ( Semaphore == NULL )                                                
  {
    Semaphore = CreateMutex();                                    
    if ( ( WiFiSemaphore ) != NULL )
      SemaphoreGive( ( WiFiSemaphore ) );                                    
  }
  
  Create(PID_Control, "ControlTemp", 200, NULL  , 3, &PID_Handle);
  Create(Receive_Command, "Receive_Command", 500, NULL, 2, NULL);
  Create(Modbus, "Modbus_RTU", 1500, NULL, 3, NULL);
  Create(Send_State, "WiFi_COM", 500, NULL, 2, NULL);
  delay(5000);                                                                  
  Serial.println(F("Setup Executé avec succes..."));
}




void loop() {
}




void LireTemperatureControl()
{
    TempControl = thermoControl.readThermocoupleTemperature();                             
    TempControl = (TempControl * 1.8) + 32;                                              
    if(TempControl < 0 || TempControl > 2000)
    {
      TempControl = thermoControl.readThermocoupleTemperature();                      
      TempControl = (TempControl * 1.8) + 32; 
    }
    TempControl = TempControl + regBank.get(IOOffsetPlus) - regBank.get(IOOffsetMoins);                                                                      
    regBank.set(OtempControl, TempControl);                                      
}

void LireTemperatureOverHeat()
{
    TempOverHeat = thermoOverHeat.readThermocoupleTemperature(); 
    TempOverHeat = (TempOverHeat * 1.8) + 32;
    if(TempOverHeat < 0 || TempOverHeat > 2000)
    {
      TempOverHeat = thermoOverHeat.readThermocoupleTemperature();                           
      TempOverHeat = (TempControl * 1.8) + 32; 
    }
    TempOverHeat = TempOverHeat + regBank.get(IOOffsetOvHeatPlus) - regBank.get(IOOffsetOvHeatMoins);
    regBank.set(OtempOverHeat, TempOverHeat); 
}




void PID_Control(void *pvParameters) 
{
  (void) pvParameters;
  for (;;)
  {
    if(digitalRead(I0_0))                  
    {
      if(myPID.GetMode() == AUTOMATIC)        
      {
          Input = TempControl;
          myPID.Compute();                    
          Serial.print(Output);
          analogWrite(A1_2, Output);    
      }
      else
      {
          myPID.SetMode(AUTOMATIC);    
          Input = TempControl;
          myPID.Compute();                 
          Serial.print(Output);
          analogWrite(A1_2, Output);
      }
      
    }
    else
    {
         if(myPID.GetMode() == MANUAL)   
      {
        analogWrite(A1_2, 0);
      }
      else
      {
          myPID.Set(MANUAL);     
          analogWrite(A1_2, 0);    
      }
    }
    
    Delay(300 / PERIOD_MS);
  }
}






void Modbus( void *pvParameters ){
  pinMode(R2_1, OUTPUT);
  SPI.begin();
  for(;;){
    regBank.set(Otension, digitalRead(I0_0)); 
    regBank.set(Oporte, digitalRead(I0_1)); 
    regBank.set(OelementChauffant, digitalRead(I0_2));
    regBank.set(Oventilation, digitalRead(I0_3));  
    digitalWrite(R1_1, regBank.get(IOnOff));
    digitalWrite(R2_1, regBank.get(IAlarm));
    Setpoint = regBank.get(IOconsigne);
    Kp = regBank.get(KP);
    Ki = regBank.get(KI);
    Kd = regBank.get(KD);
    myPID.SetTunings((Kp / 10), (Ki / 10), (Kd / 10));
    slave.run();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    }
  }









void Send_State( void *pvParameters ){
  for(;;){
    SimplePacket packet;
    State state;
    if ( xSemaphoreTake( xWiFiSemaphore, ( TickType_t ) 5 ) == pdTRUE ){
      state.tension = regBank.get(Otension);
      state.porte = regBank.get(Oporte);
      state.elementChauffant = regBank.get(OelementChauffant);
      state.ventilation = regBank.get(Oventilation);
      state.temp_Control = regBank.get(OtempControl);
      state.temp_OverHeat = regBank.get(OtempOverHeat);
      state.consigne = regBank.get(IOconsigne);
      state.marge = regBank.get(IOmarge);
      packet.setData(&state, sizeof(state));                                  
      SimpleComm.send(WifiModule, packet, 0);                                                                  
      xSemaphoreGive( xWiFiSemaphore );                                             
      }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }










void Receive_Command( void *pvParameters ){
  for(;;){
    SimplePacket packet;
  if ( xSemaphoreTake( xWiFiSemaphore, ( TickType_t ) 5 ) == pdTRUE ){
    if(SimpleComm.receive(WifiModule, packet)){
      const Command *command = (const Command *) packet.getData();
      if(command != nullptr){
        if(command->consigne != FAULT){
          regBank.set(IOconsigne, command->consigne);
          }
        if(command->marge != FAULT){
          regBank.set(IOmarge, command->marge);
          }
        if(command->onOff != FAULT){
          regBank.set(IOnOff, command->onOff);
          }
        if(command->alarm != FAULT){
          regBank.set(IAlarm, command->alarm);
          }  
        }  
    }
            xSemaphoreGive( xWiFiSemaphore );                                      
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

}
