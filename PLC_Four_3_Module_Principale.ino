/*
 * Autheur :           Momar DIA
 * Date:               2020/04/115
 * Modified:           .
 * Description:        Ceci est le programme embarqué du contrôleur des fours. Ce programme contrôle la temperature du four et envoie des informations
                    sur l'êtat des fours au Scada qui se trouve sur le serveur de la compagnie par protocol modbus WiFi, en même temps ce programme
                    envoie les mêmes informations au Scada du Touchberry par modbus Ethernet. Ce programme exécute les commandes reçues des Scada
 * Nomenclarture Spéciale : 
    - 21 = thermocouple control select pin
    - 22 = thermocouple overheat select pin
    - 50 = SO PIN
    - 51 = SI PIN
    - 52 = SCK PIN

modbus registers follow the following format
00001-09999  Digital Outputs, A master device can read and write to these registers
10001-19999  Digital Inputs, A master device can only read the values from these registers
30001-39999  Analog Inputs, A master device can only read the values from these registers
40001-49999  Analog Outputs, A master device can read and write to these registers 

Analog values are 16 bit unsigned words stored with a range of 0-32767
Digital values are stored as bytes, a zero value is OFF and any nonzer value is ON

It is best to configure registers of like type into contiguous blocks.  this
allows for more efficient register lookup and and reduces the number of messages
required by the master to retrieve the data
*/


/*---------------- BIBLIOTHEQUES -------------------*/
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
  uint16_t tension;                                    //             //
  uint16_t porte;                                      //             //
  uint16_t elementChauffant;                           //             //
  uint16_t ventilation;                         
  uint16_t temp_Control;                              //             //
  uint16_t temp_OverHeat;                             //             //
  uint16_t consigne;                                  //             //
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
modbusDevice regBank;                                                        
modbusSlave slave;                                                
double Setpoint, Input, Output, Kp, Ki, Kd;                        
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);             
const uint16_t FAULT = 4000;
SemaphoreHandle_t xWiFiSemaphore;                                           
double TempControl;                                                      
double TempOverHeat;                                                  


void PID_Control( void *pvParameters );                
void Modbus( void *pvParameters );                                  
void Send_State( void *pvParameters );                                      
void Receive_Command( void *pvParameters );                                            






/*------------------- INDEXS -----------------------*/
// IO for read and write 
// I for write
// O for Read
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

                                                   
  slave._device = &regBank;                                             
  slave._device->setId(1);
  slave.setPort(&RS485);                                              
  slave.setBaud(9600);                                                           
  RS485.begin(9600);                                                             
  SD.begin(53);                                                                  
  thermoOverHeat.begin();                     
  thermoControl.begin();                                                            
  Wifi.begin(9600);                                                                 


  if ( xWiFiSemaphore == NULL )                                                
  {
    xWiFiSemaphore = xSemaphoreCreateMutex();                                    
    if ( ( xWiFiSemaphore ) != NULL )
      xSemaphoreGive( ( xWiFiSemaphore ) );                                    
  }
  
  /*--- TASKS INITIALISATION ---*/
  xTaskCreate(PID_Control, "ControlTemp", 200, NULL  , 3, &PID_Handle);
  xTaskCreate(Receive_Command, "Receive_Command", 500, NULL, 2, NULL);
  xTaskCreate(Modbus, "Modbus_RTU", 1500, NULL, 3, NULL);
  xTaskCreate(Send_State, "WiFi_COM", 500, NULL, 2, NULL);
  if(lastConfig[elChauffants])
   {
       digitalWrite(R1_1, HIGH);
   }
  delay(5000);                                                                  
  Serial.println(F("Setup Executé avec succes..."));
}




void loop() {
  // Vide les choses se passent dans les tâches
}






/*------------------- LIRE TEMPERATURE Thermocouple control(FUNCTION) -----------------------*/
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
      TempOverHeat = thermoOverHeat.readThermocoupleTemperature();                              // Lecture de la temperatue en degre celcius
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
    //taskDISABLE_INTERRUPTS();
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
          myPID.SetMode(AUTOMATIC);           // Allumer le PID 
          Input = TempControl;
          myPID.Compute();                   // Calculer si une nouvelle valeur de OUTPUT est necessaire
          Serial.print(Output);
          analogWrite(A1_2, Output);
      }
      
    }
    else
    {
         if(myPID.GetMode() == MANUAL)        // Si le PID est actif
      {
        analogWrite(A1_2, 0);
      }
      else
      {
          myPID.SetMode(MANUAL);             // Eteindre le PID
          analogWrite(A1_2, 0);        // Fermer les éléments chauffants
      }
    }
    //taskENABLE_INTERRUPTS();
    vTaskDelay(300 / portTICK_PERIOD_MS);  // one tick delay (15ms) 35 = 500ms
  }
}






void Modbus( void *pvParameters ){
  //pinMode(R1_1, OUTPUT);
  pinMode(R2_1, OUTPUT);
  SPI.begin();
  for(;;){
    //taskDISABLE_INTERRUPTS();
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
    //taskENABLE_INTERRUPTS();
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
