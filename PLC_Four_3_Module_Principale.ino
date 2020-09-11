/*
 * Autheur :           Momar DIA
 * Date:               2020/04/115
 * Modified:           .
 * Company:            Tecnickrome Aeronautique
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
#include <Adafruit_MAX31856.h>
#include <Arduino_FreeRTOS.h>
#include <modbusRegBank.h>
#include <modbusDevice.h>
#include <ArduinoJson.h>
#include <modbusSlave.h>
#include <SimpleComm.h>
#include <WifiModule.h>
#include <modbus.h>
#include <semphr.h>
#include <PID_v1.h>
#include <RS485.h>
#include <SPI.h>
#include <SD.h>


/*---------------- TYPE DEFINITION -------------------*/
typedef struct {                                      
  uint16_t tension;                                    //             //
  uint16_t porte;                                      //             //
  uint16_t elementChauffant;                           //             //
  uint16_t ventilation;                                // ETAT DU FOUR//
  uint16_t temp_Control;                              //             //
  uint16_t temp_OverHeat;                             //             //
  uint16_t consigne;                                  //             //
  uint16_t marge;                                     //             //
} State;  

typedef struct {
  uint16_t consigne;                                  // COMMANDES  //
  uint16_t marge;                                     //     DU     //
  uint16_t onOff;                                      //   MODULE   // 
  uint16_t alarm;                                      //    WIFI    //
} Command;




/*-------------------------------- INDEX --------------------------------*/
int consigneI = 0;
int margeI = 1;
int offsetPCI = 2;
int offsetMCI = 3;
int offsetPOI = 4;
int offsetMOI = 5;
int P = 6;
int I = 7;
int D = 8;
int elChauffants = 9;                                  // etat du four, en fonction ou pas
double lastConfig[10];                                 // contient une copie de la dernière configuration







/*---------------- VARIABLES -------------------*/
const String ConfigurationFile = "config.txt";                              // FICHIER CONFIG
const char *filename = "config.txt";  
Adafruit_MAX31856 thermoControl = Adafruit_MAX31856(21);             // Variable Thermocouple Control
Adafruit_MAX31856 thermoOverHeat = Adafruit_MAX31856(20);             // Variable Thermocouple Control
modbusDevice regBank;                                                        // Modbus data bank
modbusSlave slave;                                                           // Modbus slave protocol handler
double Setpoint, Input, Output, Kp, Ki, Kd;                                  // PID In Out variables
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);                   // Modbus computer Object
const uint16_t FAULT = 4000;
SemaphoreHandle_t xWiFiSemaphore;                                            // Deux tâches ne peuvent pas accéder le WIFI en même temps
double TempControl;                                                          // Temperature de controle du four en Décimal pour le PID
double TempOverHeat;                                                         // Temperature OverHeat en décimal


/*---------------- TASKS -------------------*/
void PID_Control( void *pvParameters );                                       // PID control task
void Overheat_Control( void *pvParameters );                                  // Overheat control task
void Save_Config( void *pvParameters );                                       // save configuration
void Modbus_RTU( void *pvParameters );                                        // Modbus Com RTU HMI (dans notre cas le touch berry)
void Send_State( void *pvParameters );                                        // Send Oven state to WiFi module
void Receive_Command( void *pvParameters );                                   // Receive Command from WiFi Module (Serveur)
TaskHandle_t PID_Handle;                                                      // Handler to pause or resume the task






/*------------------- INDEXS -----------------------*/
// IO for read and write 
// I for write
// O for Read
const int Otension =           10001;                                          // Témoins de tension du four  (I0_0)
const int Oporte =             10002;                                          // Témoins de la porte du four  (I0_1)
const int OelementChauffant =  10003;                                          // Témoins des élements chauffants  (I0_2)
const int Oventilation =       10004;                                          // Témoins de la ventilation       (I0_3)
const int IOnOff =             00001;                                          // Bouton ON/OFF   (R1_1 PIN)
const int IAlarm =             00002;                                          // Bonton Alarm    (R2_1 PIN)
const int OtempControl =       30001;                                          // Température ambiante du Four Thermocouple de controle
const int OtempOverHeat =      30002;                                          // Température ambiante du Four Thermocouple OverHeat
const int IOconsigne =         40001;                                          // Consigne de chauffe
const int IOmarge =            40002;                                          // Marge de température
const int IOOffsetPlus =       40003;                                          // Bouton du Offset
const int KP =                 40004;                                          // Constante P du PID X 10 
const int KI =                 40005;                                          // Constante I du PID X 10 
const int KD =                 40006;                                          // Constante D du PID X 10 
const int IOOffsetOvHeatPlus = 40007;
const int IOOffsetMoins =      40008;
const int IOOffsetOvHeatMoins =40009;
bool justStarted;



/*------------------- SETUP -----------------------*/
// RUN ONCE AT START
void setup() {
  justStarted = true;
  Serial.println(F("Début du Setup..."));
  Serial.begin(9600);                                                            // initialize the serial Monitor with a baud rate of 9600

  /*--- MODBUS VARIABLES ---*/
  regBank.add(Otension);                                           
  regBank.add(Oporte);                                          
  regBank.add(OelementChauffant);                                  
  regBank.add(Oventilation);
  regBank.add(OtempControl);                                        
  regBank.add(OtempOverHeat);                                      
  regBank.add(IOconsigne);                                               
  regBank.add(IOmarge);                                                       
  regBank.add(IOnOff);                                            
  regBank.add(IAlarm);                                                     
  regBank.add(IOOffsetPlus);
  regBank.add(KP);
  regBank.add(KI);
  regBank.add(KD);  
  regBank.add(IOOffsetOvHeatPlus);
  regBank.add(IOOffsetMoins);
  regBank.add(IOOffsetOvHeatMoins);

                                                   
  slave._device = &regBank;                                                      // Passe la banque de registre au modbus
  slave._device->setId(1);                                                       // Donne le ID du device
  slave.setPort(&RS485);                                                         // MODBUS HMI (touchberry) port
  slave.setBaud(9600);                                                           // Set modbus baud rate to 9600
  RS485.begin(9600);                                                             // Initialize the Modbus Serial Port
  SD.begin(53);                                                                  // Initialize the sd card module
  thermoOverHeat.begin();
  thermoOverHeat.setNoiseFilter(MAX31856_NOISE_FILTER_60HZ);
  thermoOverHeat.setThermocoupleType(MAX31856_TCTYPE_J);                          // Configuration du type de Thermocouple ('J' pour thermocouple de type J)
  thermoControl.begin();                                                         // Initialisation du thermocouple control
  thermoControl.setNoiseFilter(MAX31856_NOISE_FILTER_60HZ);
  thermoControl.setThermocoupleType(MAX31856_TCTYPE_J);                          // Configuration du type de Thermocouple ('J' pour thermocouple de type J)
  WifiModule.begin(9600);                                                        // Initialisation de la communication avec le ESP32(WiFi module)
  SimpleComm.begin();                                                            // Initialisation du protocole de communication avec le ESP32
  loadConfiguration();                                                            // Charge la dernière configuration enregistrée du four dans la carte SD      


  if ( xWiFiSemaphore == NULL )                                                  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xWiFiSemaphore = xSemaphoreCreateMutex();                                    // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xWiFiSemaphore ) != NULL )
      xSemaphoreGive( ( xWiFiSemaphore ) );                                      // Make the Serial Port available for use, by "Giving" the Semaphore.
  }
  
  /*--- TASKS INITIALISATION ---*/
  xTaskCreate(PID_Control, "ControlTemp", 200, NULL  , 3, &PID_Handle);
  xTaskCreate(Overheat_Control, "ControlOverheat", 200, NULL, 2, NULL);
  xTaskCreate(Receive_Command, "Receive_Command", 500, NULL, 2, NULL);
  xTaskCreate(Save_Config, "saveConfig", 1000, NULL, 1, NULL);
  xTaskCreate(Modbus_RTU, "Modbus_RTU", 1500, NULL, 3, NULL);
  xTaskCreate(Send_State, "WiFi_COM", 500, NULL, 2, NULL);
  pinMode(R1_1, OUTPUT);
  if(lastConfig[elChauffants])
   {
       digitalWrite(R1_1, HIGH);
   }
  delay(5000);                                                                  // Wait for the wifi Module to start in order to avoid filling his serial buffer
  Serial.println(F("Setup Executé avec succes..."));
}




void loop() {
  // Vide les choses se passent dans les tâches
}






/*------------------- LIRE TEMPERATURE Thermocouple control(FUNCTION) -----------------------*/
void LireTemperatureControl()
{
    TempControl = thermoControl.readThermocoupleTemperature();                              // Lecture de la temperatue en degre celcius
    TempControl = (TempControl * 1.8) + 32;                                                 // Conversion en Farenheit
    if(TempControl < 0 || TempControl > 2000)
    {
      TempControl = thermoControl.readThermocoupleTemperature();                              // Lecture de la temperatue en degre celcius
      TempControl = (TempControl * 1.8) + 32; 
    }
    TempControl = TempControl + regBank.get(IOOffsetPlus) - regBank.get(IOOffsetMoins);                                                                       // Ajout de l'offset
    regBank.set(OtempControl, TempControl);                                                 // Cast en int et enregistrer dans la banque de registre
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






/*------------------- LoadConfiguration(Function) -----------------------*/
void loadConfiguration() {
  SD.begin(53);                                                                 // just in case
  File file = SD.open(ConfigurationFile);                                       // Création du lecteur de fichier
  StaticJsonDocument<175> doc;                                                   // Création du Buffer qui va contenir la chaine
  DeserializationError error = deserializeJson(doc, file);                      // conversion de la chaine de caractère en Jason
  if (error){
    Serial.println(F("Failed to read file, using default configuration"));
    Serial.println(error.c_str());}

  // Copy values from the JsonDocument to the Config
  regBank.set(IOconsigne, doc["consigne"] | 375);
  regBank.set(IOOffsetPlus, doc["offsetPlus"] | 1);
  regBank.set(IOmarge, doc["marge"] | 25);
  regBank.set(IOOffsetOvHeatPlus, doc["offsetOvPlus"] | 1);
  regBank.set(IOOffsetOvHeatMoins, doc["offsetOvMoins"] | 1);
  regBank.set(IOOffsetMoins, doc["offsetMoins"] | 1);
  Kp = doc["Kp"] | 2;
  Ki = doc["Ki"] | 2;
  Kd = doc["Kd"] | 2;
  regBank.set(OelementChauffant, doc["elCh"] | 0);
  regBank.set(IOnOff, doc["elCh"] | 0);
  Setpoint = regBank.get(IOconsigne);
  regBank.set(KP, Kp);
  regBank.set(KI, Ki);
  regBank.set(KD, Kd);
  
  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
  updateConfig();
  Serial.println(F("Configuration chargée avec success"));
}







/*------------------- PID_CONTROL(TASK) -----------------------*/
void PID_Control(void *pvParameters) 
{
  (void) pvParameters;
  for (;;)
  {
    //taskDISABLE_INTERRUPTS();
    if(digitalRead(I0_0))                     // Si le four est allumé 
    {
      if(myPID.GetMode() == AUTOMATIC)        // SI le PID est actif
      {
          Input = TempControl;
          myPID.Compute();                    // Calculer si une nouvelle valeur de OUTPUT est necessaire
          Serial.print(Output);
          analogWrite(A1_2, Output);    // Envoyer la commande au four
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







/*------------------- OverHeat_CONTROL(TASK) -----------------------*/
void Overheat_Control(void *p)
{
  for (;;)
  {
    LireTemperatureControl();
    LireTemperatureOverHeat();
    //taskDISABLE_INTERRUPTS();
       if(regBank.get(OtempOverHeat) >= ( regBank.get(IOconsigne) + regBank.get(IOmarge)))        // s'il y a une surchauffe
          {
             myPID.SetMode(MANUAL);
             regBank.set(IAlarm, 255);            // Sinon allumer l'alarme
             analogWrite(A1_2, 0);   // Eteindre les éléments chauffants
             vTaskSuspend( PID_Handle );  // suspension du processus PID_Control jusqu'à ce que le overheat finisse
                 
           }
        else                                                            // Si on est plus en overheat
           {}  
    
      //taskENABLE_INTERRUPTS();
      vTaskDelay(1000 / portTICK_PERIOD_MS);  // one tick delay (15ms) in between reads for stability
    }
    
  }







/*------------------- Save_Config(TASK) -----------------------*/
void Save_Config( void *pvParameters ){
  //SD.begin(53);
  for(;;){
    if(checkChange())
    {
    taskENTER_CRITICAL();
    SD.remove(filename);                                     //Suprime le dernier fichier config créé
    File file = SD.open(ConfigurationFile, FILE_WRITE);                  // Ouvre le fichier config, s'il n'existe pas il sera créé
    delay(100);
    if (!file) {
      Serial.println(F("Failed to create the save configuration file"));
      }
      
    StaticJsonDocument<150> doc;                                     // Création du buffer Jason
    /*--- enregistrement des valeurs dans le document ---*/
    doc["consigne"] = regBank.get(IOconsigne);
    doc["marge"] = regBank.get(IOmarge);
    doc["offsetPlus"] = regBank.get(IOOffsetPlus);
    doc["offsetOvPlus"] = regBank.get(IOOffsetOvHeatPlus);
    doc["offsetMoins"] = regBank.get(IOOffsetMoins);
    doc["offsetOvMoins"] = regBank.get(IOOffsetOvHeatMoins);
    doc["Kp"] = Kp;
    doc["Ki"] = Ki;
    doc["Kd"] = Kd;
    doc["elCh"] = regBank.get(OelementChauffant);
    //if(serializeJson(doc, file) == 0) {
    //  serializeJson(doc, file);
    //}
      serializeJson(doc, file);                     // Sérialize json to the File
      file.close();                                                 // Close the file
      //}
    //else{
     // Serial.println(F("CONFIGURATION FILE SAVE FAILLED"));
      //}
    //taskENABLE_INTERRUPTS();
    taskEXIT_CRITICAL();
    updateConfig();
    }
    else
    {}
    vTaskDelay(600000 / portTICK_PERIOD_MS);
    }
    
  }








/*------------------- Modbus_RTU(TASK) -----------------------*/
void Modbus_RTU( void *pvParameters ){
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








/*------------------- Send_State(TASK) -----------------------*/
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
      packet.setData(&state, sizeof(state));                                            // Create packet from data
      SimpleComm.send(WifiModule, packet, 0);                                          // Send Packet                               
      xSemaphoreGive( xWiFiSemaphore );                                                 // Now free or "Give" the WiFi Port for others.
      }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }









/*------------------- Receive_Command(TASK) -----------------------*/
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
            xSemaphoreGive( xWiFiSemaphore );                                       // Now free or "Give" the Serial Port for others.
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

}


/*------------------------------- Met à jour last config --------------------------*/
void updateConfig()
{
    lastConfig[consigneI] = regBank.get(IOconsigne);
    lastConfig[margeI] = regBank.get(IOmarge);
    lastConfig[offsetPCI] = regBank.get(IOOffsetPlus);
    lastConfig[offsetMCI] = regBank.get(IOOffsetMoins);
    lastConfig[offsetPOI] = regBank.get(IOOffsetOvHeatPlus);
    lastConfig[offsetMOI] = regBank.get(IOOffsetOvHeatMoins);
    lastConfig[P] = regBank.get(KP);
    lastConfig[I] = regBank.get(KI);
    lastConfig[D] = regBank.get(KD);
    lastConfig[elChauffants] = regBank.get(OelementChauffant);
}


/*------------------------------- Vérifie si last config est à jour --------------------------*/
bool checkChange()
{
  if(lastConfig[consigneI] != Setpoint)
  {
    return true;
  }  

  else if(lastConfig[margeI] != regBank.get(IOmarge))
  {
    return true;  
  }
  else if(lastConfig[offsetPCI] != regBank.get(IOOffsetPlus))
  {
    return true;  
  }
  else if(lastConfig[offsetMCI] != regBank.get(IOOffsetMoins))
  {
    return true;  
  }
  else if(lastConfig[offsetPOI] != regBank.get(IOOffsetOvHeatPlus))
  {
    return true;  
  }
  else if(lastConfig[offsetMOI] != regBank.get(IOOffsetOvHeatMoins))
  {
    return true;  
  }
  else if(lastConfig[P] != regBank.get(KP))
  {
    return true;  
  }
  else if(lastConfig[I] != regBank.get(KI))
  {
    return true;  
  }
  else if(lastConfig[D] != regBank.get(KD))
  {
    return true;  
  }
  else if(lastConfig[elChauffants] != regBank.get(OelementChauffant))
  {
    return true;  
  }
  return false;
}

  
