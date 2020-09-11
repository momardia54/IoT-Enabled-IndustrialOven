/*
 * Autheur :           Momar DIA
 * Date:               2020/04/17
 * Modified:           .
 * Company:            Tecnickrome Aeronautique Inc
 * License:            Ce programme est la propriété intellectuelle de Tecnickrome Aéronautique Inc et en aucun cas ne peut être dupliqué ni réutiliser sans autorisation.
 * Description:        Ce programme permet la communication en WiFi avec le serveur. Il recoit l'etat du four from le module principale par Serial2 et l'envoie 
                       au serveur par modbus ip puis retransmet les commandes du serveur au module principale par le port serial2. s'il n'y a pas de commande, il
                       envoie FAULT qui est une valeur par défaut 4000 qui signifi dans notre code null ou pas de valeur.
                       Seulement les HOLDING registeurs sont utilisés dans notre cas car modubus Wifi support un type de registre à la fois

Analog values are 16 bit unsigned words stored with a range of 0-32767
Digital values are stored as bytes, a zero value is OFF and any nonzer value is ON

It is best to configure registers of like type into contiguous blocks.  this
allows for more efficient register lookup and and reduces the number of messages
required by the master to retrieve the data
*/



/*---------------- BIBLIOTHEQUES -------------------*/
#include <ModbusIP_ESP32.h>
#include <SimpleComm.h>
#include <Modbus.h>
#include <WiFi.h>





/*---------------- VARIABLES -------------------*/
const uint16_t FAULT = 4000;                                            // Command value to replace de NULL Value cause Simple comme does not accept NULL and replace it with 0
SimplePacket packet;
ModbusIP mb;                                                            // Etats et commandes du four
SemaphoreHandle_t xSerial2Semaphore;                                     // Permet que 2 tâches communiquent pas en même temps avec le module principale


/*------------------- Tasks ------------------------*/
TaskHandle_t TaskReceive_State;                                          // Receive the state of the oven from the main module
TaskHandle_t TaskModbus_WiFi;                                            // Communicate Modbus with the server and send the Command to the main board
TaskHandle_t TaskCheck_WiFi;                                             // Check the status of the WiFi connection. If not connecter, try to reconnect it





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




// C'est le mapping des holding registers dans le scada en commençant par 0
// Certains Scada commencent pas 1 donc être attentif en conséquence 
/*------------------- Indexs -----------------------*/
// IO for read and write 
// I for write 
// O for Read
  const int Otension = 0;
  const int Oporte = 1;
  const int OelementChauffant = 2;
  const int Oventilation = 3;
  const int OtempControl = 4;
  const int OtempOverHeat = 5;
  const int Oconsigne = 6;
  const int Omarge = 7;
  const int IOnOff = 8;
  const int IAlarm = 9; 
  const int Iconsigne = 10;
  const int Imarge = 11;







/*------------------- TASKS -----------------------*/
  void Receive_State( void * pvParameters );
  void Modbus_WiFi( void * pvParameters );
  void Check_WiFi( void * pvParameters );

//   Wifi bureau "TecnickromeBureau", "net1516cet"

/*------------------- Setup, run only at the start -----------------------*/
void setup() {
  delay(10000);
  Serial.begin(9600);                                                           // Initialize serial for debugging
  Serial2.begin(9600);                                                          // Initialize serial 2 for com with main board
  SimpleComm.begin();                                                           // Initialize com protocol with main board
  Serial.println(F("esp32 started"));
  mb.config("IOT", "");                                      // configuration du modbus et connection au WiFi
  delay(5000);
  /*--- Modbus INITIALISATION ---*/
  mb.addHreg(Otension, FAULT);
  mb.addHreg(Oporte, FAULT);
  mb.addHreg(OelementChauffant, FAULT);
  mb.addHreg(Oventilation, FAULT);
  mb.addHreg(OtempControl, FAULT);
  mb.addHreg(OtempOverHeat, FAULT);
  mb.addHreg(Oconsigne, FAULT);
  mb.addHreg(Omarge, FAULT);
  mb.addHreg(Iconsigne, FAULT);
  mb.addHreg(Imarge, FAULT);
  mb.addHreg(IOnOff, FAULT);
  mb.addHreg(IAlarm, FAULT);

  /*--- Semaphore Initialization ---*/
  if ( xSerial2Semaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerial2Semaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerial2Semaphore ) != NULL )
      xSemaphoreGive( ( xSerial2Semaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }
  /*--- TASK INITIALISATION ---*/
  xTaskCreatePinnedToCore(
             Modbus_WiFi,  /* Task function. */
             "TaskModbus_WiFi",    /* name of task. */
             50000,      /* Stack size of task */
             NULL,       /* parameter of the task */
             2,          /* priority of the task */
             &TaskModbus_WiFi,     /* Task handle to keep track of created task */
             1);         /* pin task to core 0 */


  xTaskCreatePinnedToCore(
             Check_WiFi,  /* Task function. */
             "Check_WiFi",    /* name of task. */
             10000,      /* Stack size of task */
             NULL,       /* parameter of the task */
             3,          /* priority of the task */
             &TaskCheck_WiFi,     /* Task handle to keep track of created task */
             1);         /* pin task to core 0 */


  xTaskCreatePinnedToCore(
             Receive_State,  /* Task function. */
             "Receive_State",    /* name of task. */
             10000,      /* Stack size of task */
             NULL,       /* parameter of the task */
             3,          /* priority of the task */
             &TaskReceive_State,     /* Task handle to keep track of created task */
             0);         /* pin task to core 0 */
Serial.println(F("Setup Successful !! "));

} 







void loop() {
  // Vide les choses se passent dans les tâches
}






/*------------------- Receive State from main module(TASK) -----------------------*/
void Receive_State( void * pvParameters ){
  
  for(;;)
  {
    //taskDISABLE_INTERRUPTS();
    Serial.println(F("                                                              Receive_State"));

    if ( xSemaphoreTake( xSerial2Semaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
          
          if (SimpleComm.receive(Serial2, packet)) {                                    
             const State *state = (const State *) packet.getData();
            if (state != nullptr) {
                mb.Hreg(Otension, state->tension);
                mb.Hreg(Oporte, state->porte);
                mb.Hreg(OelementChauffant, state->elementChauffant);
                mb.Hreg(Oventilation, state->ventilation);
                mb.Hreg(OtempControl, state->temp_Control);
                mb.Hreg(OtempOverHeat, state->temp_OverHeat);
                mb.Hreg(Oconsigne, state->consigne);
                mb.Hreg(Omarge, state->marge);
                // Print received data
                Serial.print("  temp ");
                Serial.print(mb.Hreg(OtempOverHeat));
            }
            
            
          }
          xSemaphoreGive( xSerial2Semaphore ); // Now free or "Give" the Serial Port for others.
        }
     // taskENABLE_INTERRUPTS();
      vTaskDelay(900 / portTICK_PERIOD_MS);
    }

}








/*------------------- Check Wifi Connection(TASK) -----------------------*/
void Check_WiFi( void * pvParameters ){
  
  for(;;){
      Serial.println("                                                              Check_WiFi");
      if (WiFi.status() != WL_CONNECTED) {
          Serial.println(F(" Wifi Not connected "));
          ESP.restart();
          }                
    vTaskDelay(60000 / portTICK_PERIOD_MS);                                               // délais d'execution 10min
    }
}






/*------------------- Communicate modbus with the server(TASK) -----------------------*/
void Modbus_WiFi( void * pvParameters ){
  Command command;
  
  for(;;)
  {
    
      //taskDISABLE_INTERRUPTS();
      Serial.println(F("                                                              Modbus_WiFi"));
      Serial.println(xPortGetFreeHeapSize());
      mb.task();                                                                          // Where the Magic modbus happens
      delay(50);
      if(mb.Hreg(Iconsigne) == 0 && mb.Hreg(Imarge) == 0)
      {
            mb.Hreg(Iconsigne, FAULT);                                                            // peut the value to null to avoid keep sending value if there is no command
            mb.Hreg(Imarge, FAULT);
            mb.Hreg(IOnOff, FAULT);
            mb.Hreg(IAlarm, FAULT);
      }
      delay(150);
      
      if ( xSemaphoreTake( xSerial2Semaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
        command.consigne = mb.Hreg(Iconsigne);
        command.marge = mb.Hreg(Imarge);
        command.onOff = mb.Hreg(IOnOff);
        command.alarm = mb.Hreg(IAlarm);
        packet.setData(&command, sizeof(command));                                        // prepare the command
    
        // Send packet
        SimpleComm.send(Serial2, packet, 0);                                              // send command to the main module

      xSemaphoreGive( xSerial2Semaphore ); // Now free or "Give" the Serial Port for others.
    }
    mb.Hreg(Iconsigne, FAULT);                                                            // peut the value to null to avoid keep sending value if there is no command
    mb.Hreg(Imarge, FAULT);
    mb.Hreg(IOnOff, FAULT);
    mb.Hreg(IAlarm, FAULT);
      //taskENABLE_INTERRUPTS();
      vTaskDelay(1000 / portTICK_PERIOD_MS);                     // délais d'execution
      
  }
}
