#include <ModbusIP>
#include <Comm.h>
#include <Modbus.h>
#include <WiFi.h>



const uint16_t FAULT = 4000;                                           
Packet packet;
ModbusIP mb;                                                            

Task Receive_State;                                         
Task Modbus_WiFi;                                           
Task Check_WiFi;                                            




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






  void Receive_State( void * pvParameters );
  void Modbus_WiFi( void * pvParameters );
  void Check_WiFi( void * pvParameters );

void setup() {
  delay(10000);
  Serial.begin(600);                                                      
  Serial2.begin(600);                                                      
  Comm.begin();                   
  mb.config("IOT", "");                        
  delay(5000);
  
  
  mb.add(Otension, FAULT);
  mb.add(Oporte, FAULT);
  mb.add(OelementChauffant, FAULT);
  mb.add(Oventilation, FAULT);
  mb.add(OtempControl, FAULT);
  mb.add(OtempOverHeat, FAULT);
  mb.add(Oconsigne, FAULT);
  mb.add(Omarge, FAULT);
  mb.add(Iconsigne, FAULT);
  mb.add(Imarge, FAULT);
  mb.add(IOnOff, FAULT);
  mb.add(IAlarm, FAULT);

  if ( Semaphore == NULL 
  {
    Semaphore = SemaphoreCreate();  
    if ( ( Semaphore ) != NULL )
      SemaphoreGive( ( Semaphore ) );  
  }
  Create(
             Modbus_WiFi,
             "Modbus_WiFi",  
             1,    
             NULL,    
             2,   
             &Modbus_WiFi,   
             1);   


  Create(
             Check_WiFi,  
             "Check_WiFi",  
             1,     
             NULL,   
             1,     
             &Check_WiFi,    
             1);       


  Create(
             Receive_State
             "Receive_State",  
             1,      
             NULL,    
             1,       
             &Receive_State,   
             0);     
Serial.println(F("Setup Successful !! "));

} 







void loop() {
 
}



void Receive_State( void * pvParameters ){
  
  for(;;)
  {

    if ( SemaphoreTake( Semaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
          
          
            if (state != nullptr) {
                mb.set(Otension, state->tension);
                mb.set(Oporte, state->porte);
                mb.set(OelementChauffant, state->elementChauffant);
                mb.set(Oventilation, state->ventilation);
                mb.set(OtempControl, state->temp_Control);
                mb.set(OtempOverHeat, state->temp_OverHeat);
                mb.set(Oconsigne, state->consigne);
                mb.set(Omarge, state->marge);
                Serial.print("  temp ");
                Serial.print(mb.Hreg(OtempOverHeat));
            
            
          }
          SemaphoreGive( Semaphore ); 
        }
      Delay(900 / PERIOD_MS);
    }

}





void Check_WiFi( void * pvParameters ){
  
  for(;;){
      if (WiFi.status() != WL_CONNECTED) {
          Serial.println(F(" Wifi Not connected "));
          restart();
          }                
    Delay(60000 / portTICK_PERIOD_MS);                          
    }
}




void Modbus_WiFi( void * pvParameters ){
  Command command;
  
  for(;;)
  {
    
      mb.execute();                                                                      
      delay(50);
      
      
      if ( SemaphoreTake( Semaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
        command.consigne = mb.set(Iconsigne);
        command.marge = mb.set(Imarge);
        command.onOff = mb.set(IOnOff);
        command.alarm = mb.set(IAlarm);
        packet.setData(&command, sizeof(command));                                      
    
        Comm.send(Serial2, packet, 0);                                       

      xSemaphoreGive( xSerial2Semaphore ); 
    }
    mb.set(Iconsigne, FAULT);                                                         
    mb.set(Imarge, FAULT);
    mb.set(IOnOff, FAULT);
    mb.set(IAlarm, FAULT);
    Delay(1000 / PERIOD_MS);              
      
  }
}
