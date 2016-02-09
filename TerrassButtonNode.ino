#include <Wire.h>
#include <PortExpander_I2C.h>
#include <MySensor.h>
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <avr/wdt.h>
#include <SimpleTimer.h>


 //#define NDEBUG                        // enable local debugging information

#define SKETCH_NAME "Terrass light switch"
#define SKETCH_MAJOR_VER "1"
#define SKETCH_MINOR_VER "0"
#define NODE_ID 20 //or AUTO to let controller assign     

#define RELAY1_CHILD_ID 30
#define RELAY2_CHILD_ID 31
#define RELAY3_CHILD_ID 32
#define RELAY4_CHILD_ID 33
#define RELAY5_CHILD_ID 34

#define RELAY1_STATUS_CHILD_ID 50
#define RELAY2_STATUS_CHILD_ID 51
#define RELAY3_STATUS_CHILD_ID 52
#define RELAY4_STATUS_CHILD_ID 53
#define RELAY5_STATUS_CHILD_ID 54


#define BUTTON1_CHILD_ID 40
#define BUTTON2_CHILD_ID 41
#define BUTTON3_CHILD_ID 42
#define BUTTON4_CHILD_ID 43
#define BUTTON5_CHILD_ID 44

#define BUTTON1DELAYED_CHILD_ID 45
#define BUTTON2DELAYED_CHILD_ID 46
#define BUTTON3DELAYED_CHILD_ID 47
#define BUTTON4DELAYED_CHILD_ID 48
#define BUTTON5DELAYED_CHILD_ID 49

#define TEMP1_CHILD_ID	60
#define TEMP2_CHILD_ID	61

#define REBOOT_CHILD_ID                       100
#define RECHECK_SENSOR_VALUES                 101 
#define LOCAL_SWITCHING_CHILD_ID              106


#define BUTTON1_PIN	0
#define BUTTON2_PIN	1
#define BUTTON3_PIN	2
#define BUTTON4_PIN	3
#define BUTTON5_PIN	4

#define LED1_PIN	A0
#define LED2_PIN	A1
#define LED3_PIN	A2
#define LED4_PIN	A3

#define RELAY1_PIN	3
#define RELAY2_PIN	4
#define RELAY3_PIN	5
#define RELAY4_PIN	6
#define RELAY5_PIN	7

#define ONE_WIRE_BUS              8      // Pin where dallase sensor is connected 

#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0 // GPIO value to write to turn off attached relay

#define TEMPCHECK_TIME 120000

PortExpander_I2C pe(0x20);

//int buttonPin = 0; 
//int ledPin = A0; 
//boolean ledState;
const unsigned long debounceTime = 10;  // milliseconds
unsigned long switch1PressTime;  // when the switch last changed state
unsigned long switch2PressTime;  // when the switch last changed state
unsigned long switch3PressTime;  // when the switch last changed state
unsigned long switch4PressTime;  // when the switch last changed state
unsigned long switch5PressTime;  // when the switch last changed state

unsigned long switch1Time;  // when the switch last changed state
unsigned long switch2Time;  // when the switch last changed state
unsigned long switch3Time;  // when the switch last changed state
unsigned long switch4Time;  // when the switch last changed state
unsigned long switch5Time;  // when the switch last changed state


byte oldSwitch1State = HIGH;  // assume switch open because of pull-up resistor
byte oldSwitch2State = HIGH;  // assume switch open because of pull-up resistor
byte oldSwitch3State = HIGH;  // assume switch open because of pull-up resistor
byte oldSwitch4State = HIGH;  // assume switch open because of pull-up resistor
byte oldSwitch5State = HIGH;  // assume switch open because of pull-up resistor

boolean bRelay1State=false;
boolean bRelay2State=false;
boolean bRelay3State=false;
boolean bRelay4State=false;
boolean bRelay5State=false;


boolean bRelay1DelayMessageSent=false;
boolean bRelay2DelayMessageSent=false;
boolean bRelay3DelayMessageSent=false;
boolean bRelay4DelayMessageSent=false;
boolean bRelay5DelayMessageSent=false;

boolean localSwitching = true;

#define RADIO_RESET_DELAY_TIME 50 //Задержка между сообщениями
#define MESSAGE_ACK_RETRY_COUNT 5  //количество попыток отсылки сообщения с запросом подтверждения
#define DATASEND_DELAY  10

boolean gotAck=false; //подтверждение от гейта о получении сообщения 
int iCount = MESSAGE_ACK_RETRY_COUNT;

boolean boolRecheckSensorValues = false;

OneWire oneWire(ONE_WIRE_BUS);        // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);  // Pass the oneWire reference to Dallas Temperature. 
unsigned long previousTempMillis=0;
float lastTemp1 = -1;
float lastTemp2 = -1;

MySensor sensor_node;

MyMessage msgButton1(BUTTON1_CHILD_ID, V_LIGHT);
MyMessage msgButton2(BUTTON2_CHILD_ID, V_LIGHT);
MyMessage msgButton3(BUTTON3_CHILD_ID, V_LIGHT);
MyMessage msgButton4(BUTTON4_CHILD_ID, V_LIGHT);
MyMessage msgButton5(BUTTON5_CHILD_ID, V_LIGHT);

MyMessage msgRelay1Status(RELAY1_STATUS_CHILD_ID, V_STATUS);
MyMessage msgRelay2Status(RELAY2_STATUS_CHILD_ID, V_STATUS);
MyMessage msgRelay3Status(RELAY3_STATUS_CHILD_ID, V_STATUS);
MyMessage msgRelay4Status(RELAY4_STATUS_CHILD_ID, V_STATUS);
MyMessage msgRelay5Status(RELAY5_STATUS_CHILD_ID, V_STATUS);

MyMessage msgRelay1(RELAY1_CHILD_ID, V_STATUS);
MyMessage msgRelay2(RELAY2_CHILD_ID, V_STATUS);
MyMessage msgRelay3(RELAY3_CHILD_ID, V_STATUS);
MyMessage msgRelay4(RELAY4_CHILD_ID, V_STATUS);
MyMessage msgRelay5(RELAY5_CHILD_ID, V_STATUS);


MyMessage msgRelay1Delayed(BUTTON1DELAYED_CHILD_ID, V_STATUS);
MyMessage msgRelay2Delayed(BUTTON2DELAYED_CHILD_ID, V_STATUS);
MyMessage msgRelay3Delayed(BUTTON3DELAYED_CHILD_ID, V_STATUS);
MyMessage msgRelay4Delayed(BUTTON4DELAYED_CHILD_ID, V_STATUS);
MyMessage msgRelay5Delayed(BUTTON5DELAYED_CHILD_ID, V_STATUS);


MyMessage msgLocalSwitching(LOCAL_SWITCHING_CHILD_ID, V_STATUS);

MyMessage msgTemp1(TEMP1_CHILD_ID, V_TEMP);
MyMessage msgTemp2(TEMP2_CHILD_ID, V_TEMP);

void setup() {
  // put your setup code here, to run once:
  pe.init();
 Serial.begin(115200);

  for( int i = 0; i < 8; i++ ){
    pe.pinMode(i,INPUT);
  } 

pinMode(A0, OUTPUT);
pinMode(A1, OUTPUT);
pinMode(A2, OUTPUT);
pinMode(A3, OUTPUT);

pinMode(8, INPUT);

  // requestTemperatures() will not block current thread
  sensors.setWaitForConversion(false);


  sensor_node.begin(incomingMessage, NODE_ID, false);

  sensor_node.wait(RADIO_RESET_DELAY_TIME);
  sensor_node.sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER"."SKETCH_MINOR_VER);


  	//relays
    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(RELAY1_CHILD_ID, S_LIGHT);   

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(RELAY2_CHILD_ID, S_LIGHT);   

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(RELAY3_CHILD_ID, S_LIGHT);   

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(RELAY4_CHILD_ID, S_LIGHT);   

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(RELAY5_CHILD_ID, S_LIGHT);   


  	//relays status
    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(RELAY1_STATUS_CHILD_ID, S_BINARY);   

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(RELAY2_STATUS_CHILD_ID, S_BINARY);   

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(RELAY3_STATUS_CHILD_ID, S_BINARY);   

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(RELAY4_STATUS_CHILD_ID, S_BINARY);   

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(RELAY5_STATUS_CHILD_ID, S_BINARY);   


    //buttons S_LIGHT
    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(BUTTON1_PIN, S_LIGHT);  

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(BUTTON2_PIN, S_LIGHT);  

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(BUTTON3_PIN, S_LIGHT);  

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(BUTTON4_PIN, S_LIGHT);  

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(BUTTON5_PIN, S_LIGHT);    


    //temperature sensors
    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(TEMP1_CHILD_ID, S_TEMP);     
    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(TEMP2_CHILD_ID, S_TEMP);  


    //reboot sensor command
    sensor_node.wait(RADIO_RESET_DELAY_TIME);
    sensor_node.present(REBOOT_CHILD_ID, S_BINARY); //, "Reboot node sensor", true); 

    //reget sensor values
    sensor_node.wait(RADIO_RESET_DELAY_TIME);
  	sensor_node.present(RECHECK_SENSOR_VALUES, S_LIGHT);     

    //local switching
    sensor_node.wait(RADIO_RESET_DELAY_TIME);
  	sensor_node.present(LOCAL_SWITCHING_CHILD_ID, S_BINARY);    

  	//checkTempTimer.setInterval(10000, checkTemperature);

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.request(RELAY1_CHILD_ID, V_LIGHT);
    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.request(RELAY2_CHILD_ID, V_LIGHT);
    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.request(RELAY3_CHILD_ID, V_LIGHT);
    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.request(RELAY4_CHILD_ID, V_LIGHT);
    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.request(RELAY5_CHILD_ID, V_LIGHT);
    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.request(LOCAL_SWITCHING_CHILD_ID, V_STATUS);    
}

void loop() {

chechButton1();
chechButton2();
chechButton3();
chechButton4();
chechButton5();

checkTemperature();

sensor_node.process();


if (boolRecheckSensorValues)
{

  boolRecheckSensorValues = false;
  resendRelayStatus();
}

}


void switchRelayON_OFF( byte RelayPin, byte Status )
{
	digitalWrite(RelayPin, Status ); //switch relay

	if ( RelayPin > 3 )
	{
		digitalWrite( 10 + RelayPin, Status ); //switch led
	}
}


void incomingMessage(const MyMessage &message) {

  if (message.isAck())
  {
    gotAck = true;
    return;
  }

    if ( message.sensor == REBOOT_CHILD_ID && message.getBool() == true && strlen(message.getString())>0 ) {
             wdt_enable(WDTO_30MS);
              while(1) {};

     }
     
     if (message.type==V_LIGHT && strlen(message.getString())>0 ) 
     {

     	switch(message.sensor)
     	{
     		case RELAY1_CHILD_ID:

     		   bRelay1State = message.getBool();
     		   switchRelayON_OFF( RELAY1_PIN, message.getBool()?RELAY_ON:RELAY_OFF );

     		   			//Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgRelay1Status.set(message.getBool()?"1":"0"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;	

     			break;
     		case RELAY2_CHILD_ID:

     		   bRelay2State = message.getBool();
     		   switchRelayON_OFF( RELAY2_PIN, message.getBool()?RELAY_ON:RELAY_OFF );
     		   	
     		   			//Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgRelay2Status.set(message.getBool()?"1":"0"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;	

     			break;
     		case RELAY3_CHILD_ID:

     		   bRelay3State = message.getBool();
     		   switchRelayON_OFF( RELAY3_PIN, message.getBool()?RELAY_ON:RELAY_OFF );
  
       		   			//Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgRelay3Status.set(message.getBool()?"1":"0"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;	   		   	
     			break;
     		case RELAY4_CHILD_ID:

     		   bRelay4State = message.getBool();
     		   switchRelayON_OFF( RELAY4_PIN, message.getBool()?RELAY_ON:RELAY_OFF );
 
      		   			//Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgRelay4Status.set(message.getBool()?"1":"0"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;	    		   	
     			break;   
       		case RELAY5_CHILD_ID:

     		   bRelay5State = message.getBool();
     		   switchRelayON_OFF( RELAY5_PIN, message.getBool()?RELAY_ON:RELAY_OFF );

     		   			//Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgRelay5Status.set(message.getBool()?"1":"0"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;	

     		   	
     			break;      			  		     		
     	}
     
     }

    if ( message.sensor == LOCAL_SWITCHING_CHILD_ID  && strlen(message.getString())>0) {
         
         
         localSwitching = message.getBool()?false:true;

     		   			//Отсылаем состояние переключателя
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgLocalSwitching.set(localSwitching?"0":"1"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;	

         

     }

    if ( message.sensor == RECHECK_SENSOR_VALUES && strlen(message.getString())>0) {
         
         if (message.getBool() == true)
         {
            boolRecheckSensorValues = true;


         }

     }

        return;      
} 


void chechButton1 ()
{

  byte switchState = pe.digitalRead (BUTTON1_PIN);
  
  if (switchState != oldSwitch1State)
    {

    // debounce
    if (millis () - switch1PressTime >= debounceTime)
       {
       //switchTime = switchPressTime;

       if (switchState == LOW)
          {

		    //Отсылаем нажатие кнопки с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
      
    	            sensor_node.send(msgButton1.set("1"), true);
                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;

            if ( localSwitching )  
            {
			    if ( !bRelay1State ) //msgRelay1
			    {

			    	bRelay1State = true;
			    	switchRelayON_OFF( RELAY1_PIN, RELAY_ON );

					    //Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgRelay1Status.set("1"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;			    	

			    }	
			    else
			    {

			    	bRelay1State = false;
			    	switchRelayON_OFF( RELAY1_PIN, RELAY_OFF );       

					    //Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgRelay1Status.set("0"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;	
			    	//send status message		    	  			    	
			    }
			}

		  #ifdef NDEBUG	
          Serial.println ("Switch closed.");
          #endif

          }  // end if switchState is LOW
        else
          {

          #ifdef NDEBUG
          Serial.print ("Switch press time: ");
          Serial.println (millis () - switch1PressTime);   
          Serial.println ("Switch opened.");
          #endif 

          bRelay1DelayMessageSent = false;	

		    //Отсылаем нажатие кнопки с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
      
    	            sensor_node.send(msgButton1.set("0"), true);
                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;          

                       
          }  // end if switchState is HIGH

       switch1PressTime = millis ();  // when we closed the switch 
       oldSwitch1State =  switchState;  // remember for next time 

           
       }  // end if debounce time up
        
    }  // end of state change
    else
    {
    	if (switchState == LOW && !bRelay1DelayMessageSent)
    	{
          if (millis () - switch1PressTime >=2000)
          {
          	if ( bRelay1State )
          	{
   
			}
			else
			{
				//send delayed status message 				
			}

		    //Отсылаем нажатие кнопки с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
    	             // Send in the new temperature                  
    	            sensor_node.send(msgRelay1Delayed.set(bRelay1State?"1":"0"), true);
                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;  
			bRelay1DelayMessageSent = true;
          }

    	}

    }

}


void chechButton2 ()
{

  byte switchState = pe.digitalRead (BUTTON2_PIN);
  
  if (switchState != oldSwitch2State)
    {

    // debounce
    if (millis () - switch2PressTime >= debounceTime)
       {
       //switchTime = switchPressTime;

       if (switchState == LOW)
          {

		    //Отсылаем нажатие кнопки с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
      
    	            sensor_node.send(msgButton2.set("1"), true);
                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;

            if ( localSwitching )  
            {
			    if ( !bRelay2State )
			    {

			    	bRelay2State = true;
			    	switchRelayON_OFF( RELAY2_PIN, RELAY_ON );

					    //Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgRelay2Status.set("1"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;			    	

			    }	
			    else
			    {

			    	bRelay2State = false;
			    	switchRelayON_OFF( RELAY2_PIN, RELAY_OFF );       

					    //Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgRelay2Status.set("0"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;			    	

			    	//send status message		    	  			    	
			    }
			}

		  #ifdef NDEBUG	
          Serial.println ("Switch closed.");
          #endif

          }  // end if switchState is LOW
        else
          {

          #ifdef NDEBUG
          Serial.print ("Switch press time: ");
          Serial.println (millis () - switch2PressTime);   
          Serial.println ("Switch opened.");
          #endif 

          bRelay2DelayMessageSent = false;	

		    //Отсылаем нажатие кнопки с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
      
    	            sensor_node.send(msgButton2.set("0"), true);
                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;          

                       
          }  // end if switchState is HIGH

       switch2PressTime = millis ();  // when we closed the switch 
       oldSwitch2State =  switchState;  // remember for next time 

           
       }  // end if debounce time up
        
    }  // end of state change
    else
    {
    	if (switchState == LOW && !bRelay2DelayMessageSent)
    	{
          if (millis () - switch2PressTime >=2000)
          {
          	if ( bRelay2State )
          	{
   
			}
			else
			{
				//send delayed status message 				
			}

		    //Отсылаем нажатие кнопки с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
    	             // Send in the new temperature                  
    	            sensor_node.send(msgRelay2Delayed.set(bRelay2State?"1":"0"), true);
                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;  
			bRelay2DelayMessageSent = true;
          }

    	}

    }

}


void chechButton3 ()
{

  byte switchState = pe.digitalRead (BUTTON3_PIN);
  
  if (switchState != oldSwitch3State)
    {

    // debounce
    if (millis () - switch3PressTime >= debounceTime)
       {
       //switchTime = switchPressTime;

       if (switchState == LOW)
          {

		    //Отсылаем нажатие кнопки с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
      
    	            sensor_node.send(msgButton3.set("1"), true);
                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;

            if ( localSwitching )  
            {
			    if ( !bRelay3State )
			    {

			    	bRelay3State = true;
			    	switchRelayON_OFF( RELAY3_PIN, RELAY_ON );

					    //Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgRelay3Status.set("1"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;			    	
			    }	
			    else
			    {

			    	bRelay3State = false;
			    	switchRelayON_OFF( RELAY3_PIN, RELAY_OFF );       

					    //Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgRelay3Status.set("0"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;			    	

			    	//send status message		    	  			    	
			    }
			}

		  #ifdef NDEBUG	
          Serial.println ("Switch closed.");
          #endif

          }  // end if switchState is LOW
        else
          {

          #ifdef NDEBUG
          Serial.print ("Switch press time: ");
          Serial.println (millis () - switch2PressTime);   
          Serial.println ("Switch opened.");
          #endif 

          bRelay3DelayMessageSent = false;	

		    //Отсылаем нажатие кнопки с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
      
    	            sensor_node.send(msgButton3.set("0"), true);
                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;          

                       
          }  // end if switchState is HIGH

       switch3PressTime = millis ();  // when we closed the switch 
       oldSwitch3State =  switchState;  // remember for next time 

           
       }  // end if debounce time up
        
    }  // end of state change
    else
    {
    	if (switchState == LOW && !bRelay3DelayMessageSent)
    	{
          if (millis () - switch3PressTime >=2000)
          {
          	if ( bRelay3State )
          	{
   
			}
			else
			{
				//send delayed status message 				
			}

		    //Отсылаем нажатие кнопки с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
    	             // Send in the new temperature                  
    	            sensor_node.send(msgRelay3Delayed.set(bRelay3State?"1":"0"), true);
                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;  
			bRelay3DelayMessageSent = true;
          }

    	}

    }

}



void chechButton4 ()
{

  byte switchState = pe.digitalRead (BUTTON4_PIN);
  
  if (switchState != oldSwitch4State)
    {

    // debounce
    if (millis () - switch4PressTime >= debounceTime)
       {
       //switchTime = switchPressTime;

       if (switchState == LOW)
          {

		    //Отсылаем нажатие кнопки с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
      
    	            sensor_node.send(msgButton4.set("1"), true);
                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;

            if ( localSwitching )  
            {
			    if ( !bRelay4State )
			    {

			    	bRelay4State = true;
			    	switchRelayON_OFF( RELAY4_PIN, RELAY_ON );

					    //Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgRelay4Status.set("1"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;			    	

			    }	
			    else
			    {

			    	bRelay4State = false;
			    	switchRelayON_OFF( RELAY4_PIN, RELAY_OFF );       

					    //Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgRelay4Status.set("0"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;			    	

			    	//send status message		    	  			    	
			    }
			}

		  #ifdef NDEBUG	
          Serial.println ("Switch closed.");
          #endif

          }  // end if switchState is LOW
        else
          {

          #ifdef NDEBUG
          Serial.print ("Switch press time: ");
          Serial.println (millis () - switch2PressTime);   
          Serial.println ("Switch opened.");
          #endif 

          bRelay4DelayMessageSent = false;	

		    //Отсылаем нажатие кнопки с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
      
    	            sensor_node.send(msgButton4.set("0"), true);
                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;          

                       
          }  // end if switchState is HIGH

       switch4PressTime = millis ();  // when we closed the switch 
       oldSwitch4State =  switchState;  // remember for next time 

           
       }  // end if debounce time up
        
    }  // end of state change
    else
    {
    	if (switchState == LOW && !bRelay4DelayMessageSent)
    	{
          if (millis () - switch4PressTime >=2000)
          {
          	if ( bRelay4State )
          	{
   
			}
			else
			{
				//send delayed status message 				
			}

		    //Отсылаем нажатие кнопки с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
    	             // Send in the new temperature                  
    	            sensor_node.send(msgRelay4Delayed.set(bRelay4State?"1":"0"), true);
                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;  
			bRelay4DelayMessageSent = true;
          }

    	}

    }

}



void chechButton5 ()
{

  byte switchState = pe.digitalRead (BUTTON5_PIN);
  
  if (switchState != oldSwitch5State)
    {

    // debounce
    if (millis () - switch5PressTime >= debounceTime)
       {
       //switchTime = switchPressTime;

       if (switchState == LOW)
          {

		    //Отсылаем нажатие кнопки с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
      
    	            sensor_node.send(msgButton5.set("1"), true);
                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;

            if ( localSwitching )  
            {
			    if ( !bRelay5State )
			    {

			    	bRelay5State = true;
			    	switchRelayON_OFF( RELAY5_PIN, RELAY_ON );

					    //Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgRelay5Status.set("1"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;			    	
			    }	
			    else
			    {

			    	bRelay5State = false;
			    	switchRelayON_OFF( RELAY5_PIN, RELAY_OFF );       

					    //Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgRelay5Status.set("0"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;			    	

			    	//send status message		    	  			    	
			    }
			}

		  #ifdef NDEBUG	
          Serial.println ("Switch closed.");
          #endif

          }  // end if switchState is LOW
        else
          {

          #ifdef NDEBUG
          Serial.print ("Switch press time: ");
          Serial.println (millis () - switch2PressTime);   
          Serial.println ("Switch opened.");
          #endif 

          bRelay5DelayMessageSent = false;	

		    //Отсылаем нажатие кнопки с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
      
    	            sensor_node.send(msgButton5.set("0"), true);
                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;          

                       
          }  // end if switchState is HIGH

       switch5PressTime = millis ();  // when we closed the switch 
       oldSwitch5State =  switchState;  // remember for next time 

           
       }  // end if debounce time up
        
    }  // end of state change
    else
    {
    	if (switchState == LOW && !bRelay5DelayMessageSent)
    	{
          if (millis () - switch5PressTime >=2000)
          {
          	if ( bRelay5State )
          	{
   
			}
			else
			{
				//send delayed status message 				
			}

		    //Отсылаем нажатие кнопки с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
    	             // Send in the new temperature                  
    	            sensor_node.send(msgRelay5Delayed.set(bRelay5State?"1":"0"), true);
                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;  
			bRelay5DelayMessageSent = true;
          }

    	}

    }

}


void resendRelayStatus()
{

   			//Отсылаем состояние реле с подтверждением получения
        iCount = MESSAGE_ACK_RETRY_COUNT;

          while( !gotAck && iCount > 0 )
            {
  
	            sensor_node.send(msgRelay1Status.set(bRelay1State?"1":"0"), true);
                sensor_node.wait(RADIO_RESET_DELAY_TIME);
              iCount--;
             }

            gotAck = false;	


   	
   			//Отсылаем состояние реле с подтверждением получения
        iCount = MESSAGE_ACK_RETRY_COUNT;

          while( !gotAck && iCount > 0 )
            {
  
	            sensor_node.send(msgRelay2Status.set(bRelay2State?"1":"0"), true);
                sensor_node.wait(RADIO_RESET_DELAY_TIME);
              iCount--;
             }

            gotAck = false;	


   			//Отсылаем состояние реле с подтверждением получения
        iCount = MESSAGE_ACK_RETRY_COUNT;

          while( !gotAck && iCount > 0 )
            {
  
	            sensor_node.send(msgRelay3Status.set(bRelay3State?"1":"0"), true);
                sensor_node.wait(RADIO_RESET_DELAY_TIME);
              iCount--;
             }

            gotAck = false;	   		   	


   			//Отсылаем состояние реле с подтверждением получения
        iCount = MESSAGE_ACK_RETRY_COUNT;

          while( !gotAck && iCount > 0 )
            {
  
	            sensor_node.send(msgRelay4Status.set(bRelay4State?"1":"0"), true);
                sensor_node.wait(RADIO_RESET_DELAY_TIME);
              iCount--;
             }

            gotAck = false;	    		   	


   			//Отсылаем состояние реле с подтверждением получения
        iCount = MESSAGE_ACK_RETRY_COUNT;

          while( !gotAck && iCount > 0 )
            {
  
	            sensor_node.send(msgRelay5Status.set(bRelay5State?"1":"0"), true);
                sensor_node.wait(RADIO_RESET_DELAY_TIME);
              iCount--;
             }

            gotAck = false;	
   			  		     		


}


void checkTemperature()
{


    unsigned long currentTempMillis = millis();
    if((currentTempMillis - previousTempMillis ) > TEMPCHECK_TIME ) {
        // Save the current millis
        previousTempMillis = currentTempMillis;

// Fetch temperatures from Dallas sensors
  sensors.requestTemperatures();

  // query conversion time and sleep until conversion completed
  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
  // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
  sensor_node.wait(conversionTime);

 float temperature = static_cast<float>(static_cast<int>(sensors.getTempCByIndex(0) * 10.)) / 10.;




         if (temperature != lastTemp1 && temperature != -127.00 && temperature != 85.00 ) {

          		#ifdef NDEBUG                
                Serial.print ("Temp: ");
          	    Serial.println (temperature); 
          	    #endif

     		   			//Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgTemp1.set(temperature,2), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;	     


			                if ( temperature >= 60 )
			                {
			                	//switch off all relays
     		  					 bRelay2State = false;
     		   					 switchRelayON_OFF( RELAY2_PIN, RELAY_OFF );			                	
     		  					 bRelay3State = false;
     		   					 switchRelayON_OFF( RELAY3_PIN, RELAY_OFF );
     		  					 bRelay4State = false;
     		   					 switchRelayON_OFF( RELAY4_PIN, RELAY_OFF );
     		  					 bRelay5State = false;
     		   					 switchRelayON_OFF( RELAY5_PIN, RELAY_OFF );   
     		   					 resendRelayStatus();  		   					      		   					 
			                }     		

            lastTemp1 = temperature;
        	} 

 temperature = static_cast<float>(static_cast<int>(sensors.getTempCByIndex(1) * 10.)) / 10.;




         if (temperature != lastTemp2 && temperature != -127.00 && temperature != 85.00 ) {
                
          		#ifdef NDEBUG
                Serial.print ("Temp2: ");
          	    Serial.println (temperature); 
          	    #endif

     		   			//Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgTemp2.set(temperature,2), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;	          		

			                if ( temperature >= 60 )
			                {
			                	//switch off all relays
     		  					 bRelay1State = false;
     		   					 switchRelayON_OFF( RELAY1_PIN, RELAY_OFF );	
     		   					 resendRelayStatus();			                	
			                }     	

            lastTemp2 = temperature;
        	} 



      }
}
