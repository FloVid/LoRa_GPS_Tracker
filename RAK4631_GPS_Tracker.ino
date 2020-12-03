/**
 * Tracking LoRa tag system OTAA 
 * FLOVID 2020, flovid.net
 * Credits: Bernd Giesecke, Antonio Valente
 */
#include <Arduino.h>
#include <LoRaWan-RAK4630.h>   //http://librarymanager/All#SX126x
#include <SPI.h>

//GPS variables
void read_GPS_data(void);
bool hasGPSdata = false;
bool newpos = false;
unsigned char loraData[8];
int32_t newlat = 0;
int32_t newlong = 0;
//float VBAT = 3; //Oscar average max voltaje supply for a LiPo battery 3.7 V, play with this value to get a correct one in ADC

// RAK4630 supply two LED
#ifndef LED_BLUE //Oscar Blue LED
#define LED_BLUE 35
#endif

#ifndef LED_GREEN  //Oscar Green LED
#define LED_GREEN 36
#endif

bool doOTAA = true;
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60	/**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATARATE DR_0	/*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_5	/*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 3	/**< Number of trials for the join request. */
DeviceClass_t gCurrentClass = CLASS_A;	/* class definition*/
lmh_confirm gCurrentConfirm = LMH_CONFIRMED_MSG;	/* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;	/* data port*/

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
 */
static lmh_param_t lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DATARATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
										lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler};

//OTAA keys !!!! KEYS ARE MSB !!!!
//OTAA keys !!!! KEYS ARE MSB !!!!
uint8_t nodeDeviceEUI[8] = {FILL WITH YOUR KEY};
uint8_t nodeAppEUI[8] = {FILL WITH YOUR KEY}; //Oscar if Chisrpstack is used, fill with 0x00, 0x00, 0x00.....,no needed for autentication, TTN same for all nodes into the same application                                                                   
uint8_t nodeAppKey[16] = {FILL WITH YOUR KEY};


// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE 64	/**< buffer size of the data to be transmitted. */
#define LORAWAN_APP_INTERVAL 20000	/**< Defines for user timer, the application data transmission interval. 20s, value in [ms]. */
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];	//< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.

TimerEvent_t appTimer;
static uint32_t timers_init(void);
static uint32_t count = 0;
static uint32_t count_fail = 0;

void setup()
{
	pinMode(LED_BLUE, OUTPUT);
	digitalWrite(LED_BLUE, LOW);
  pinMode(LED_GREEN, OUTPUT);
	digitalWrite(LED_GREEN, LOW);

	// Initialize LoRa chip.
	lora_rak4630_init();

	// Initialize Serial for debug output
	Serial.begin(115200);
	/* while (!Serial)
	{
		delay(10);
	} */
	Serial.println("=====================================");
	Serial.println("Welcome to RAK4630 LoRaWan!!!");
	Serial.println("Type: OTAA");

  #if defined(REGION_AS923)
    Serial.println("Region: AS923");
  #elif defined(REGION_AU915)
    Serial.println("Region: AU915");
  #elif defined(REGION_CN470)
    Serial.println("Region: CN470");
  #elif defined(REGION_CN779)
    Serial.println("Region: CN779");
  #elif defined(REGION_EU433)
    Serial.println("Region: EU433");
  #elif defined(REGION_IN865)
    Serial.println("Region: IN865");
  #elif defined(REGION_EU868)
    Serial.println("Region: EU868");
  #elif defined(REGION_KR920)
    Serial.println("Region: KR920");
  #elif defined(REGION_US915)
    Serial.println("Region: US915");
  #elif defined(REGION_US915_HYBRID)
    Serial.println("Region: US915_HYBRID");
  #else
    Serial.println("Please define a region in the compiler options.");
  #endif
    Serial.println("=====================================");

  //gps init
  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);
  pinMode(34, OUTPUT);
  digitalWrite(34, 0);
  delay(1000);
  digitalWrite(34, 1);
  delay(1000);
  Serial1.begin(9600);
  while (!Serial1);
  digitalWrite(LED_BLUE, HIGH);
  delay(500);
  digitalWrite(LED_BLUE, LOW);
  delay(500);
  digitalWrite(LED_BLUE, HIGH);
  delay(500);
  digitalWrite(LED_BLUE, LOW);
  delay(500);
  digitalWrite(LED_BLUE, HIGH);
  delay(500);
  digitalWrite(LED_BLUE, LOW);
  delay(500);
  Serial.println();
  Serial.println("GPS uart init ok!");
  Serial.println();

  // Set the analog reference to 3.0V (default = 3.6V)
	analogReference(AR_INTERNAL_3_0);

	// Set the resolution to 12-bit (0..4096)
	analogReadResolution(12); // Can be 8, 10, 12 or 14

	// Let the ADC settle
	delay(1);

	//creat a user timer to send data to server period
	uint32_t err_code;
	err_code = timers_init();
	if (err_code != 0)
	{
		Serial.printf("timers_init failed - %d\n", err_code);
	}

	// Setup the EUIs and Keys
	lmh_setDevEui(nodeDeviceEUI);
	lmh_setAppEui(nodeAppEUI);
	lmh_setAppKey(nodeAppKey);

	// Initialize LoRaWan
	err_code = lmh_init(&lora_callbacks, lora_param_init, doOTAA);
	if (err_code != 0)
	{
		Serial.printf("lmh_init failed - %d\n", err_code);
	}

	// Start Join procedure
	lmh_join();
}

void loop()
{
	// Handle Radio events
	Radio.IrqProcess();
}

/**@brief LoRa function for handling HasJoined event.
 */
void lorawan_has_joined_handler(void)
{
	Serial.println("OTAA Mode, Network Joined!");
  digitalWrite(LED_GREEN, HIGH);
  delay(500);
  digitalWrite(LED_GREEN, LOW);
  delay(500);
  digitalWrite(LED_GREEN, HIGH);
  delay(500);
  digitalWrite(LED_GREEN, LOW);
  delay(500);
  digitalWrite(LED_GREEN, HIGH);
  delay(500);
  digitalWrite(LED_GREEN, LOW);

	lmh_error_status ret = lmh_class_request(gCurrentClass);
	if (ret == LMH_SUCCESS)
	{
		delay(1000);
		TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
		TimerStart(&appTimer);
	}
}

/**@brief Function for handling LoRaWan received data from Gateway
 *
 * @param[in] app_data  Pointer to rx data
 */
void lorawan_rx_handler(lmh_app_data_t *app_data)
{
	Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
				  app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
	Serial.printf("switch to class %c done\n", "ABC"[Class]);
	// Informs the server that switch has occurred ASAP
	m_lora_app_data.buffsize = 0;
	m_lora_app_data.port = gAppPort;
	lmh_send(&m_lora_app_data, gCurrentConfirm);
}

void send_lora_frame(void)
{
	if (lmh_join_status_get() != LMH_SET)
	{
		//Not joined, try again later
		return;
	}

  read_GPS_data();
  if(hasGPSdata && newpos)
  {
    uint32_t i = 0;
    memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
    m_lora_app_data.port = gAppPort;
    m_lora_app_data.buffer[i++] = loraData[0];
    m_lora_app_data.buffer[i++] = loraData[1];
    m_lora_app_data.buffer[i++] = loraData[2];
    m_lora_app_data.buffer[i++] = loraData[3];
    m_lora_app_data.buffer[i++] = loraData[4];
    m_lora_app_data.buffer[i++] = loraData[5];
    m_lora_app_data.buffer[i++] = loraData[6];
    m_lora_app_data.buffer[i++] = loraData[7];
    m_lora_app_data.buffsize = i;

    lmh_error_status error = lmh_send(&m_lora_app_data, gCurrentConfirm);

    if (error == LMH_SUCCESS)
    {
      count++;
      Serial.printf("lmh_send ok count %d\n", count);
    }
    else
    {
      count_fail++;
      Serial.printf("lmh_send fail count %d\n", count_fail);
    }
  }
  else
  {
    delay(1000);
    TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
    TimerStart(&appTimer);
  }

}

/**@brief Function for handling user timerout event.
 */
void tx_lora_periodic_handler(void)
{
	TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
	TimerStart(&appTimer);
	Serial.println("Preparing for send now...");
	send_lora_frame();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
uint32_t timers_init(void)
{
	TimerInit(&appTimer, tx_lora_periodic_handler);
	return 0;
}

void read_GPS_data()
{
  uint32_t j = 0;
  String gpsdata = "";
  String gcavalues[15];
  /* float bat = analogRead(A0)*VBAT*0.000439453125;  //Oscar 4.2V max charge LiPo battery = max voltage to supply node = VBAT, ADC_BAT = VBAT*(1.5M/(1Mhom+1.5Mhom)) = BVAT*0.6 = 4.2V*0.6 = 2.52V, 3000mV ADC range at 12-bit ADC resolution = 3000/4096 = 0.732421875 mV = 0.000732421875 V, REAL_VBAT_MV_PER_LSB factor = 2.52V*0.000732421875 V = 0.001845703125
  bat *= 10;  //Oscar to round to send battery value */

  float bat = analogRead(A0);
  bat *= 1.67;	// we divided by 3/5 VBAT (0.6, or voltaje divider) so multiply back
  bat *= 3;	// Multiply by 3V, our reference voltage
  bat /= 4096;	// convert to voltage
  bat *= 10;	//Oscar to round to send battery value

  // see if GPS has valid data
  uint8_t pos[64];
  gpsdata.reserve(64);
  time_t timeout = millis();
  while((gpsdata.indexOf("GGA") == -1) && ((millis() - timeout) < 5000))
  {
      gpsdata = Serial1.readStringUntil(0x0A);
  }
  for(uint8_t k = 0; k < gpsdata.length(); k++) {
      if(gpsdata[k] == ',') 
      {
          pos[j] = k;
          j++;
      }
  }
  pos[j] = gpsdata.length();
  for(uint8_t k = 0; k < j; k++) 
  {
          gcavalues[k]=gpsdata.substring(pos[k]+1, pos[k+1]);
  }
  int numbersats = gcavalues[6].toInt();
  int fixq = gcavalues[5].toInt();
  char latChar = (char)gcavalues[2].charAt(0);  //Oscar N or S
  char longChar = (char)gcavalues[4].charAt(0); //Oscar E or W
  Serial.print("[GPS] Number of sat: ");
  Serial.print(numbersats);
  Serial.print(", [GPS] fix quality: ");
  Serial.println(fixq);

  if (gcavalues[2].length() > 0 && gcavalues[4].length() > 0) //Oscar there are lat and long
  { 
    /*  */ 
    float degrees = (uint16_t)(gcavalues[1].toFloat() / 100.0);
    float minutes = (gcavalues[1].substring(gcavalues[1].length()-8)).toFloat();
    int32_t latitude = (degrees + minutes / 60.0) * 10000;
    float degrees2 = (uint16_t)(gcavalues[3].toFloat() / 100.0); 
    float minutes2 = (gcavalues[3].substring(gcavalues[3].length()-8)).toFloat();  
    int32_t longitude = (degrees2 + minutes2 / 60.0) * 10000;
    if(latChar == 'S')
      latitude = latitude * -1;
    
    if(longChar == 'W')
      longitude = longitude * -1;

    if(newlat != latitude and newlong != longitude)
    {
      newlat = latitude;
      newlong = longitude;
      newpos = true;
    }
    else
    {
      newpos = false;
      return;
    }
     

    loraData[0] = bat;
             
    loraData[1] = latitude >> 16;
    loraData[2] = latitude >> 8;
    loraData[3] = latitude;

    loraData[4] = longitude >> 24;
    loraData[5] = longitude >> 16;
    loraData[6] = longitude >> 8;
    loraData[7] = longitude;
    hasGPSdata = true;
    return;
  } 
  else
  {
    Serial.println("[GPS] Waiting for valid data...");
    gpsdata = "";
    hasGPSdata = false;
    digitalWrite(LED_BLUE, HIGH);
    delay(250);
    digitalWrite(LED_BLUE, LOW);
    delay(250);
    digitalWrite(LED_BLUE, HIGH);
    delay(250);
    digitalWrite(LED_BLUE, LOW);
    delay(250);
    digitalWrite(LED_BLUE, HIGH);
    delay(250);
    digitalWrite(LED_BLUE, LOW);
    delay(250);
    digitalWrite(LED_BLUE, HIGH);
    delay(250);
    digitalWrite(LED_BLUE, LOW);
    delay(250);
    digitalWrite(LED_BLUE, HIGH);
    delay(250);
    digitalWrite(LED_BLUE, LOW);
    delay(250);
    digitalWrite(LED_BLUE, HIGH);
    delay(250);
    digitalWrite(LED_BLUE, LOW);
    return;
  }

  gpsdata = "";
}
