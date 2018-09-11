#include <stdio.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

//---Объявление пинов
#define IN1 8 
#define IN2 7
#define EN1 6


//---Структуры данных
typedef struct { 
  float latitude;
  float longitude;
} coordinate_t;

//---Глобальные переменные
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

TinyGPSPlus gps;
SoftwareSerial ss(3, 4);

coordinate_t coordinates[] = {{59.973362, 30.323406},
                              {59.861815, 30.234037},
                              {59.849877, 30.234844},
                              {59.845775, 30.200382}};

int coordinate_count = sizeof(coordinates)/sizeof(coordinate_t);
float last_latitude = 0;
float last_longitude = 0;
float last_course = 0;
float heading_degrees_current = 0;
float heading_degrees_need = 0;
float heading_degrees_need_gps = 0;
int next_coordinate = 0;
float max_error_latitude = 0.000010;
float max_error_longitude = 0.000010;
float max_error_heading_degrees = 5;

//---Вычисление направления
void calc_direction(float lat_1, float long_1, float lat_2, float long_2)
{
    float x1 = 0;
    float y1 = 0;
    float y2 = 0;

    x1 = sin((-long_2+long_1)*PI/180);
    y1 = cos(lat_2*PI/180) * tan(lat_1*PI/180);
    y2 = sin(lat_2*PI/180) * cos((-long_2+long_1)*PI/180);
    float Result = atan(x1/(y1-y2))*180/PI;

    if(Result < 0)
        Result = 360 + Result;
    if((long_2 < long_1) && (long_2 > (long_1-180)))
        if(Result > 180)
            Result = Result - 180;
    if(Result > 360)
        Result = Result - 360;
    heading_degrees_need = Result;
}

//---Установка в нужном направлении по компасу
void set_direction()
{
  while(true)
  {
    Serial.println("Обновляем показания датчиков");
    update_heading();
    Serial.print("Heading (degrees) curent: "); Serial.println(heading_degrees_current);
    Serial.print("Heading (degrees) need: "); Serial.println(heading_degrees_need);
    update_current_gps();
    Serial.print("last_course: "); Serial.println(last_course);
    Serial.print("last_latitude: "); Serial.println(last_latitude, 6);
    Serial.print("last_longitude: "); Serial.println(last_longitude, 6);
    if(((heading_degrees_current-max_error_heading_degrees) <= heading_degrees_need) && ((heading_degrees_current+max_error_heading_degrees) >= heading_degrees_need))
      break;

    if(heading_degrees_current<heading_degrees_need)
    {
      Serial.println("Требуется повернуть направо");
      set_right();
      motor_on();
    }
    if(heading_degrees_current>heading_degrees_need)
    {
      Serial.println("Требуется повернуть налево");
      set_left();
      motor_on();
    }
    delay(500);
  }
  Serial.println("Направление задано");
  motor_off();
  
}

//---Показать параметры магнитометра
void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

//---Получить текущий угол по компасу
void update_heading()
{
  sensors_event_t event; 
  mag.getEvent(&event);
  
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  float declinationAngle = 0.22;
  heading += declinationAngle;

  if(heading < 0)
    heading += 2*PI;
    
  if(heading > 2*PI)
    heading -= 2*PI;
   
  heading_degrees_current = heading * 180/M_PI; 
  
}

//---Обновить данные GPS
void update_current_gps()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    //gps.f_get_position(&flat, &flon, &age);
    last_latitude = gps.location.lat();
    last_longitude = gps.location.lng();
    last_course = gps.course.deg();
    //Serial.print("LAT=");
    //Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    //Serial.print(" LON=");
    //Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    //Serial.print(" SAT=");
    //Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    //Serial.print(" PREC=");
    //Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }
}

//---Включить мотор
void motor_on()
{
  analogWrite(EN1, 130);
}

//---Выключить мотор
void motor_off()
{
  analogWrite(EN1, 0);
}

//---Установка назад
void set_back()
{
  digitalWrite (IN2, HIGH);
  digitalWrite (IN1, LOW); 
}

//---Установка вперед
void set_forward()
{
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW); 
}

//---Установка влево
void set_left()
{

}

//---Установка вправо
void set_right()
{

}

//---Проверка достигли ли текущей точки
bool check_coordinates()
{
  if(((last_latitude + max_error_latitude) >= coordinates[next_coordinate].latitude) && ((last_latitude - max_error_latitude) <= coordinates[next_coordinate].latitude))
    if(((last_longitude + max_error_longitude) >= coordinates[next_coordinate].longitude) && ((last_longitude - max_error_longitude) <= coordinates[next_coordinate].longitude))
      return true;
    else
      return false;
  else
    return false;
}

void setup()
{
  Serial.begin(115200);
  ss.begin(9600);
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  displaySensorDetails();
  Serial.println("Обнаружено " + String(coordinate_count) + " координат в памяти устройства");
  //---Настройка пинов шилда мотора
  pinMode (EN1, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);

  int value = EEPROM.read(0);
  if(value != 255)
    next_coordinate = value;
  else
    next_coordinate = 0;

  Serial.print(value);
}
void loop()
{
  for(int i=next_coordinate; i < coordinate_count; i++)
  {
    next_coordinate = i;
    while(true)
    {
      //1-обновления показателей
      Serial.println("Обновляем показания датчиков");
      update_heading();
      update_current_gps();
      if(last_latitude == 0 || last_longitude == 0)
      {
        Serial.println("Ждем пока появится сигнал GPS");
        motor_off();
        delay(3000);
        return;
      }
      //calc_direction(coordinates[next_coordinate].latitude, coordinates[next_coordinate].longitude, last_latitude, last_longitude);
      heading_degrees_need = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), coordinates[next_coordinate].latitude, coordinates[next_coordinate].longitude);
      Serial.print("Heading (degrees) curent: "); Serial.println(heading_degrees_current);
      Serial.print("Heading (degrees) need: "); Serial.println(heading_degrees_need);
      Serial.print("Heading (degrees) need(GPS): "); Serial.println(heading_degrees_need_gps);
      Serial.print("last_course: "); Serial.println(last_course);
      Serial.print("last_latitude: "); Serial.println(last_latitude, 6);
      Serial.print("last_longitude: "); Serial.println(last_longitude, 6);
      
      //2-Выставляем направление
      Serial.println("Высталяем Направление");
      if((heading_degrees_current-max_error_heading_degrees) > heading_degrees_need || (heading_degrees_current+max_error_heading_degrees) < heading_degrees_need)
        set_direction();
      //3-Едем прямо
      Serial.println("Едем прямо");
      set_forward();
      motor_on();
      if(check_coordinates())
      {
        Serial.println("Достигли цели");
        EEPROM.write(0, next_coordinate++);
        motor_off();
        break;
      }
      delay(1000);
    }
  }
  EEPROM.write(0, 255);
  delay(5000);
}
