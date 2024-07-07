#include <Timezone.h>

#include <TimeLib.h>

#include <NTPClient.h>

#include <WiFiUdp.h>
#include <ESP8266Firebase.h>
//#include <FirebaseESP8266.h>
#include <ESP8266WiFi.h>


#define FIREBASE_HOST "esp8266-4c31a-default-tdb.firebase.com"

Firebase firebase(FIREBASE_HOST);
char buff [20];
volatile byte indx;
String receivedString;
int nextId =1;
String currentDate;
String currentTime;
String STATE;


WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "time.google.com");
// Định nghĩa múi giờ cho Việt Nam
TimeChangeRule myDST = {"DST", Last, Sun, Mar, 2, 420};    // Múi giờ hè (DST) kích hoạt vào Chủ Nhật cuối cùng của tháng 3 lúc 2:00 AM
TimeChangeRule mySTD = {"STD", Last, Sun, Oct, 2, 360};    // Múi giờ tiêu chuẩn (STD) kích hoạt vào Chủ Nhật cuối cùng của tháng 10 lúc 2:00 AM
Timezone myTZ(myDST, mySTD);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ketnoiwifi();
  timeClient.begin();
  setTime(20, 25, 56, 5, 6, 2024); // Đặt thời gian là 20:20:56, Thứ Tư, ngày 5 tháng 6 năm 2024

  
  //Serial.println("sdsadsadas");
}

void loop() {
 
  timeClient.update();
  time_t utcTime = timeClient.getEpochTime();
  // Chuyển múi giờ sang múi giờ của Việt Nam
  time_t localTime = myTZ.toLocal(utcTime);
  String currentDate =  getDate(localTime);
  String currentTime = getTime(localTime);
  firebase.setString("Example/setString",currentDate); 
  firebase.setString("Example/Time",currentTime);
  
    if (Serial.available() > 0) {
      
      if (receivedString.isEmpty()){
        receivedString = Serial.readString(); // Đọc chuỗi từ Serial
       // firebase.setString("data1",receivedString);
        int index = receivedString.indexOf(":") + 1;
        String valueSensor = receivedString.substring(index);
        float soilValue = valueSensor.toFloat();
        if(soilValue >60) STATE ="ON";
        else if(soilValue <=60) STATE ="OFF";
        currentDate =  getDate(localTime);
        currentTime = getTime(localTime);
        String nextidString = String(nextId);
        String dataPath = "data/" + nextidString;
        firebase.setFloat("/" + dataPath + "/soilValue", soilValue);
        firebase.setString("/" + dataPath + "/Date", currentDate);
        firebase.setString("/" + dataPath + "/Time", currentTime);
        firebase.setString("/" + dataPath + "/STATE", STATE);
        firebase.json(true);

        nextId++;
      }
      else{
        receivedString = "";
      }
     
  }
   }
    // Kiểm tra nếu dữ liệu hợp lệ
    


void ketnoiwifi(){
  Serial.println("bat dau ket noi wifi");
  WiFi.begin("your wifi","password");
  Serial.println(WiFi.localIP());
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("...");
    delay(500);
  }
  Serial.println("");
  Serial.println("ket noi thanh cong");
  Serial.println(WiFi.localIP());

}

// Hàm trả về ngày tháng năm từ thời gian 

String getDate(time_t timestamp) {/-strong/-heart:>:o:-((:-hint dayOfMonth = day(timestamp);
  int monthOfYear = month(timestamp);
  int yearNumber = year(timestamp);

  return String(dayOfMonth) + "/" + String(monthOfYear) + "/" + String(yearNumber);
}

String getTime(time_t timestamp) {
  int hourOfDay = hour(timestamp);
  int minuteOfHour = minute(timestamp);
  int secondOfMinute = second(timestamp);

  return String(hourOfDay) + ":" + String(minuteOfHour) + ":" + String(secondOfMinute);
}