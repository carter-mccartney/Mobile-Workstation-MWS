//THis should allow basic functionailty of the esp32s to take communication to and from teh user
//Many Pins can be used 15 adn 18 are used currently, can use 25 26 32 35
// 14 12
//below has been tested
//0 4 17 16 25 26 27 18 5

const int trigPin1 = 14;//red
const int echoPin1 = 12;//yellow
long duration;
int distance;
String name = "Ultra One";
void setup() {
  pinMode(trigPin1, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT);   // Sets the echoPin as an Input
  Serial.begin(9600);         // Starts the serial communication
}
void loop() {
  // Clears the trigPin
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "ID") {
      Serial.write("\nUltra One\n");
    } else if (command == "DISTANCE") {
      digitalWrite(trigPin1, LOW);
      //confirm it is low
      delayMicroseconds(2);

      digitalWrite(trigPin1, HIGH);
      //allow a slight delay to get some feedback loop
      delayMicroseconds(10);
      digitalWrite(trigPin1, LOW);

      duration = pulseIn(echoPin1, HIGH);
      // Calculating the distance
      distance = (duration/2)/29.1;
      // Prints the distance on the Serial Monitor

      //Distance would be in CM
      Serial.print("\n");
      Serial.println(distance);
      //should end with newline
    }
  }
}