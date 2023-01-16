#include"topicMqtt.h";

void mqttloop(){
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
}

void callback(char* topic,byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  String msg;
  for (int i = 0; i < length; i++) {
    msg = msg + (char)payload[i];
  }
   Serial.println(msg);
  if (String(topic) == subscribe_topic) { 
    if (msg == "1"){
      //digitalWrite(ledPin, LOW);
      Serial.println("Turn on LED");
    } else {
      //digitalWrite(ledPin, HIGH);
      Serial.println("Turn off LED");
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection..");
    if (client.connect(mqtt_client, mqtt_username, mqtt_password)) {
      Serial.println("Connected");
      client.subscribe(subscribe_topic);
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("Try again in 5 seconds...");
      delay(5000);
    }
  }
}
