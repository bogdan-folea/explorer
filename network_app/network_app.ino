#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>

#define DEBUG

WiFiUDP udp;

SoftwareSerial swSerial(5, 4, false, 256);

char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
boolean client_exists = false;

char RXBuffer[20];
unsigned int crt = 0;
boolean valid = false;

void setup()
{
    swSerial.begin(9600);
    #ifdef DEBUG
      Serial.begin(115200);
    #endif
    
    WiFi.mode(WIFI_AP_STA);
    IPAddress ip(12, 0, 0, 1);
    IPAddress subnetmask(255, 255, 255, 0);
    WiFi.softAPConfig(ip, ip, subnetmask);
    WiFi.softAP("EXPLORER", "asgard");

    udp.begin(1032);

    char start[6]; int ok = 0;
    do{
        swSerial.readBytesUntil('\0', start, 6);
        String recv = String(start);
        if( recv.equals("START") ){
            ok = 1;
            #ifdef DEBUG
                Serial.println("STARTED\n");
            #endif
        }
    }while( ok == 0 );

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
    /*
     * Linux -> FreeRTOS
     */
    int packetSize = udp.parsePacket();
    if( packetSize > 0 ){
        client_exists = true;
        
        #ifdef DEBUG
            Serial.print("Received packet: ");
        #endif
        
        // Read the packet.
        udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
        //packetBuffer[packetSize] = '\0';

        boolean command_ok = false;
        switch( packetBuffer[packetSize-1] ){
        case 'f': command_ok = true;
                  break;
        case 'b': command_ok = true;
                  break;
        case 'l': command_ok = true;
                  break;
        case 'r': command_ok = true;
                  break;
        default: command_ok = false;
        }
            
        if( command_ok == true ){
            swSerial.write(packetBuffer[packetSize-1]);
    
            #ifdef DEBUG
                Serial.println(packetBuffer);
                
                // Send a reply to the sender IP address and port.
                udp.beginPacket( udp.remoteIP(), udp.remotePort() );
                
                switch( packetBuffer[packetSize-1] ){
                    case 'f': udp.write("fwd");
                              break;
                    case 'b': udp.write("back");
                              break;
                    case 'l': udp.write("left");
                              break;
                    case 'r': udp.write("right");
                              break;
                }
                udp.write(" cmd - ack");
                udp.endPacket();
            #endif
        }
    }
    udp.flush();

    /*
     * FreeRTOS -> Linux
     */
    if( swSerial.available() > 0 ){
        char recv = swSerial.read();

        if( recv == 'S' ){
            valid = true;
            crt = 0;
        } else if( recv == '\0' ){
            RXBuffer[ crt ] = '\0';
            
            /* Forward information to any known client. */
            if( client_exists == true ){
                udp.beginPacket( udp.remoteIP(), udp.remotePort() );
                udp.write( RXBuffer );
                udp.endPacket();
                udp.flush();
            }
            valid = false;
            crt = 0;
        } else if( valid == true ){
            RXBuffer[ crt++ ] = recv;
        }
    }
}
