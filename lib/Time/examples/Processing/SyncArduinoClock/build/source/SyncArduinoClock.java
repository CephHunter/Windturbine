import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import processing.serial.*; 
import java.util.Date; 
import java.util.Calendar; 
import java.util.GregorianCalendar; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class SyncArduinoClock extends PApplet {

/**
 * SyncArduinoClock.
 *
 * portIndex must be set to the port connected to the Arduino
 *
 * The current time is sent in response to request message from Arduino
 * or by clicking the display window
 *
 * The time message is 11 ASCII text characters; a header (the letter 'T')
 * followed by the ten digit system time (unix time)
 */







public static final short portIndex = 0;  // select the com port, 0 is the first port
public static final String TIME_HEADER = "T"; //header for arduino serial time message
public static final char TIME_REQUEST = 7;  // ASCII bell character
public static final char LF = 10;     // ASCII linefeed
public static final char CR = 13;     // ASCII linefeed
Serial myPort;     // Create object from Serial class

public void setup() {
  
  println(Serial.list());
  println(" Connecting to -> " + Serial.list()[portIndex]);
  myPort = new Serial(this,Serial.list()[portIndex], 9600);
  println(getTimeNow());
}

public void draw()
{
  textSize(20);
  textAlign(CENTER);
  fill(0);
  text("Click to send\nTime Sync", 0, 75, 200, 175);
  if ( myPort.available() > 0) {  // If data is available,
    char val = PApplet.parseChar(myPort.read());         // read it and store it in val
    if(val == TIME_REQUEST){
       long t = getTimeNow();
       sendTimeMessage(TIME_HEADER, t);
    }
    else
    {
       if(val == LF)
           ; //igonore
       else if(val == CR)
         println();
       else
         print(val); // echo everying but time request
    }
  }
}

public void mousePressed() {
  sendTimeMessage( TIME_HEADER, getTimeNow());
}


public void sendTimeMessage(String header, long time) {
  String timeStr = String.valueOf(time);
  myPort.write(header);  // send header and time to arduino
  myPort.write(timeStr);
  myPort.write('\n');
}

public long getTimeNow(){
  // java time is in ms, we want secs
  Date d = new Date();
  Calendar cal = new GregorianCalendar();
  long current = d.getTime()/1000;
  long timezone = cal.get(cal.ZONE_OFFSET)/1000;
  long daylight = cal.get(cal.DST_OFFSET)/1000;
  return current + timezone + daylight;
}
  public void settings() {  size(200, 200); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "SyncArduinoClock" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
