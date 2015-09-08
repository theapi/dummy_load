// Graphing sketch
// Takes ASCII-encoded strings

// Serial port to connect to
String serialPortName = "/dev/ttyUSB0";
 

 
import processing.serial.*;
 
Serial myPort;        // The serial port
int xPos = 1;         // horizontal position of the graph
float[] previousVals; 
 
void setup () {
 // set the window size:
 size(640, 480);        
 
 // List all the available serial ports
 println(Serial.list());
 
 // Open the port
 myPort = new Serial(this, serialPortName, 115200);
 // don't generate a serialEvent() unless you get a newline character:
 myPort.bufferUntil('\n');
 // set inital background:
 background(255);
 
 previousVals = new float[7];
 for (int i = 0; i < 7; i++) {
   previousVals[i] = 0.1; 
 }
}
 
void draw () {
 // everything happens in the serialEvent()
}
 
void serialEvent (Serial myPort) {
   // get the ASCII string:
   String inString = myPort.readStringUntil('\n');
   println(inString);

   if (inString != null) {
     // split the string at delimiter (space)
     String[] nums = split(inString, ' ');
     
     for (int i = 0; i < 7; i++) {
       
       // trim off any whitespace:
       inString = trim(nums[i]);
       //print(inString);
       //print(":");
       // convert to a float and map to the screen height:
       float inByte = new Float(inString); 
       print(inByte);
       print(" ");
       inByte = map(inByte, 0, 128, 0, height);
       
       // draw the line:
       stroke(127,34,255);
       strokeWeight(2);
       //print(inByte);
       //print(" ");
       if (previousVals[i] == 0.1) {
         point(xPos, height - inByte);
       } else {
         line(xPos, previousVals[i], xPos, height - inByte);
         //line(xPos, height, xPos, height - inByte);
       }
  
       previousVals[i] = height - inByte;
     }
     println("");
     // at the edge of the screen, go back to the beginning:
     if (xPos >= width) {
       xPos = 0;
       background(255); 
     } 
     else {
       // increment the horizontal position:
       xPos++;
     }
   }
}
 
