/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.RobotMap;
import java.sql.Array;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class LEDLights {

    AddressableLEDBuffer m_ledBuffer;
    AddressableLED m_led;

    int m_rainbowFirstPixelHue = 0;
    int setbrightestone = 0;
    int counter = 0;
    int upCounter = 0;
    public static int pattern = 2;

    Timer timer = new Timer();
    
    boolean blinkeven = false;
    boolean greenIsFirst = true;
    boolean blueIsFirst = true;
    boolean greenUp = false;

    public LEDLights() {

      timer.start();
      m_led = RobotMap.led;
    
      // Length is expensive to set, so only set it once, then just update data
      m_ledBuffer = new AddressableLEDBuffer(65);
      m_led.setLength(m_ledBuffer.getLength());
    
      // Set the data
      m_led.setData(m_ledBuffer);
      m_led.start();
    }

    public void run(int pattern) {
        switch(pattern){
          //Disabled / Rainbow
          case 1:
            rainbow();
            break;

          //Teleop / Green and Gold Gradient
          case 2:
            GreenGold();
            break;

          //Autonomous / Green and Gold Blinking 
          case 3:
            up(2);
            break;

          //One ball recieved / Solid purple
          case 4:
            singleColor(255, 0, 255);
            break;

          //Shooter up to speed / Flash blue
          case 5:
            blink(0, 0, 255);
            break;

          //PID drive enabled / Flash red
          case 6:
            blink(255, 0, 0);
            break;

          //High Torque Mode enabled / Solid orange
          case 7:
            singleColor(255, 100, 0);
            break;

          //Shooter up to speed & PID enabled / Flashing blue & red
          case 8:
            blinkMultipleColors(255, 0, 0, 0, 0, 255);
            break;

          //HTM & PID Enabled / Flashing red & orange
          case 9:
            blinkMultipleColors(255, 0, 0, 255, 100, 0);
            break;

          //Auto Climb Active / Yellow moving upward
          case 10:
            autoClimbLEDs(1);
            break;

          //Default / Rainbow
          default:
            rainbow();
      }
    }

    //Displays a single color.
    public void singleColor(int r,int g, int b) {
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, r, g, b);
        }
       m_led.setData(m_ledBuffer);
    }

    //Displays a rainbow pattern that continuously moves down the line.
    public void rainbow() {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
        m_ledBuffer.setHSV(i, hue, 255, 128);
      }
      m_rainbowFirstPixelHue += 3;
      m_rainbowFirstPixelHue %= 180;
      m_led.setData(m_ledBuffer);
    }

    //Displays a pattern of 'crawling' yellow and green lights
    public void up(int speed){
      counter++;
      //Sets the speed that 'greenUp' changes at
      if (counter%speed==0) {

        //Runs if the boolean 'greenUp' is false
        if(!greenUp) {
          m_ledBuffer.setRGB(setbrightestone, 150, 70, 0);
          m_ledBuffer.setRGB(setbrightestone + 2, 100, 97, 0);
          m_ledBuffer.setRGB(setbrightestone + 4, 50, 123, 0);
          m_ledBuffer.setRGB(setbrightestone + 6, 0, 150, 0);
        } 

        //Runs if the boolean 'greenUp' is true
        else if(greenUp) {            
          m_ledBuffer.setRGB(setbrightestone, 0, 150, 0);
          m_ledBuffer.setRGB(setbrightestone + 2, 50, 123, 0);
          m_ledBuffer.setRGB(setbrightestone + 4, 100, 97, 0);
          m_ledBuffer.setRGB(setbrightestone + 6, 150, 70, 0);
        }

        //Sets the brightness
        setbrightestone+=1;
        if(setbrightestone>33)
        setbrightestone=0;
        upCounter++;
        m_led.setData(m_ledBuffer);

        //Changes the 'greenUp' boolean to false/true
        if(upCounter >= 34) {
          greenUp = !greenUp;
          upCounter = 0;
        }  
      } 
    }

    //Displays a single group of lights continously running down the line
    public void down(){ 
      //Starts a loop where the code is constantly moving down the led strip
      for (int i = 0; i < 150; i++){
        m_ledBuffer.setRGB(i, 0, 0, 0);
      }
        
      m_ledBuffer.setRGB(28 + setbrightestone, 25, 25, 0);
      m_ledBuffer.setRGB(28 + setbrightestone + 1, 75, 75, 0);
      m_ledBuffer.setRGB(28 + setbrightestone+2, 150, 150, 0);
      m_ledBuffer.setRGB(28 + setbrightestone+3, 255, 255, 0);

      m_ledBuffer.setRGB(27 - setbrightestone, 25, 25, 0);
      m_ledBuffer.setRGB(27 - setbrightestone-1, 75, 75, 0);
      m_ledBuffer.setRGB(27 - setbrightestone-2, 150, 150, 0);
      m_ledBuffer.setRGB(27 - setbrightestone-3, 255, 255, 0);

      setbrightestone += 1;
      if (setbrightestone > 74)
        setbrightestone = 0;
        
      m_led.setData(m_ledBuffer);
    }

    //Displays a gradient of green and gold that move along the line
    public void GreenGold(){
      int[] colorOne; 
      int[] colorTwo;
      //Gets the Timer 'timer' and changes greenIsFirst to false/true after 0.2 seconds
      if(timer.get() > 0.2) {
        greenIsFirst = !greenIsFirst;
        timer.reset();
      }

      //Runs if 'greenIsFirst' is true. Switches the color of 'colorTwo' with 'colorOne'
      if(greenIsFirst) {
        colorOne = new int[] {0, 150, 0};
        colorTwo = new int[] {150, 70, 0};
      } 
      
      //Runs if 'greenIsFirst' is false. Switches the color of 'colorOne' with 'colorTwo'
      else {
        colorOne = new int[] {150, 70, 0};
        colorTwo = new int[] {0, 150, 0};
      }

      //Makes the LED lights run the assigned colors
      for(int i = 0; i < 100; i++) {
          if(i % 2 == 0) m_ledBuffer.setRGB(i, colorOne[0], colorOne[1], colorOne[2]);
         else {
          m_ledBuffer.setRGB(i, colorTwo[0], colorTwo[1], colorTwo[2]);
        }
      }

      //Sets the data of the LED strip being used
      m_led.setData(m_ledBuffer);
    }

    //An EPIC mode that rapidly changes the LEDs from (69, 4, 20) to (4, 20, 69)
    public void nice(){
      //Creates the integers 'colorOne' and 'colorTwo' as arrays
      int[] colorOne; 
      int[] colorTwo;

      //Uses the timer to change blueIsFirst to false/true every 0.05 seconds
      if(timer.get() > 0.05) {
        blueIsFirst = !blueIsFirst;
        timer.reset();
      }

      //Runs if blueIsFirst is true. Switches 'colorOne' with 'colorTwo'
      if(blueIsFirst) {
        colorOne = new int[] {4, 20, 69};
        colorTwo = new int[] {69, 4, 20};
      } 
      
      //Runs if blueIsFirst is false. Switches 'colorTwo' with 'colorOne'
      else {
        colorOne = new int[] {69, 4, 20};
        colorTwo = new int[] {4, 20, 69};
      }

      //Makes the LEDs run the assigned colors
      for(int i = 0; i < 100; i++) {
          if(i % 2 == 0) m_ledBuffer.setRGB(i, colorOne[0], colorOne[1], colorOne[2]);
         else {
          m_ledBuffer.setRGB(i, colorTwo[0], colorTwo[1], colorTwo[2]);
        }
      }

      //Sets the data for the LED strip being used
      m_led.setData(m_ledBuffer);
    }


    //Displays LEDs that switch from blank to a color
    public void blink(int r, int g, int b) {

      //Creates the integer 'startingLED'
      int startingLED;

      //Runs if the 'blinkeven' boolean is true
      if (blinkeven)  {
          startingLED=1;
      }
      //Runs if the 'blinkeven' boolean is false
      else startingLED = 0;

        //Runs a loop that swithes the colored LEDs to blank
      for (int i = 0; i < 56; i++){
        m_ledBuffer.setRGB(i, 0, 0, 0);
      }
          
      //Runs a loop that switches the blank LEDs to the assigned color
      for (int i = startingLED; i < 56; i+=2){
        m_ledBuffer.setRGB(i, r, g, b);
      }
      
      //Sets the counter that periodically switches the blanks and colors
      counter++;
      if (counter % 10 == 0) {
        blinkeven = !blinkeven;
        m_led.setData(m_ledBuffer);
      }
    }

    //Displays LEDs that switch from one color to another
    public void blinkMultipleColors(int r1, int g1, int b1, int r2, int g2, int b2) {
      //Sets the integer 'startingLED'
      int startingLED;
      
      //Runs if 'blinkeven' is true. Sets 'startingLED' to 1
      if (blinkeven)  {
          startingLED=1;
      }

      //Runs if 'blinkeven' is false. Sets 'startingLED' to 0
      else startingLED = 0;
        
      //Sets the LEDs to the first color
      for (int i = 0; i < 56; i++){
        m_ledBuffer.setRGB(i, r2, g2, b2);
      }

      //Sets the LEDs to the second color
      for (int i = startingLED; i < 56; i+=2){
        m_ledBuffer.setRGB(i, r1, g1, b1);
      }

      //Starts a counter that periodically changes 'blinkeven' to false/true
      counter++;
      if (counter % 10 == 0) {
        blinkeven = !blinkeven;
        //Sets the data of the LED strip
        m_led.setData(m_ledBuffer);
      }  
    }

    //Displays yellow LEDs that move up the strip when climbing
    public void autoClimbLEDs(int speed){
      //Sets a counter that increases depending on the speed that is assigned
      counter++;
      if(counter%speed==0) {
        //Creates a loop that sets the LEDs to blank after a certain amount of LEDs have been assigned to yellow
        for (int i=0;i<56;i++){
          m_ledBuffer.setRGB(i, 0, 0, 0);
        }

        m_ledBuffer.setRGB(setbrightestone, 25, 25, 0);
        m_ledBuffer.setRGB(setbrightestone+1, 75, 75, 0);
        m_ledBuffer.setRGB(setbrightestone+2, 150, 150, 0);
        m_ledBuffer.setRGB(setbrightestone+3, 255, 255, 0);

        m_ledBuffer.setRGB(55-setbrightestone, 25, 25, 0);
        m_ledBuffer.setRGB(55-setbrightestone-1, 75, 75, 0);
        m_ledBuffer.setRGB(55-setbrightestone-2, 150, 150, 0);
        m_ledBuffer.setRGB(55-setbrightestone-3, 255, 255, 0);

        setbrightestone+=1;
        if(setbrightestone>24)
        setbrightestone=0;
        m_led.setData(m_ledBuffer);
      }
    }
}
