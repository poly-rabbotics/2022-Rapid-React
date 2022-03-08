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

/**
 * 
 * LEDs:pp
 * Green (Moving Upward):     Ready to shoot
 * Green (static):            Idling
 * Yellow (Static):           Teleop Activated
 * Rainbow (Epic Gaymer):     Bot is Offline
 * Blue (Moving Upaward):     Robot is ready to climb
 * Purple (Moving Forward):   Bot is ready to transfer to next bar.
 * Rainbow (Epic Gaymer):     Autonomous Activated
 * 
 * 
 */

public class LEDLights {

    AddressableLEDBuffer m_ledBuffer;
    AddressableLED m_led;
    int m_rainbowFirstPixelHue = 0;
    int setbrightestone = 0;
    int counter = 0;
    boolean blinkeven = false;
    public static int pattern = 2;
    Timer timer = new Timer();
    boolean greenIsFirst = true;
    boolean greenUp = false;
    int upCounter = 0;

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
            //disabled/rainbow
            case 1:
              rainbow();
              break;
            //teleop/greengold  
            case 2:
              GreenGold();
              break;
            //autonomous
            case 3:
              up(2);
              break;
            //one ball recieved/solid purple
            case 4:
              singleColor(255, 0, 255); //RR: actually lets change this to solid green if that's not already used for something else
              break;
            //shooter up to speed/flash blue
            case 5:
              blink(0, 0, 255);
              break;
            //PID drive enabled/flash red
            case 6:
              blink(255, 0, 0);
              break;
            //High Torque Mode enabled/solid orange
            case 7:
              singleColor(255, 100, 0);
              break;
            //Shooter up to speed & PID enabled/flashing blue & red
            case 8:
              blinkMultipleColors(255, 0, 0, 0, 0, 255);
              break;
            //HTM & PID Enabled/flashing red & orange
            case 9:
              blinkMultipleColors(255, 0, 0, 255, 100, 0);
              break;

            default:
              rainbow();
         }
 
      }

      public void singleColor(int r,int g, int b) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, r, g, b);
         }
         
         m_led.setData(m_ledBuffer);
        
      }

      public void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
          // Set the value
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
      }

      public void up(int speed){
          counter++;
          if (counter%speed==0) {

          if(!greenUp) {

            m_ledBuffer.setRGB(setbrightestone, 150, 70, 0);
            m_ledBuffer.setRGB(setbrightestone + 2, 100, 97, 0);
            m_ledBuffer.setRGB(setbrightestone + 4, 50, 123, 0);
            m_ledBuffer.setRGB(setbrightestone + 6, 0, 150, 0);

          }
          
          else if(greenUp) {            
            m_ledBuffer.setRGB(setbrightestone, 0, 150, 0);
            m_ledBuffer.setRGB(setbrightestone + 2, 50, 123, 0);
            m_ledBuffer.setRGB(setbrightestone + 4, 100, 97, 0);
            m_ledBuffer.setRGB(setbrightestone + 6, 150, 70, 0);
          }
         setbrightestone+=1;
         if(setbrightestone>33)
            setbrightestone=0;
            upCounter++;
            m_led.setData(m_ledBuffer);

            if(upCounter >= 34) {
              greenUp = !greenUp;
              upCounter = 0;
            }
            
          }
          
      }

      public void down(){
        
        for (int i = 0; i < 150; i++){
          m_ledBuffer.setRGB(i, 0, 0, 0);
      
          }

        // (int i=setbrightestone;i<setbrightestone+4;i+=4){
        m_ledBuffer.setRGB(28 + setbrightestone, 25, 25, 0);
        m_ledBuffer.setRGB(28 + setbrightestone + 1, 75, 75, 0);
        m_ledBuffer.setRGB(28 + setbrightestone+2, 150, 150, 0);
        m_ledBuffer.setRGB(28 + setbrightestone+3, 255, 255, 0);

        m_ledBuffer.setRGB(27 - setbrightestone, 25, 25, 0);
        m_ledBuffer.setRGB(27 - setbrightestone-1, 75, 75, 0);
        m_ledBuffer.setRGB(27 - setbrightestone-2, 150, 150, 0);
        m_ledBuffer.setRGB(27 - setbrightestone-3, 255, 255, 0);
       // }
       setbrightestone += 1;
       if (setbrightestone > 74)
          setbrightestone = 0;
       
        
        m_led.setData(m_ledBuffer);
    }

    public void GreenGold(){
      int[] colorOne; 
      int[] colorTwo;

      if(timer.get() > 0.2) {
        greenIsFirst = !greenIsFirst;
        timer.reset();
      }
      if(greenIsFirst) {
        colorOne = new int[] {0, 150, 0};
        colorTwo = new int[] {150, 70, 0};
      } else {
        colorOne = new int[] {150, 70, 0};
        colorTwo = new int[] {0, 150, 0};
      }

      for(int i = 0; i < 100; i++) {
          if(i % 2 == 0) m_ledBuffer.setRGB(i, colorOne[0], colorOne[1], colorOne[2]);
         else {
          m_ledBuffer.setRGB(i, colorTwo[0], colorTwo[1], colorTwo[2]);
        }
      }

      m_led.setData(m_ledBuffer);
    }

    public void nice(){
      int[] colorOne; 
      int[] colorTwo;

      if(timer.get() > 0.05) {
        greenIsFirst = !greenIsFirst;
        timer.reset();
      }
      if(greenIsFirst) {
        colorOne = new int[] {4, 20, 69};
        colorTwo = new int[] {69, 4, 20};
      } else {
        colorOne = new int[] {69, 4, 20};
        colorTwo = new int[] {4, 20, 69};
      }

      for(int i = 0; i < 100; i++) {
          if(i % 2 == 0) m_ledBuffer.setRGB(i, colorOne[0], colorOne[1], colorOne[2]);
         else {
          m_ledBuffer.setRGB(i, colorTwo[0], colorTwo[1], colorTwo[2]);
        }
      }

      m_led.setData(m_ledBuffer);
    }

    public void blink(int r, int g, int b)
    {
        

        int startingLED;
        if (blinkeven)  {
            startingLED=1;
        }
        else startingLED = 0;

        for (int i = 0; i < 56; i++){
            m_ledBuffer.setRGB(i, 0, 0, 0);
        
            }

         for (int i = startingLED; i < 56; i+=2){
            m_ledBuffer.setRGB(i, r, g, b);
             
            }
            counter++;
            if (counter % 10 == 0)
            {
                blinkeven = !blinkeven;
                m_led.setData(m_ledBuffer);
            }
        
    }

    public void blinkMultipleColors(int r1, int g1, int b1, int r2, int g2, int b2)
    {
        int startingLED;
        if (blinkeven)  {
            startingLED=1;
        }
        else startingLED = 0;
        

        for (int i = 0; i < 56; i++){
            m_ledBuffer.setRGB(i, r2, g2, b2);
        
            }

         for (int i = startingLED; i < 56; i+=2){
            m_ledBuffer.setRGB(i, r1, g1, b1);
             
            }

            counter++;
            if (counter % 10 == 0)
            {
                blinkeven = !blinkeven;
                m_led.setData(m_ledBuffer);
            }
        
    }

}
