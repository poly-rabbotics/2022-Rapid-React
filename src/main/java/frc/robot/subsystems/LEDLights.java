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
 * LEDs:
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

    public LEDLights() {

        timer.start();
        m_led = RobotMap.led;
    
        
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(150);
        m_led.setLength(m_ledBuffer.getLength());
    
        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
      }

      public void run(int pattern, int[] values) {
         switch(pattern){
            case 1:
                up(values[0], values[1], Integer.toString(values[2]));
                break;

            case 2:
                singleColor(values[0], values[1], values[2]);

            case 3:
                blink(values[0], values[1], values[2]);
                
            case 4:
                rainbow();

            default:
                singleColor(0,0,0);
         }
 
      }

      public void singleColor(int r,int g, int b) {
        for (int i = 0; i < 56; i++) {
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

      public void up(int distance, double speed, String color){
          counter++;
          int interval = 255/distance;
          if (counter%speed==0) {

          for (int i = 0; i < 56; i++){
            m_ledBuffer.setRGB(i, 0, 0, 0);
        
          }

          if(color == "blue") {
            /*
            for(int i = 0; i < distance; i++) {
              int thing = i - 1;
              m_ledBuffer.setRGB(setbrightestone + thing, interval*i, interval*i, 255);
            }
            */

            /*
            m_ledBuffer.setRGB(setbrightestone, 0, 0, 255);
            m_ledBuffer.setRGB(setbrightestone + 1, 75, 75, 255);
            m_ledBuffer.setRGB(setbrightestone + 2, 150, 150, 255);
            m_ledBuffer.setRGB(setbrightestone + 3, 255, 255, 255);
          
            
            m_ledBuffer.setRGB(55-setbrightestone, 0, 0, 255);
            m_ledBuffer.setRGB(55-setbrightestone + 1, 75, 75, 255);
            m_ledBuffer.setRGB(55-setbrightestone + 2, 150, 150, 255);
            m_ledBuffer.setRGB(55-setbrightestone + 3, 255, 255, 255);
            */
          }
          
          else if(color == "green") {
            m_ledBuffer.setRGB(setbrightestone, 0, 25, 0);
            m_ledBuffer.setRGB(setbrightestone + 1, 0, 75, 0);
            m_ledBuffer.setRGB(setbrightestone + 2, 0, 150, 0);
            m_ledBuffer.setRGB(setbrightestone + 3, 0, 255, 0);
  
            m_ledBuffer.setRGB(55-setbrightestone, 0, 25, 0);
            m_ledBuffer.setRGB(55-setbrightestone-1, 0, 75, 0);
            m_ledBuffer.setRGB(55-setbrightestone-2, 0, 150, 0);
            m_ledBuffer.setRGB(55-setbrightestone-3, 0, 255, 0);
          }

          /*
          // (int i=setbrightestone;i<setbrightestone+4;i+=4){
          m_ledBuffer.setRGB(setbrightestone, 25, 25, 0);
          m_ledBuffer.setRGB(setbrightestone + 1, 75, 75, 0);
          m_ledBuffer.setRGB(setbrightestone + 2, 150, 150, 0);
          m_ledBuffer.setRGB(setbrightestone + 3, 255, 255, 0);

          m_ledBuffer.setRGB(55-setbrightestone, 25, 25, 0);
          m_ledBuffer.setRGB(55-setbrightestone-1, 75, 75, 0);
          m_ledBuffer.setRGB(55-setbrightestone-2, 150, 150, 0);
          m_ledBuffer.setRGB(55-setbrightestone-3, 255, 255, 0);
         // }
         */
         setbrightestone+=1;
         if(setbrightestone>55)
            setbrightestone=0;
            m_led.setData(m_ledBuffer);
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
            m_ledBuffer.setRGB(i, 255, 0, 0);
             
            }
            counter++;
            if (counter % 10 == 0)
            {
                blinkeven = !blinkeven;
                m_led.setData(m_ledBuffer);
            }
        
    }

}
