package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.LEDReader.IndexedColorIterator;
import edu.wpi.first.wpilibj.util.Color;

import java.util.Iterator;
import java.util.Optional;
import java.util.Random;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
public class LEDs extends SubsystemBase {
  final AddressableLED leds;
  final AddressableLEDBuffer ledBuff;

  final Random random = new Random();
  final int[] raindrop =
      new int[180]; // Random numbers generated at startup. https://xkcd.com/221/.
  String hubStart;
  boolean isRedHubActive;
  Color allianceColor;

  Timer timer;
  int timespeed;
  int time;
  int value; // COLOR
  int value2;
  int x;
  int i = -1; // makes the ControlledLED function start at index 0 after first iteration
  int i2 = -1; // makes the ControlledLED function start at index 0 after first iteration
  int hue = -1; // make the ControlledLED function start at hue 0
  double leftTrigger;

  final boolean solidColor = false;
  Color color = Color.kRed;
  
  final Distance LED_spacing = Units.Meters.of(1/14);  // 14  led groups  per meter
  final LinearVelocity lin_vel = Units.MetersPerSecond.of(1);
  
  // defines a Rainbow pattern and a scrolling version
  Frequency pattern_Rainbow_Speed = Frequency.ofBaseUnits((.1), Units.Hertz);
  final LEDPattern pattern_Rainbow = LEDPattern.rainbow(255, 255);
  final LEDPattern pattern_Rainbow_Scroll = pattern_Rainbow.scrollAtRelativeSpeed(pattern_Rainbow_Speed);

  // defines a Red-Blue Gradient pattern and a scrolling version
  Frequency pattern_Grad_Speed = Frequency.ofBaseUnits(1, Units.Hertz);
  final LEDPattern pattern_Grad = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kBlue); 
  final LEDPattern pattern_Grad_scroll = pattern_Grad.scrollAtRelativeSpeed(pattern_Grad_Speed);

  // defines which pattern gets applied to the LED
  final LEDPattern pattern_To_Apply = pattern_Rainbow_Scroll;

  public LEDs() {
    leds = new AddressableLED(5);
    ledBuff =
        new AddressableLEDBuffer(
            70); // 420 LEDs on the strip total but 6 LEDs/LED group (at 14/Meter the strip is 5 Meters)
    leds.setLength(ledBuff.getLength());
    leds.setData(ledBuff);
    leds.start();
  }

  public void Pattern() {
    if (!solidColor) {
      // apply pattern to buff
      pattern_To_Apply.applyTo(ledBuff);
      // write buff to led strip
    } 
    else {
      i++;
      if(i >= 70){
      i = 0;
      } 
      ledBuff.setLED(i,color);
    } leds.setData(ledBuff);
  }

  public Command ColorChange(Color colorToChange) { 
    return run(()->{
      i2++;
      if(i2 >= 70){
      i2 = 0;
      } 
      ledBuff.setLED(i2,color);
      leds.setData(ledBuff);
    });
  }
    
  public void ControlledLED() {
    hue++;
    if (hue > 180){
      hue = 0;
    }
    i++;
    if(i >= 70){
      i = 0;
    } 
    ledBuff.setHSV(i, hue, 255, 255);
    leds.setData(ledBuff);
  }

  public Color HubTimer() {
  Optional<Alliance>  alliance =  DriverStation.getAlliance();
  switch (alliance.toString()){
    case "Blue": allianceColor = Color.kBlue;
    break;

    case "Red" : allianceColor = Color.kRed;
    break;

    default: allianceColor = Color.kBlack;
    break;
    }
  switch (DriverStation.getGameSpecificMessage().charAt(0)){
    case 'R': isRedHubActive = false;
    break;

    case 'B': isRedHubActive = true;
    break;

    default: return allianceColor;
  }
  if (DriverStation.isTeleopEnabled() && !timer.isRunning()) {
    timer.reset();
    timer.start();
  }

  if (timer.get() >= 110) {
    // Endgame 
    // always green
    return Color.kGreen;

  } else if (timer.get() >= 85) {
    // Shift 4
    // if not red first and we are red then green, otherwise our color
      if(!isRedHubActive && alliance.toString() == "Red") {
      return Color.kGreen;  
    } 
    else {return allianceColor;}

  } else if (timer.get() >= 60) {
    // Shift 3
    // if red first and we are red then green, otherwise our color
      if(isRedHubActive && alliance.toString() == "Red") {
      return Color.kGreen;  
    } 
    else {return allianceColor;}

  } else if (timer.get() >= 35) {
    // Shift 2 
    // if not red first and we are red then green, otherwise our color
        if(!isRedHubActive && alliance.toString() == "Red") {
      return Color.kGreen;  
    } 
    else {return allianceColor;}

  } else if (timer.get() >= 10) {
    // Shift 1
    // if red first and we are red then green, otherwise our color
    if(isRedHubActive && alliance.toString() == "Red") {
      return Color.kGreen;  
    } 
    else {return allianceColor;}

  } else {
    // Transition phase
    // always green
    return Color.kGreen;
  }
  }
}
