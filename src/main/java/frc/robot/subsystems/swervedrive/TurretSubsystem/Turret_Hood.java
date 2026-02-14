package frc.robot.subsystems.swervedrive.TurretSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfigAccessor;

public class Turret_Hood {
    SparkMax hood = new SparkMax(Constants.ShooterConstants.HoodMotorID,MotorType.kBrushless);
    XboxController controller = new XboxController(0);
    public void hood(){
        //POV angles: up-0, right-90, down-180, left-270
          if (controller.getPOV() == 0){
            //hood.set(0.5);
            System.out.println("UP");
        } if (controller.getPOV() == 180){
           // hood.set(-0.5);
           System.out.println("DOWN");
        } else {
            hood.set(0);
        }
    }
    //LIMIT SWITCH CODE: EDIT LATER
    // Initializes an AnalogTrigger on port 0
    AnalogTrigger m_trigger0 = new AnalogTrigger(0);
    // Initializes an AnalogInput on port 1
    AnalogInput m_input = new AnalogInput(1);
    // Initializes an AnalogTrigger using the above input
    AnalogTrigger m_trigger1 = new AnalogTrigger(m_input);
    public void analogToDigital(){
        // Enables 2-bit oversampling
        m_input.setAverageBits(2);
        // Sets the trigger to enable at a raw value of 3500, and disable at a value of 1000
        m_trigger0.setLimitsRaw(1000, 3500);
        // Sets the trigger to enable at a voltage of 4 volts, and disable at a value of 1.5 volts
        m_trigger0.setLimitsVoltage(1.5, 4);

    }
}
