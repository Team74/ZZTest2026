package frc.robot.subsystems.swervedrive.TurretSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfigAccessor;

public class Turret_Hood extends SubsystemBase {
    SparkMax hood = new SparkMax(Constants.ShooterConstants.HoodMotorID,MotorType.kBrushless);
    //XboxController controller = new XboxController(1);
    PIDController HoodRangePID = Constants.ShooterConstants.HoodPID;
    double hoodSpeed = 0.25;
   
    DigitalInput magSwitch = new DigitalInput(2);
    AnalogInput hoodPot = new AnalogInput(0);
    AnalogPotentiometer stringPot = new AnalogPotentiometer(hoodPot);
    double hoodPosition;
    double hoodTarget;
    double hoodPIDSpeed;

    public Command MoveHoodIn(){
        return run(()->{
            hood.set(-hoodSpeed);
        });
    }

    public Command MoveHoodOut(){
        return run(()->{
            hood.set(hoodSpeed);
            if (magSwitch.get() == false){
                hood.set(0);
                System.out.println("SWITCHED");
            }
        });
    }

    public Command StopHood(){
        return run(()->{
            hood.set(0);

        });
    }

    public Command MoveHood(boolean reverse){
        return run(()->{
            double desiredSpeed = -hoodSpeed;
            if(reverse){
                desiredSpeed = -desiredSpeed;
            }
            hood.set(desiredSpeed);
        });
    }
    //automatically moves hood to correct position 
    // when given a distance to the hub
    public Command HoodRanging (double Distance) {
        return run(()->{
            hoodPosition = stringPot.get();
            //TODO: find data for (Distance -> hoodPosition) conversion equation
            //converts from distance to hood position
            hoodTarget = (Distance*0.5);
            // moves hood to correct position
            hoodPIDSpeed = HoodRangePID.calculate(hoodPosition, hoodTarget);
            hood.set(Distance);

        });
    }

}
