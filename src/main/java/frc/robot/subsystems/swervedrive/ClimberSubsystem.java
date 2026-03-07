package frc.robot.subsystems.swervedrive;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{

    SparkMax climbMax;
    double climbSpeed;

    public ClimberSubsystem(){
        climbMax = new SparkMax(Constants.ClimberConstants.ClimbMotorID, MotorType.kBrushless);
        climbSpeed = Constants.ClimberConstants.ClimbSpeed;
    }

    public Command ClimbUp(){
        return run(()->{
          climbMax.set(climbSpeed);
        });
    } 
    public Command ClimbDown(){
        return run(()->{
          climbMax.set(-climbSpeed);
        });
    } 
    public Command ClimbStop(){
        return run(()->{
          climbMax.set(0);
        });
    } 
}
