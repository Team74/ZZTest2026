package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.studica.frc.AHRS;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.LimelightHelpers;

public class Dashboard {
    DoublePublisher LFdriveSpeed, RFdriveSpeed, LBdriveSpeed, RBdriveSpeed;
    DoublePublisher xPub;
    DoublePublisher Encoder1,Encoder2,Encoder3,Encoder4,Gyro,Pose_X,Pose_Y,Pose_rot;
    SendableChooser<String> autoncode1;
    DoubleSubscriber ySub;
    Dashboard(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("datatable");
        xPub = table.getDoubleTopic("x").publish();
       
        Encoder1 = table.getDoubleTopic("Encoder1").publish();
        Encoder2 = table.getDoubleTopic("Encoder2").publish();
        Encoder3 = table.getDoubleTopic("Encoder3").publish();
        Encoder4 = table.getDoubleTopic("Encoder4").publish();
        
        autoncode1 = new SendableChooser<String>();
        SmartDashboard.putData("Auton Chooser", autoncode1); 
        autoncode1.addOption("one", "one");
        autoncode1.addOption("two", "two"); 
        autoncode1.addOption("three", "three");
        autoncode1.addOption("four", "four");
       
        Gyro = table.getDoubleTopic("Gyro").publish();

        LFdriveSpeed = table.getDoubleTopic("LF Speed").publish();
        RFdriveSpeed = table.getDoubleTopic("RF Speed").publish();
        LBdriveSpeed = table.getDoubleTopic("LB Speed").publish();
        RBdriveSpeed = table.getDoubleTopic("RB Speed").publish();
        ySub = table.getDoubleTopic("y").subscribe(0.0);
        table.getDoubleTopic("y").publish(); 

        Pose_X = table.getDoubleTopic("Pose_X").publish();
        Pose_Y = table.getDoubleTopic("Pose_Y").publish();
        Pose_rot = table.getDoubleTopic("Pose_rot").publish();
    }
    void updateDashboard(){
        xPub.set(1.0);
     //   System.out.println(ySub.get();

    }
    void updatePose (double X, double Y, double rot) {
    Pose_X.set(X);
    Pose_Y.set(Y);
    Pose_rot.set(rot);
    }
    void updateDashboardSwerveModules( SwerveModule leftFront, SwerveModule rightFront, SwerveModule leftBack, SwerveModule rightBack){
    
        double LFAngle = leftFront.returnRotation(); 
        Encoder1.set(LFAngle);
        double LBAngle = leftBack.returnRotation(); 
        Encoder2.set(LBAngle);
        double RFAngle = rightFront.returnRotation(); 
        Encoder3.set(RFAngle);
        double RBAngle = rightBack.returnRotation(); 
        Encoder4.set(RBAngle);

        double LFSpeed = leftFront.driveMotor.getEncoder().getVelocity(); 
        LFdriveSpeed.set(LFSpeed*-1);
        double RFSpeed = rightFront.driveMotor.getEncoder().getVelocity(); 
        RFdriveSpeed.set(RFSpeed*-1);
        double LBSpeed = leftBack.driveMotor.getEncoder().getVelocity(); 
        LBdriveSpeed.set(LBSpeed*-1);
        double RBSpeed = rightBack.driveMotor.getEncoder().getVelocity(); 
        RBdriveSpeed.set(RBSpeed*-1);
    }       
    void updateDashboardGyro(AHRS gyro){
        Gyro.set(gyro.getAngle());
    }
    void updatefielddata (Field2d m_field) {
        SmartDashboard.putData(m_field);

    }
   /*  public Field2d()
    public void setRobotPose(
        double xMeters,
        double yMeters,
        Rotation2d rotation)
    public Pose2d getRobotPose()
*/
}