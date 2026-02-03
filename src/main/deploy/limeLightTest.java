package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers.PoseEstimate;

import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limeLightTest {
    AHRS gyro;
    double limex;
    double limey;
    double limearea;
    double currentTarget;
    PIDController PIDAngle = new PIDController(0.016667*2, 0, 0);
    PIDController PIDPush = new PIDController(0.016667*1.2, 0, 0);
    private final ProfiledPIDController rotationPID;
    private final ProfiledPIDController rangePID;
    driveTrain driveTrain;
    limeLightTest(driveTrain _DriveTrain) {
        driveTrain = _DriveTrain;
        gyro = driveTrain.gyro;

        // Tune these PID values for your robot
        //rotationPID = new PIDController(0.0025+0.0023889, 0, 0);
        rotationPID = new ProfiledPIDController(.04, 0.0, 0.0, new TrapezoidProfile.Constraints(300, 200));
        rangePID = new ProfiledPIDController(0.3, 0.0, 0.0, new TrapezoidProfile.Constraints(50, 5));

        // Set tolerance for both controllers
        rotationPID.setTolerance(0.3); // 1 degree tolerance
        rangePID.setTolerance(0.01); // 5cm tolerance
        
        rotationPID.enableContinuousInput(-180.0, 180.0);
        rangePID.enableContinuousInput(-180.0, 180.0);
    }

    public double LimeTest () {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        

        //read values periodically
        limex = tx.getDouble(0.0);
        limey = ty.getDouble(0.0);
        limearea = ta.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", limex);
        SmartDashboard.putNumber("LimelightY", limey);
        SmartDashboard.putNumber("LimelightArea", limearea);
       // System.out.println("tx value: " + limex + "ty value:" + limey + "ta value:" + limearea);
        
        if (limex < - (5 + limearea)) {
   
            return -PIDPush.calculate(limex, 0.0);
        }
        if (limex > (5 + limearea)) {
         
            return -PIDPush.calculate(limex, 0.0);
        }
        return 0;
    }
    
    double ReefCenter() {
        currentTarget =  60 * (int)Math.round((gyro.getAngle() % 360) / 60);
        return calculaterotation(currentTarget);
 
    }
    double calculaterotation(Double targetangle){
        double currentAngle = (gyro.getAngle() % 360); 
        //System.out.println("currentAngle: " + currentAngle + " targetangle:" + targetangle);
        
        return PIDAngle.calculate(currentAngle,targetangle);

        // if (currentAngle > targetangle + (2 + limearea)) {
      
        //     return PIDAngle.calculate(currentAngle,currentTarget);  
        // }
        // if (currentAngle < targetangle - (2 + limearea)) {
     
        //     return PIDAngle.calculate(currentAngle,targetangle);
        // }
        // return 0; 
    }
    double ReefPush() {
        if (!TrackCheck(gyro.getAngle(), currentTarget, (5 + limearea))){
            return 0.0;
        }
        if (!TrackCheck(limex, 0, (8 + limearea))){
            return 0.0;
        }
        
        if (limearea > 0 && limearea < 9) {
            if (ReefCenter() == 0 && LimeTest() == 0) {
           
                return -0.5;
            } else {
         
                return -0.2;
            }
        }
        
    return 0;
    }
    boolean TrackCheck(double input, double target,double error) {
    return (input < (target + error) && input > (target - error));
    }
    boolean CanSee(){
        if (limearea == 0.0){
            return false;
        } else {
            return true;
        }
    }

    public double limelight_aim_proportional(double kMaxAngularSpeed)
    {    
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .035;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= kMaxAngularSpeed;

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are different.  
    // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
    public double limelight_range_proportional(double kMaxSpeed)
    {    
        double kP = .1;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= kMaxSpeed;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }

    void LimeTarget(double getPeriod){
        LimeTargetWithRot(getPeriod, 0);
    }
    void LimeTargetWithRot(double getPeriod, double rot){
        double rotationOutput = LLGetRotation();
        double rangeOutput = LLGetRangeOutput();
        
        rotationOutput = MathUtil.clamp(rotationOutput, -1, 1);
        rangeOutput = MathUtil.clamp(rangeOutput, -0.7, 0.7);

        driveTrain.driveLL(rangeOutput, -rotationOutput, rot, false, getPeriod);
    }

    double LLGetRangeOutput() {
        var tag = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
        var tag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        double dist = tag.getTranslation().getNorm();
        System.out.println("tag rot: " + tag.getRotation().getAngle() + " tag1 rot: " + tag1.pose.getRotation() + " tag1 x: " + tag1.pose.getX() + " tag1 y: " + tag1.pose.getY());
        // Calculate control outputs
        double rangeOutput = rangePID.calculate(dist, 0.2);
        rangeOutput *= 4;
    
        rangeOutput = MathUtil.applyDeadband(rangeOutput, 0.1);
        
        return rangeOutput;
    }

    double LLGetRotation() {
        double ty = LimelightHelpers.getTY("limelight");
        
        double rotationOutput = rotationPID.calculate(ty, 0.0);

        return rotationOutput;
    }

    double CurrentTargetId() {
        var tagId = LimelightHelpers.getFiducialID("limelight");

        return tagId;
    }
}