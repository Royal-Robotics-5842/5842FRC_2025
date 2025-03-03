// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;

//Making the SwerveModules with all the different paramters

public class SwerveSubsystem extends SubsystemBase {
    public final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveMotorReversed,
            DriveConstants.kFrontLeftTurningMotorReversed,
            DriveConstants.kFrontLeftTurnAbsoluteEncoderPort);

    public final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveMotorReversed,
            DriveConstants.kFrontRightTurningMotorReversed,
            DriveConstants.kFrontRightTurnAbsoluteEncoderPort);

    public final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveMotorReversed,
            DriveConstants.kBackLeftTurningMotorReversed,
            DriveConstants.kBackLeftTurnAbsoluteEncoderPort);

    public final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveMotorReversed,
            DriveConstants.kBackRightTurningMotorReversed,
            DriveConstants.kBackRightTurnAbsoluteEncoderPort);

    //Old way of constructing //public final AHRS gyro = new AHRS(SPI.Port.kMXP); //Defineing the Gyro
    public final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    public final PIDController turningPID = new PIDController(0.012, 0, 0);

    //THIS IS JUST FOR TRAJECTORIES
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
    new Rotation2d(0),  new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    });

    
    public final Field2d m_field = new Field2d(); //For Glass
    
    RobotConfig config;
    // Basic targeting data
    
    public SwerveSubsystem() { 
        
      new Thread(() -> {
        try {
            Thread.sleep(1000);
            zeroHeading();
        } catch (Exception e) {
        }
    }).start();

     try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Register swerve auto
    AutoBuilder.configure(
      this::getLimelightPose2d,
      this::resetOdometry,
      this::getRobotRelativeSpeeds,   
      (speeds, feedforwards) -> driveRobotRelative(speeds),
      new PPHolonomicDriveController(
        new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0),
        new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0)
      ), 
      config,
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
        }, this
      
    );

}
  public Command gotoPath(List<Waypoint> path) {
      PathPlannerPath plannedPath = new PathPlannerPath(path,
        Constants.AutoConstants.pathPlanningConstraints,
       null,// The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );
    plannedPath.preventFlipping = true;
    System.out.println("HIIIIII");
    return AutoBuilder.followPath(plannedPath);
  }

 public void driveRobotRelative(ChassisSpeeds robotRelative) {
  SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(robotRelative);
  setModuleStates(targetStates);
 }


public SwerveModuleState[] getModuleStates()
{
  SwerveModuleState[] states = new SwerveModuleState[] {
    frontRight.getState(),
    frontLeft.getState(),
    backRight.getState(),
    backLeft.getState()
  };

  return states;
}

public ChassisSpeeds getRobotRelativeSpeeds() 
{
  return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
}


  public void zeroHeading() {
      gyro.reset(); //Making all values 0 (Pitch, Yaw, Roll)
  }
  
  public double getHeading() 
  {
    return Math.IEEEremainder(-gyro.getAngle(), 360); //Get the position the swerve is facing
  }
    
  public Rotation2d getRotation2d()
  {
    return Rotation2d.fromDegrees(getHeading()); //For FRC functions, just converts where your facing to a "Rotation2d" type
  }
  
  //FOR TRAJECTORIES
  public Pose2d getPose() {
    return odometer.getPoseMeters();
  } 

  public Pose2d getLimelightPose2d() {
    return SwerveDrivePoseEstimator.getEstimatedPosition();//.limeLightPosition;
  } 

  //FOR TRAJECTORIES
  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
     } ,pose);
  }

  public void stopModules() 
  {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) 
  {
    //Not sure tbh, just FRC stuff -- read WPILIB Documentation
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]); 
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double targetingAngularVelocity = turningPID.calculate(LimelightHelpers.getBotPose2d("limelight-lite").getRotation().getDegrees(), 0);
    

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
  
    // convert to radians per second for our drive method
    //targetingAngularVelocity *= Math.PI;

    //invert since tx is positive when the target is to the right of the crosshair
    //targetingAngularVelocity *= -1.0;
    
    return targetingAngularVelocity;
  }

  public SwerveDrivePoseEstimator SwerveDrivePoseEstimator = new SwerveDrivePoseEstimator
  (Constants.DriveConstants.kDriveKinematics, getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()}, Pose2d.kZero,
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  public double limelight_range_proportional()
  {    
    double kP = 0.05;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight-lite") * kP;
    targetingForwardSpeed *= Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }
  @Override

  public void periodic() {
    double tx = LimelightHelpers.getTX("limelight-lite");  // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY("limelight-lite");  // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA("limelight-lite");  // Target area (0% to 100% of image)
    boolean hasTarget = LimelightHelpers.getTV("limelight-lite"); // Do you have a valid target?

  
    double txnc = LimelightHelpers.getTXNC("limelight-lite");  // Horizontal offset from principal pixel/point to target in degrees
    double tync = LimelightHelpers.getTYNC("limelight-lite");  // Vertical  offset from principal pixel/point to target in degrees


    //ALL OF THIS HAPPENS 5 TIMES A SECOND
    //Just stuff for debugging and for Driver!

      odometer.update(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()});
    

      boolean useMegaTag2 = true; //set to false to use MegaTag1
      boolean doRejectUpdate = false;
      if(useMegaTag2 == false)
      {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-lite");
        
        if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
        {
          if(mt1.rawFiducials[0].ambiguity > .7)
          {
            doRejectUpdate = true;
          }
          if(mt1.rawFiducials[0].distToCamera > 3)
          {
            doRejectUpdate = true;
          }
        }
        if(mt1.tagCount == 0)
        {
          doRejectUpdate = true;
        }
  
        if(!doRejectUpdate)
        {
          SwerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
          SwerveDrivePoseEstimator.addVisionMeasurement(
              mt1.pose,
              mt1.timestampSeconds);
        }
      }
      else if (useMegaTag2 == true)
      {
        LimelightHelpers.SetRobotOrientation("limelight-lite", SwerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-lite");
        if(Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
          doRejectUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
          doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
          SwerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
          SwerveDrivePoseEstimator.addVisionMeasurement(
              mt2.pose,
              mt2.timestampSeconds);
        }
      }

  

    
    SmartDashboard.putString("Robot Location", getPose().toString());
    SmartDashboard.putData(m_field);

    
    SmartDashboard.putNumber("Turn BL", backLeft.getTurningPosition());
    SmartDashboard.putNumber("Turn FL", frontLeft.getTurningPosition());
    SmartDashboard.putNumber("Turn BR", backRight.getTurningPosition());
    SmartDashboard.putNumber("Turn FR", frontRight.getTurningPosition());

    SmartDashboard.putString("LimeLightPose", LimelightHelpers.getBotPose2d("limelight-lite").toString());
    SmartDashboard.putString("LimeLightPoseEstimate", SwerveDrivePoseEstimator.getEstimatedPosition().toString());
    
    SwerveDrivePoseEstimator.update(gyro.getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()});
    m_field.setRobotPose(SwerveDrivePoseEstimator.getEstimatedPosition());//odometer.getPoseMeters());
  
    Pose2d limeLightPosition = LimelightHelpers.getBotPose2d_wpiBlue("limelight-lite");

  }

}