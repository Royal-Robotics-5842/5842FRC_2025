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
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants.Side;
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
    
  public Pose2d lastPose = new Pose2d(0,0, new Rotation2d(0));  
  public DriverStation.Alliance allianceColor = DriverStation.getAlliance().get() ;
  public final Field2d m_field = new Field2d(); //For Glass
  RobotConfig config;
  ChassisSpeeds speeds;
  Pose2d finalPose = new Pose2d();

  public ProfiledPIDController thetaController = new ProfiledPIDController(2, 0, 0,
    new TrapezoidProfile.Constraints(6.28, 3.14));
    
  public HolonomicDriveController holonomicController = new HolonomicDriveController(
    new PIDController(1.3, 0, 0), 
    new PIDController(1.3, 0, 0),
      thetaController);
    
  //Old way of constructing //public final AHRS gyro = new AHRS(SPI.Port.kMXP); //Defineing the Gyro
  public final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  public final PIDController turningPID = new PIDController(0.012, 0, 0);
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public SwerveSubsystem() { 
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    //gyro.reset();
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
      this::resetSwervePoseEstimator,
      this::getRobotRelativeSpeeds,   
      (speeds, feedforwards) -> driveRobotRelative(speeds),
      new PPHolonomicDriveController(
        new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.1),
        new PIDConstants(Constants.AutoConstants.kPThetaController, 0.25, 0.2)
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

    /*
      PathPlannerPath plannedPath = new PathPlannerPath(
        PathPlannerPath.waypointsFromPoses(
        SwerveDrivePoseEstimator.getEstimatedPosition(), 
        finalPose),

      Constants.AutoConstants.pathPlanningConstraints,
       null,// The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );
    plannedPath.preventFlipping = true;
    return AutoBuilder.followPath(plannedPath);
    */
    
    //THIS IS JUST FOR TRAJECTORIES
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
    new Rotation2d(),  new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    });

  public SwerveDrivePoseEstimator SwerveDrivePoseEstimator = new SwerveDrivePoseEstimator
    (Constants.DriveConstants.kDriveKinematics, new Rotation2d(), new SwerveModulePosition[] 
    {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    }, 
    Pose2d.kZero,
    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));


  public void setLastPose(Pose2d pose2d) 
  {
    lastPose = pose2d;
  }

  public void driveRobotRelative(ChassisSpeeds robotRelative) 
  {
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
    return chassisSpeeds;
  }


  public void zeroHeading() 
  {
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
  public Pose2d getPose() 
  {
    return odometer.getPoseMeters();
  } 

  public Pose2d getLimelightPose2d() {
    return SwerveDrivePoseEstimator.getEstimatedPosition();//.limeLightPosition;
  } 

  //FOR TRAJECTORIES
  public void resetOdometry(Pose2d pose) 
  {
    odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
     } 
     ,pose);
  }

  public void resetSwervePoseEstimator(Pose2d pose)
  {
    SwerveDrivePoseEstimator.resetPosition(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
     } 
     ,pose);
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

  @Override

  public void periodic() {
    SwerveDrivePoseEstimator.update(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()});
    
    m_field.setRobotPose(getLimelightPose2d());//odometer.getPoseMeters());
    //ALL OF THIS HAPPENS 5 TIMES A SECOND
    //Just stuff for debugging and for Driver!

    odometer.update(getRotation2d(), new SwerveModulePosition[] {
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backLeft.getPosition(),
    backRight.getPosition()});
      
    allianceColor = DriverStation.getAlliance().get();

    
    SmartDashboard.putData(m_field);
    SmartDashboard.putNumber("GYRO", gyro.getAngle());
    SmartDashboard.putString("Robot Location (ODEMETER)", getPose().toString());
    SmartDashboard.putString("LimeLightPoseEstimate", SwerveDrivePoseEstimator.getEstimatedPosition().toString());
    
    boolean useMegaTag2 = true; //set to false to use MegaTag1
      boolean doRejectUpdate = false;
      if (useMegaTag2 == true && 
      LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-lite")!= null)
      {
        
        LimelightHelpers.SetRobotOrientation("limelight-lite", SwerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-lite");
        SmartDashboard.putNumber("MONKEY",mt2.avgTagDist);
        
    if(mt2 != null)
    {
      if(Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if (mt2.avgTagDist < 0.0)
      {
        doRejectUpdate =true;
      }
      if(!doRejectUpdate)
      {
        SwerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        SwerveDrivePoseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
    
  }
  }

}