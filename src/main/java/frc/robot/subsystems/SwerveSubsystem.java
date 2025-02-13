// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

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

    //THIS IS JUST FOR TRAJECTORIES
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
    new Rotation2d(0),  new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    });

    
    public final Field2d m_field = new Field2d(); //For Glass

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-lite");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

  


    public SwerveSubsystem() {   
      new Thread(() -> {
        try {
            Thread.sleep(1000);
            zeroHeading();
        } catch (Exception e) {
        }
    }).start();
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
    double kP = .02;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = table.getEntry("tx").getDouble(0) * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= Math.PI;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  public double limelight_range_proportional()
  {    
    double kP = 0.05;
    double targetingForwardSpeed = ty.getDouble(0) * kP;
    targetingForwardSpeed *= Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }
  @Override

  public void periodic() {

    //ALL OF THIS HAPPENS 5 TIMES A SECOND
    //Just stuff for debugging and for Driver!
    double limelight_x = tx.getDouble(0.0);
    double limelight_y = ty.getDouble(0.0);
    double limelight_area = ta.getDouble(0.0);

      odometer.update(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()});

    SmartDashboard.putString("Robot Location", getPose().toString());
    SmartDashboard.putBoolean("AutoBalance?", gyro.getPitch() <= 1);
    SmartDashboard.putData(m_field);
    SmartDashboard.putNumber("Pitch Of Robot", gyro.getPitch());

    
    SmartDashboard.putNumber("Turn BL", backLeft.getTurningPosition());
    SmartDashboard.putNumber("Turn FL", frontLeft.getTurningPosition());
    SmartDashboard.putNumber("Turn BR", backRight.getTurningPosition());
    SmartDashboard.putNumber("Turn FR", frontRight.getTurningPosition());

    SmartDashboard.putNumber("LimelightX", limelight_x*Math.PI);
    SmartDashboard.putNumber("LimelightY", limelight_y);
    SmartDashboard.putNumber("LimelightArea", limelight_area);
    
    m_field.setRobotPose(odometer.getPoseMeters());
  }

}