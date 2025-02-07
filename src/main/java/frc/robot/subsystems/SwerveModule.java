package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/*
    This class defines the functions of each INDIVIDUAL module. This allows for us to run commands on each module.
    It also allows us to then combine all 4 individual modules and run functions in the "SwerveSubsystem.java" file.
*/

public class SwerveModule {
    
    private final SparkMax driveMotor; //Defineing motor controller for the drive motor
    private final SparkMax turningMotor; //Defineing motor controller for the turn motor 

    //private final RelativeEncoder driveEncoder; //Constructing the relative encoder that is built in with every Neo
    public final CANcoder CANabsoluteEncoder; //Constructing the CTRE AbsoluteEncoder that comes with every SparkMaxModule

    private final PIDController turningPidController; //Constructing the PID Controller that will allow us to tell the wheel where to spin too

    public SwerveModule(int driveMotorId, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed, int encoderId)
    {

        CANabsoluteEncoder = new CANcoder(encoderId); //Giving the AbsoluteEncoder a CANid

        //Giving the motor a CANid and making it a brushless motor
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);

        //Configuring Motor Controllers------------------------------------------------------------------
        //Drive
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig
            .inverted(driveMotorReversed)
            .idleMode(IdleMode.kCoast);
            //Set Current Limit
        driveConfig.smartCurrentLimit(40);
            //Converting the encoder values from "rotations" to meters
        driveConfig.encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveConfig.encoder.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //Turn
        SparkMaxConfig turnConfig = new SparkMaxConfig();
        turnConfig.inverted(turningMotorReversed); 
        turnConfig.idleMode(IdleMode.kCoast);
        turnConfig.smartCurrentLimit(40);
        turningMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//-----------------------------------------------------------------------
        //Setting the variable to a value given from the drive motor encoder (Allowing us to know where we are on the field)
        //driveEncoder = driveMotor.getAlternateEncoder(); //   .getAlternateEncoder();
        
        turningPidController = new PIDController(0.007, 0,0); //Creating PID controller for turning

        turningPidController.enableContinuousInput(-180, 180); //Making sure that only values from 0-180 are allowed

        //CANabsoluteEncoder.configAbsoluteSensorRange(CANabsoluteEncoder.configGetAbsoluteSensorRange()); //Grabbing the configs from the pheonix tuner
        //CANabsoluteEncoder.getConfigurator();
     /* 
        var CANcoderConfig = new CANcoderConfiguration();
        
        MagnetSensorConfigs magnetCfg = new MagnetSensorConfigs();
        magnetCfg.withAbsoluteSensorDiscontinuityPoint(0.5);
        magnetCfg.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
       // magnetCfg.withMagnetOffset();
        
        CANcoderConfig.withMagnetSensor(magnetCfg);
        // /* User can change the configs if they want, or leave it empty for factory-default */
        //CANabsoluteEncoder.getConfigurator().apply(CANcoderConfig);
    }

    public double getDrivePosition() 
    {
        return driveMotor.getEncoder().getPosition(); //Get encoder value
    }

    public double getTurningPosition() 
    {
        //System.out.println("getTurningPosition: " + CANabsoluteEncoder.getAbsolutePosition().getValueAsDouble());
        return CANabsoluteEncoder.getAbsolutePosition().getValueAsDouble()*360; //Get encoder value
    }
    

    public double getDriveVelocity() 
    {
        return driveMotor.getEncoder().getVelocity();
        // TESTINGGGGG //Get velocity based off of encoders (REV does this for us)
    }

    public void resetEncoders() {

        driveMotor.getEncoder().setPosition(0);   //setPosition(0);
        CANabsoluteEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {

        //Gets the state of the module (Speed and Angle of the module) based off of the parameters below, FRC libraries do this for us
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()*(Math.PI/180)));   
    }
    
    public SwerveModulePosition getPosition() {

        //Gets the state of the module (Position (meters) and Angle of the module) based off of the parameters below, FRC libraries do this for us
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()*(Math.PI/180)));
    }

    public void setDesiredState(SwerveModuleState state) 
    {
        //Setting where we want each module to be based off the parameter
        //state = SwerveModuleState.optimize(state, getState().angle);
        state.optimize(getState().angle);
        
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        driveMotor.set(state.speedMetersPerSecond/DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getDegrees()));
    }

    public void setSpeedTurn(double speed)
    {
        turningMotor.set(speed); //Setting Speed
    }

    public void setSpeedDrive(double speed)
    {
        driveMotor.set(speed); //Setting Speed
    }

    public void stop() 
    {
        driveMotor.set(0); 
        turningMotor.set(0);
    }

    public void setToAngle(double angle)
    {
        turningMotor.set(turningPidController.calculate(getTurningPosition(), angle)); //Setting angle based off where we are and parameter
    }



}
