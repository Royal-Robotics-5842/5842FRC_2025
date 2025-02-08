package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSystem extends SubsystemBase {
    public SparkMax elevatorLeft = new SparkMax(32, MotorType.kBrushless);
    public SparkMax elevatorRight = new SparkMax(33, MotorType.kBrushless);
    private SparkClosedLoopController elevatorPid;
    private int stallLimit = 40;

    public ElevatorSystem() {
            SparkMaxConfig leftConfig = new SparkMaxConfig();
            leftConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
                leftConfig.smartCurrentLimit(stallLimit);
        elevatorLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            SparkMaxConfig rightConfig = new SparkMaxConfig();
            rightConfig
                .idleMode(IdleMode.kBrake);
                rightConfig.smartCurrentLimit(stallLimit);
                rightConfig.follow(elevatorLeft, true);
        elevatorRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        elevatorPid = elevatorLeft.getClosedLoopController();

        leftConfig.closedLoop
            .p(0.05)
            .i(0)
            .d(0)
            .outputRange(-1, 1);
    }

    public void run(double speed) {
        elevatorLeft.set(speed);
    }

    public void pidRun(double setpoint) {
        elevatorPid.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        //System.out.println("test");
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Position of elevator", elevatorLeft.getEncoder().getPosition());
    }
}
