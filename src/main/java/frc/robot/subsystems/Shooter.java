package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
    private final WPI_TalonFX followerMotor, masterMotor;

    private final PIDController pidController;
    private final SimpleMotorFeedforward feedForward;

    private double speed;
    private double thresh;

    public Shooter() {
        // Initialize motors
        masterMotor = new WPI_TalonFX(ShooterConstants.kFollowerMotorID);
        followerMotor = new WPI_TalonFX(ShooterConstants.kMasterMotorID);
        masterMotor.configFactoryDefault();
        followerMotor.configFactoryDefault();
        masterMotor.setInverted(ShooterConstants.kFrontShooterMotorInverted);
        followerMotor.setInverted(!ShooterConstants.kFrontShooterMotorInverted);
        followerMotor.follow(masterMotor);

        // Setup controllers for optimized shooting
        pidController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
        feedForward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);

        // Set default speeds
        speed = ShooterConstants.kHighGoalSpeed;
        thresh = ShooterConstants.kHighGoalThresh;
    }

    public void shoot() {
        masterMotor.setVoltage(feedForward.calculate(speed) + pidController.calculate(getVelocity(), speed));
    }

    public void stop() {
        masterMotor.set(0);
    }

    public void setToHighGoal() {
        speed = ShooterConstants.kHighGoalSpeed;
        thresh = ShooterConstants.kHighGoalThresh;
    }

    public void setToLowGoal() {
        speed = ShooterConstants.kLowGoalSpeed;
        thresh = ShooterConstants.kLowGoalThresh;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getSpeedValue() {
        return speed;
    }

    public double getThreshValue() {
        return thresh;
    }

    // Formatting here is weird, I apologize...
    // This rescales the velocity from the motors to m/s with gear ratio and diameter taken into account
    public double getVelocity() {
        return
            (double) masterMotor.getSelectedSensorVelocity() / ShooterConstants.kEncoderResolution /
            ShooterConstants.kGearRatio * 10 * Math.PI * Units.inchesToMeters(ShooterConstants.kWheelDiameter);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("RawMasterShooterVelocity", masterMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber(("RawFollowerShooterVelocity"), followerMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("ShooterVelocityInM/S", getVelocity());
    }
}
