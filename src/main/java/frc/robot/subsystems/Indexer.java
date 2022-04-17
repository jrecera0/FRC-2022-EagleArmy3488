package frc.robot.subsystems;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Indexer extends SubsystemBase{
    private final WPI_TalonFX indexer;

    private double speed = IndexerConstants.kIndexerSpeed;

    public Indexer() {
        indexer = new WPI_TalonFX(IndexerConstants.kIndexerMotorID);
        indexer.configFactoryDefault();
        indexer.setInverted(IndexerConstants.kIndexerInverted);
    }

    public void index() {
        indexer.set(speed);
    }

    public void stop() {
        indexer.set(0);
    }

    public void reverse() {
        indexer.set(-speed);
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getSpeedValue() {
        return speed;
    }

    // Returns velocity in rotations/s
    public double getIndexerVelocity() {
        return indexer.getSelectedSensorVelocity() / 2048.0 * 10.0;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("RawIndexerVelocity", indexer.getSelectedSensorVelocity());
        SmartDashboard.putNumber("IndexerVelocityInRPS", getIndexerVelocity());
    }
}
