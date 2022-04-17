// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  // Network table
  private final NetworkTable networkTable;

  // Array Structure
  // 0: tx        6: getpipe
  // 1: ty        7: ledMode
  // 2: tv        8: camMode
  // 3: ta        9: pipeline
  // 4: ts        10: stream
  // 5: tl
  private final ArrayList<NetworkTableEntry> tableEntries;
  private int streamNum;

  /** Creates a new Limelight. */
  public Limelight() {
    // Table
    networkTable = NetworkTableInstance.getDefault().getTable("limelight");

    // Data
    tableEntries = new ArrayList<NetworkTableEntry>();
    tableEntries.add(networkTable.getEntry("tx"));
    tableEntries.add(networkTable.getEntry("ty"));
    tableEntries.add(networkTable.getEntry("tv"));
    tableEntries.add(networkTable.getEntry("ta"));
    tableEntries.add(networkTable.getEntry("ts"));
    tableEntries.add(networkTable.getEntry("tl"));
    tableEntries.add(networkTable.getEntry("getpipe"));

    // Control
    tableEntries.add(networkTable.getEntry("ledMode"));
    tableEntries.add(networkTable.getEntry("camMode"));
    tableEntries.add(networkTable.getEntry("pipeline"));
    tableEntries.add(networkTable.getEntry("stream"));
  }

  public boolean isTargetToLeft() {
    return getTargetXOffset() >= LimelightConstants.kAngleThreshold;
  }

  public boolean isTargetToRight() {
    return getTargetXOffset() <= LimelightConstants.kAngleThreshold;
  }

  public boolean isTargetCentered() {
    return !isTargetToLeft() && !isTargetToRight();
  }

  public double getTargetXOffset() {
    return tableEntries.get(0).getDouble(0);
  }

  public double getTargetYOffset() {
    return tableEntries.get(1).getDouble(0);
  }

  public boolean areValidTargets() {
    return tableEntries.get(2).getDouble(0) == 1;
  }

  public double getTargetArea() {
    return tableEntries.get(3).getDouble(0);
  }

  public double getTargetSkew() {
    return tableEntries.get(4).getDouble(0);
  }

  public double getPipelineLatency() {
    // returns in ms; add 11ms for image capture latency
    return tableEntries.get(5).getDouble(0);
  }

  public Number getActivePipeline() {
    return tableEntries.get(6).getNumber(0);
  }

  public void setLedsOn() {
    tableEntries.get(7).setNumber(3);
  }

  public void setLedsOff() {
    tableEntries.get(7).setNumber(1);
  }

  public void blinkLeds() {
    tableEntries.get(7).setNumber(2);
  }

  public void setToVisionView() {
    tableEntries.get(8).setNumber(0);
  }

  public void setToDriverView() {
    tableEntries.get(8).setNumber(1);
  }

  public void setPipeline(Number pipelineNum) {
    tableEntries.get(9).setNumber(pipelineNum);
  }

  public void setStreamingMode(Number streamNum) {
    tableEntries.get(10).setNumber(streamNum);
    this.streamNum = streamNum.intValue();
  }

  public int getStreamNum() {
    return streamNum;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("IsPipelineRed", getActivePipeline().doubleValue() == 0.0);
  }
}
