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

/**
 * Limelight class that consists of a Limelight 2.0.
 * 
 * <p> For future usage, I honestly recommend that a different class is used. There's a template
 * subsystem class for the limelight that I know for a fact exists, but I never bothered using
 * it when I definitely should have. It's not that this class is poorly written or bad, it's
 * just that the one online covered quite literally all the information you can get from the
 * limelight whereas this one I think may be lacking for a few numbers and states here and there.
 * In any case, for the most part, this class provides an easy means of obtaining data reported
 * from the limelight and using that data within code to execute useful actions, e.g. targetting
 * a red or blue power cell that is on the field. It also allows for us to change the camera mode
 * relatively easily between targetting and driving vision, allowing for us to have the limelight
 * double as a high quality camera used for maneuvering the robot around on the field.
 * 
 * <p> I highly recommend you look at the documentation for the Limelight itself if you need any
 * more information on what data is reported from the device and how to do targetting while driving.
 * It's fairly detailed and they provide a ton of sample code for you to get started with, so I
 * definitely recommend that you try and look through all that to maximize the potential of this
 * camera.
 */
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

  /**
   * Gets if the targeted item is to the left of the robot's vision, with some tolerance given
   * in the form of a threshold specified in the {@link frc.robot.Constants} class.
   * @return if the targeted item is to the left
   */
  public boolean isTargetToLeft() {
    return getTargetXOffset() >= LimelightConstants.kAngleThreshold;
  }

  /**
   * Gets if the targeted item is to the right of the robot's vision, with some tolerance given
   * in the form of a threshold specified in the {@link frc.robot.Constants} class.
   * @return if the targeted item is to the right
   */
  public boolean isTargetToRight() {
    return getTargetXOffset() <= LimelightConstants.kAngleThreshold;
  }

  /**
   * Gets if the targetted item is centered with the robot's vision
   * @return if the targeted item is centered
   */
  public boolean isTargetCentered() {
    return !isTargetToLeft() && !isTargetToRight();
  }

  /**
   * Reports the current offset of the object in the x-axis from the limelight's crosshair
   * @return X-Offset of the targeted object
   */
  public double getTargetXOffset() {
    return tableEntries.get(0).getDouble(0);
  }

  /**
   * Reports the current offset of the object in the y-axis from the limelight's crosshair
   * @return Y-Offset of the targeted object
   */
  public double getTargetYOffset() {
    return tableEntries.get(1).getDouble(0);
  }

  /**
   * Reports if there is a valid target being tracked within the robot's vision
   * @return if there is a valid target visible
   */
  public boolean areValidTargets() {
    return tableEntries.get(2).getDouble(0) == 1;
  }

  /**
   * (Double check against Limelight docs) Gets how much the targeted object is
   * "full" within its tracked box
   * @return the area the target is filling up
   */
  public double getTargetArea() {
    return tableEntries.get(3).getDouble(0);
  }

  /**
   * Gets how skewed / rotated the targeted object is
   * @return rotation of the targeted object
   */
  public double getTargetSkew() {
    return tableEntries.get(4).getDouble(0);
  }

  /**
   * Gets the latency of how long it takes for the pipeline to report data back to
   * {@link NetworkTable} and our code
   * @return pipeline processing latency
   */
  public double getPipelineLatency() {
    // returns in ms; add 11ms for image capture latency
    return tableEntries.get(5).getDouble(0);
  }

  /**
   * Gets the current number of the pipeline being used by the limelight
   * @return Index number of the active pipeline
   */
  public Number getActivePipeline() {
    return tableEntries.get(6).getNumber(0);
  }

  /**
   * Sets the limelight leds to be ON
   */
  public void setLedsOn() {
    tableEntries.get(7).setNumber(3);
  }

  /**
   * Sets the limelight leds to be OFF
   */
  public void setLedsOff() {
    tableEntries.get(7).setNumber(1);
  }

  /**
   * Blinks the limelight leds
   */
  public void blinkLeds() {
    tableEntries.get(7).setNumber(2);
  }

  /**
   * Toggle the view of the limelight so that it is able to be used for targeting
   * and computer vision
   */
  public void setToVisionView() {
    tableEntries.get(8).setNumber(0);
  }

  /**
   * Toggle the view of the limelight so it is better optimized for driving and human
   * vision
   */
  public void setToDriverView() {
    tableEntries.get(8).setNumber(1);
  }

  /**
   * Sets the limelight to use a specific pipeline for computer vision
   * @param pipelineNum Index number of the pipeline to use
   */
  public void setPipeline(Number pipelineNum) {
    tableEntries.get(9).setNumber(pipelineNum);
  }

  /**
   * See limelight docs for the different modes; Allows for us to set the streaming mode of
   * the limelight when we have a second webcam plugged into the limelight's USB port
   * @param streamNum Streaming mode to set on the limelight
   */
  public void setStreamingMode(Number streamNum) {
    tableEntries.get(10).setNumber(streamNum);
    this.streamNum = streamNum.intValue();
  }

  /**
   * Gets the current value of the stream mode being used (not the views of the limelight!)
   * @return Current stream mode as a number (reference docs for different modes)
   */
  public int getStreamNum() {
    return streamNum;
  }

  /**
   * Pushes info on which pipeline we are using to ShuffleBoard
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("IsPipelineRed", getActivePipeline().doubleValue() == 0.0);
  }
}
