// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.annotation.JsonAppend.Prop;

import edu.wpi.first.hal.simulation.AnalogInDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.LimelightHelpers;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private boolean TagBool;
  private boolean hasStarted = false;

  private final RobotContainer m_robotContainer;
  //private AnalogInput analogInput;
    
      public Robot() {
        m_robotContainer = new RobotContainer();
      }
    
  
    @Override
    public void robotInit(){
    //  analogInput = new AnalogInput(3);
    for (int port = 5800; port <= 5809; port++) { PortForwarder.add(port, "limelight.local", port);
                                                  PortForwarder.add(port, "limelight-alfa.local", port); }

  }
    @Override
    public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    //SmartDashboard.putNumber("ENCODER", ElevatorSubsystem.getEncoder());
    //SmartDashboard.putNumber("SENSOR", analogInput.getVoltage());

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if(!hasStarted){
      m_robotContainer.drivetrain.getPigeon2().setYaw(m_robotContainer.pose_chooser.getSelected().getRotation().getDegrees());
      m_robotContainer.drivetrain.resetPose(m_robotContainer.pose_chooser.getSelected());
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    hasStarted = true;
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if(TagBool){
      TagBool = false;
      if(DriverStation.getAlliance().get() == Alliance.Blue){
         int[] validIDs = {17, 18, 19, 20, 21, 22};
       LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs); 
    } else{ int[] validIDs = {6, 7, 8, 9, 10, 11}; LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs); }}
  }

  @Override
  public void teleopPeriodic() {
    // Driver 1 Pathfinding
    if(RobotContainer.auton_joystick.getRawButtonPressed(5)){
      try {
        Command cmd = RobotContainer.selectReef(false, RobotContainer.auton_joystick.getPOV());
        if(cmd != null)
        cmd.schedule();
      } catch (Exception e) {
        System.out.println(e);
      }
    }
    if(RobotContainer.auton_joystick.getRawButtonPressed(6)){
      try {
        Command cmd = RobotContainer.selectReef(true, RobotContainer.auton_joystick.getPOV());
        if(cmd != null)
        cmd.schedule();
      } catch (Exception e) {
        System.out.println(e);
      }
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

  }

  @Override 
  public void testPeriodic() {
    //ElevatorSubsystem.resetEncoder();

  }

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
  
}
 
