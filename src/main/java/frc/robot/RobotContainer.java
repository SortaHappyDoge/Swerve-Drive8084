// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*; 

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.deser.DataFormatReaders.Match;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.*;
/*import frc.robot.commands.ArmCmd;
import frc.robot.commands.intakeCMD;
import frc.robot.commands.shootCMD;*/
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final SensorSubsystem sensorSubsystem = new SensorSubsystem();
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public static final CommandXboxController joystick = new CommandXboxController(0);
    public static final XboxController auton_joystick = new XboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //
    public static PathPlannerPath ID12SOURCE;
    public static PathPlannerPath ID13SOURCE;
    public static PathPlannerPath ID17SCORE_R;
    public static PathPlannerPath ID17SCORE_L;
    public static PathPlannerPath ID18SCORE_R;
    public static PathPlannerPath ID18SCORE_L;
    public static PathPlannerPath ID19SCORE_R;
    public static PathPlannerPath ID19SCORE_L;
    public static PathPlannerPath ID20SCORE_R;
    public static PathPlannerPath ID20SCORE_L;
    public static PathPlannerPath ID21SCORE_R;
    public static PathPlannerPath ID21SCORE_L;
    public static PathPlannerPath ID22SCORE_R;
    public static PathPlannerPath ID22SCORE_L;
    //

    SendableChooser<Pose2d> pose_chooser = new SendableChooser<>();
    public static final Pose2d[] kStartingPoses = {
      new Pose2d(7.576,6.5,Rotation2d.fromDegrees(-180)),     // Start 0
      new Pose2d(7.576,4,Rotation2d.fromDegrees(-180)),     // Start 1
      new Pose2d(7.576,1.5,Rotation2d.fromDegrees(-180)),     // Start 2

      new Pose2d(9.964,1.5,Rotation2d.fromDegrees(0)),// Start 3
      new Pose2d(9.964,4,Rotation2d.fromDegrees(0)),// Start 4
      new Pose2d(9.964,6.5,Rotation2d.fromDegrees(0)) // Start 5
    };


    public RobotContainer() {
        drivetrain.configureAutoBuilder();
        try {
            ID12SOURCE = PathPlannerPath.fromPathFile("ID 12 SOURCE");
            ID13SOURCE = PathPlannerPath.fromPathFile("ID 13 SOURCE");
            ID17SCORE_R = PathPlannerPath.fromPathFile("ID 17 SCORE R");
            ID17SCORE_L = PathPlannerPath.fromPathFile("ID 17 SCORE L");
            ID18SCORE_R = PathPlannerPath.fromPathFile("ID 18 SCORE R");
            ID18SCORE_L = PathPlannerPath.fromPathFile("ID 18 SCORE L");
            ID19SCORE_R = PathPlannerPath.fromPathFile("ID 19 SCORE R");
            ID19SCORE_L = PathPlannerPath.fromPathFile("ID 19 SCORE L");
            ID20SCORE_R = PathPlannerPath.fromPathFile("ID 20 SCORE R");
            ID20SCORE_L = PathPlannerPath.fromPathFile("ID 20 SCORE L");
            ID21SCORE_R = PathPlannerPath.fromPathFile("ID 21 SCORE R");
            ID21SCORE_L = PathPlannerPath.fromPathFile("ID 21 SCORE L");
            ID22SCORE_R = PathPlannerPath.fromPathFile("ID 22 SCORE R");
            ID22SCORE_L = PathPlannerPath.fromPathFile("ID 22 SCORE L");

        } catch (Exception e) {
            // TODO: handle exception
        }
        new EventTrigger("autonReadyElevator").onTrue(raiseElevatorAuton());


        configureBindings();
        SmartDashboard.putData(drivetrain.m_field);
        /*PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            m_field.setRobotPose(pose);
        });*/

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            drivetrain.m_field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            drivetrain.m_field.getObject("path").setPoses(poses);
        });


        pose_chooser.setDefaultOption("Blue Middle", kStartingPoses[1]);
        pose_chooser.addOption("Blue Left", kStartingPoses[0]);
        pose_chooser.addOption("Blue Middle", kStartingPoses[1]);
        pose_chooser.addOption("Blue Right", kStartingPoses[2]);
        pose_chooser.addOption("Red Left", kStartingPoses[3]);
        pose_chooser.addOption("Red Middle", kStartingPoses[4]);
        pose_chooser.addOption("Red Right", kStartingPoses[5]);
        SmartDashboard.putData(pose_chooser);

        PathfindingCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //joystick.a().whileTrue(new ArmCmd(elevatorSubsystem, -0.3));
        //joystick.y().whileTrue(new ArmCmd(elevatorSubsystem, 0.3));

        /*joystick.y().whileTrue(new shootCMD(sensorSubsystem, intakeSubsystem, shooterSubsystem));
        joystick.a().whileTrue(new ArmCmd(elevatorSubsystem, 0.3));*/

        joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));
        //joystick.b().onTrue(new ParallelCommandGroup(new intakeCMD(sensorSubsystem, intakeSubsystem, shooterSubsystem),elevatorSubsystem.run(()-> elevatorSubsystem.elevatorSet(0.05))));

        joystick.povRight().onTrue(elevatorSubsystem.run(()->elevatorSubsystem.elevateToReef(1)));
        joystick.povDown().onTrue(elevatorSubsystem.run(()->elevatorSubsystem.elevateToReef(0)));
        joystick.povLeft().onTrue(elevatorSubsystem.run(()->elevatorSubsystem.elevateToReef(2)));
        joystick.povUp().onTrue(elevatorSubsystem.run(()->elevatorSubsystem.elevateToReef(3)));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        /*new POVButton(auton_joystick, 0).onTrue(new InstantCommand(() -> elevatorSubsystem.elevateToReef(3)));
        new POVButton(auton_joystick, 270).onTrue(new InstantCommand(() -> elevatorSubsystem.elevateToReef(2)));
        new POVButton(auton_joystick, 90).onTrue(new InstantCommand(() -> elevatorSubsystem.elevateToReef(1)));
        new POVButton(auton_joystick, 180).onTrue(new InstantCommand(() -> elevatorSubsystem.elevateToReef(0)));*/

        new JoystickButton(auton_joystick, 2)
        .onTrue(new InstantCommand(() -> shooterSubsystem.setMotor(0.2)))
        .onFalse(new InstantCommand(() -> shooterSubsystem.setMotor(0)));
        new JoystickButton(auton_joystick, 1).onTrue(loadCoral());

        new JoystickButton(auton_joystick, SAG_BUTON)
        .onTrue(pathfindToReef(12, true, true).andThen(loadCoral()));
        new JoystickButton(auton_joystick, SOL_BUTON)
        .onTrue(pathfindToReef(13, true, true).andThen(loadCoral()));
    }
    
    
    public static Command selectReef(boolean isRight, int POV){
        boolean isBlue;
        if(DriverStation.getAlliance().get() == Alliance.Red)
        isBlue = false;
        else 
        isBlue = true;
                
        if(isBlue)
        switch (POV) {
            case 45:
                return pathfindToReef(22, isRight, isBlue);
            case 135:
                return pathfindToReef(17, isRight, isBlue);
            case 180:
                return pathfindToReef(18, isRight, isBlue);
            case 225:
                return pathfindToReef(19, isRight, isBlue);
            case 315:
                return pathfindToReef(20, isRight, isBlue);
            case 0:
                return pathfindToReef(21, isRight, isBlue);
            case 90:
                CommandScheduler.getInstance().cancelAll();
                break;
            case 270:
                CommandScheduler.getInstance().cancelAll();
                break;
        }
        else
        switch (POV) {
            case 45:
                return pathfindToReef(6, isRight, isBlue);
            case 135:
                return pathfindToReef(11, isRight, isBlue);
            case 180:
                return pathfindToReef(10, isRight, isBlue);
            case 225:
                return pathfindToReef(9, isRight, isBlue);
            case 315:
                return pathfindToReef(8, isRight, isBlue);
            case 0:
                return pathfindToReef(7, isRight, isBlue);
            case 90:
                CommandScheduler.getInstance().cancelAll();
                break;
            case 270:
                CommandScheduler.getInstance().cancelAll();
                break;
        }
        return null;
    }
    public static Command pathfindToReef(int id, boolean isRight, boolean isBlue){
        if(!isRight && isBlue)
        switch (id) {
            case 12:
                return AutoBuilder.pathfindThenFollowPath(ID12SOURCE, TunerConstants.pathfindingConstraints);
            case 13:
                return AutoBuilder.pathfindThenFollowPath(ID13SOURCE, TunerConstants.pathfindingConstraints);
            case 17:
                return AutoBuilder.pathfindThenFollowPath(ID17SCORE_L, TunerConstants.pathfindingConstraints);
            case 18:
                return AutoBuilder.pathfindThenFollowPath(ID18SCORE_L, TunerConstants.pathfindingConstraints);                
            case 19:
                return AutoBuilder.pathfindThenFollowPath(ID19SCORE_L, TunerConstants.pathfindingConstraints);
            case 20:
                return AutoBuilder.pathfindThenFollowPath(ID20SCORE_L, TunerConstants.pathfindingConstraints);
            case 21:
                return AutoBuilder.pathfindThenFollowPath(ID21SCORE_L, TunerConstants.pathfindingConstraints);
            case 22:
                return AutoBuilder.pathfindThenFollowPath(ID22SCORE_L, TunerConstants.pathfindingConstraints);   
        }
        else if(isRight && isBlue)
        switch (id) {
            case 12:
                return AutoBuilder.pathfindThenFollowPath(ID12SOURCE, TunerConstants.pathfindingConstraints);
            case 13:
                return AutoBuilder.pathfindThenFollowPath(ID13SOURCE, TunerConstants.pathfindingConstraints);
            case 17:
                return AutoBuilder.pathfindThenFollowPath(ID17SCORE_R, TunerConstants.pathfindingConstraints);
            case 18:
                return AutoBuilder.pathfindThenFollowPath(ID18SCORE_R, TunerConstants.pathfindingConstraints);                
            case 19:
                return AutoBuilder.pathfindThenFollowPath(ID19SCORE_R, TunerConstants.pathfindingConstraints);
            case 20:
                return AutoBuilder.pathfindThenFollowPath(ID20SCORE_R, TunerConstants.pathfindingConstraints);
            case 21:
                return AutoBuilder.pathfindThenFollowPath(ID21SCORE_R, TunerConstants.pathfindingConstraints);
            case 22:
                return AutoBuilder.pathfindThenFollowPath(ID22SCORE_R, TunerConstants.pathfindingConstraints);            
        }
        if(!isRight && !isBlue)
        switch (id) {
            case 11:
                return AutoBuilder.pathfindThenFollowPath(ID17SCORE_L, TunerConstants.pathfindingConstraints);
            case 10:
                return AutoBuilder.pathfindThenFollowPath(ID18SCORE_L, TunerConstants.pathfindingConstraints);                
            case 9:
                return AutoBuilder.pathfindThenFollowPath(ID19SCORE_L, TunerConstants.pathfindingConstraints);
            case 8:
                return AutoBuilder.pathfindThenFollowPath(ID20SCORE_L, TunerConstants.pathfindingConstraints);
            case 7:
                return AutoBuilder.pathfindThenFollowPath(ID21SCORE_L, TunerConstants.pathfindingConstraints);
            case 6:
                return AutoBuilder.pathfindThenFollowPath(ID22SCORE_L, TunerConstants.pathfindingConstraints);
        }
        else if(isRight && !isBlue)
        switch (id) {
            case 11:
                return AutoBuilder.pathfindThenFollowPath(ID17SCORE_R, TunerConstants.pathfindingConstraints);
            case 10:
                return AutoBuilder.pathfindThenFollowPath(ID18SCORE_R, TunerConstants.pathfindingConstraints);                
            case 9:
                return AutoBuilder.pathfindThenFollowPath(ID19SCORE_R, TunerConstants.pathfindingConstraints);
            case 8:
                return AutoBuilder.pathfindThenFollowPath(ID20SCORE_R, TunerConstants.pathfindingConstraints);
            case 7:
                return AutoBuilder.pathfindThenFollowPath(ID21SCORE_R, TunerConstants.pathfindingConstraints);
            case 6:
                return AutoBuilder.pathfindThenFollowPath(ID22SCORE_R, TunerConstants.pathfindingConstraints);
        }
        System.out.println("is null for some god forsaken reason ????");
        return null;
    }

    public Command raiseElevatorAuton(){
        if(!DriverStation.isAutonomous()){
            return new WaitCommand(0);
        }
        return new WaitCommand(0)
        .andThen(new InstantCommand(() -> elevatorSubsystem.elevateToReef(3)));
    }
    public Command scoreCoral(int level){
        FunctionalCommand cmd = new FunctionalCommand(
            () -> {elevatorSubsystem.elevateToReef(level);}, 
            () -> {}, 
            interrupted -> {}, 
            () -> {return elevatorSubsystem.getElevatorPosition() > elevatorSubsystem.kReefHeights[level] - 0.06;}, 
            elevatorSubsystem);
        Command cmd2 = new InstantCommand(() -> shooterSubsystem.setMotor(0.2));
        Command cmd3 = new WaitCommand(0.3);
        Command cmd4 = new InstantCommand(() -> shooterSubsystem.setMotor(0));
        Command cmd5 = new InstantCommand(() -> elevatorSubsystem.elevateToReef(0));
        return cmd.andThen(cmd2).andThen(cmd3).andThen(cmd4).andThen(cmd5);
    }
    public Command loadCoral(){
        FunctionalCommand cmd = new FunctionalCommand(
            () -> {
                shooterSubsystem.setMotor(0.13);
                intakeSubsystem.setMotor(-0.5);
            },
            () -> {}, 
            interrupted -> {
                shooterSubsystem.setMotor(0);
                intakeSubsystem.setMotor(0);
            }, 
            () -> {return sensorSubsystem.getState();},
            intakeSubsystem);
        return cmd;
    }

    public Command getAutonomousCommand() {
        //return Commands.print("No autonomous command configured");

        // Blue Alliance Middle Starting Pose Auton
        if(pose_chooser.getSelected() == kStartingPoses[1]){
            return new WaitCommand(0)
            .andThen(new InstantCommand(() -> elevatorSubsystem.elevateToReef(1)))
            .andThen(pathfindToReef(21, false, true))
            .andThen(scoreCoral(3))
            ;
        }

        // Blue Alliance Left Starting Pose Auton
        else if(pose_chooser.getSelected() == kStartingPoses[0]){
            return new WaitCommand(0)
                .andThen(new InstantCommand(() -> elevatorSubsystem.elevateToReef(1)))
                .andThen(pathfindToReef(20, true, true))
                .andThen(scoreCoral(3))
                .andThen(pathfindToReef(13, true, true))
                .andThen(loadCoral())
                .andThen(new InstantCommand(() -> elevatorSubsystem.elevateToReef(2)))
                .andThen(pathfindToReef(19, true, true))
                .andThen(scoreCoral(3))
                .andThen(pathfindToReef(13, true, true))
                .andThen(loadCoral())
                .andThen(new InstantCommand(() -> elevatorSubsystem.elevateToReef(2)))
                .andThen(pathfindToReef(19, false, true))
                .andThen(scoreCoral(3))
            ;
        }

        // Blue Alliance Right Starting Pose Auton
        else if(pose_chooser.getSelected() == kStartingPoses[2]){
            return new WaitCommand(0)
                .andThen(new InstantCommand(() -> elevatorSubsystem.elevateToReef(1)))
                .andThen(pathfindToReef(22, false, true))
                .andThen(scoreCoral(3))
                .andThen(pathfindToReef(12, true, true))
                .andThen(loadCoral())
                .andThen(new InstantCommand(() -> elevatorSubsystem.elevateToReef(2)))
                .andThen(pathfindToReef(17, true, true))
                .andThen(scoreCoral(3))
                .andThen(pathfindToReef(12, true, true))
                .andThen(loadCoral())
                .andThen(new InstantCommand(() -> elevatorSubsystem.elevateToReef(2)))
                .andThen(pathfindToReef(17, false, true))
                .andThen(scoreCoral(3))
            ;
        }

        // Red Alliance Right Starting Pose Auton
        else if(pose_chooser.getSelected() == kStartingPoses[3]){
            return new WaitCommand(0)
                .andThen(new InstantCommand(() -> elevatorSubsystem.elevateToReef(1)))
                .andThen(pathfindToReef(20, true, true))
                .andThen(scoreCoral(3))
                .andThen(pathfindToReef(13, true, true))
                .andThen(loadCoral())
                .andThen(new InstantCommand(() -> elevatorSubsystem.elevateToReef(2)))
                .andThen(pathfindToReef(19, true, true))
                .andThen(scoreCoral(3))
                .andThen(pathfindToReef(13, true, true))
                .andThen(loadCoral())
                .andThen(new InstantCommand(() -> elevatorSubsystem.elevateToReef(2)))
                .andThen(pathfindToReef(19, false, true))
                .andThen(scoreCoral(3))
            ;
        }

        // Red Alliance Right Starting Pose Auton
        else if(pose_chooser.getSelected() == kStartingPoses[4]){
            return new WaitCommand(0)
            .andThen(new InstantCommand(() -> elevatorSubsystem.elevateToReef(1)))
            .andThen(pathfindToReef(21, false, true))
            .andThen(scoreCoral(3))
            ;
        }

        // Red Alliance Right Starting Pose Auton
        else{
            return new WaitCommand(0)
                .andThen(new InstantCommand(() -> elevatorSubsystem.elevateToReef(1)))
                .andThen(pathfindToReef(22, false, true))
                .andThen(scoreCoral(3))
                .andThen(pathfindToReef(12, true, true))
                .andThen(loadCoral())
                .andThen(new InstantCommand(() -> elevatorSubsystem.elevateToReef(2)))
                .andThen(pathfindToReef(17, true, true))
                .andThen(scoreCoral(3))
                .andThen(pathfindToReef(12, true, true))
                .andThen(loadCoral())
                .andThen(new InstantCommand(() -> elevatorSubsystem.elevateToReef(2)))
                .andThen(pathfindToReef(17, false, true))
                .andThen(scoreCoral(3))
            ;
        }
    }
}


