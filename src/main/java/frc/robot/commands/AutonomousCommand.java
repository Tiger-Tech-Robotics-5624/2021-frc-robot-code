// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AutonomousVisionSubsystem;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;

public class AutonomousCommand extends CommandBase {
  /** Creates a new AutonomusCommand. */
  AutonomousVisionSubsystem vision;
  public AutonomousCommand(AutonomousVisionSubsystem v,DriveSubsystem drive, BeltSubsystem belt, IntakeMotorSubsystem intakeMotor, IntakePistonSubsystem intakePiston) {
    vision = v;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision, drive, belt, intakeMotor, intakePiston);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    vision.update();
    vision.printStuff();
    vision.runPath();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return vision.isComplete();
  }
}
