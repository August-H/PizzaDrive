// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_drive = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  // Deadband threshold for joystick inputs
  private static final double kDeadband = 0.1;

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
    m_drive.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    // Get raw joystick inputs
    double rawXSpeed = -m_controller.getLeftY(); // Inverted for forward motion
    double rawRot = -m_controller.getRightX();   // Inverted for CCW rotation

    // Apply deadband
    double xSpeed = (Math.abs(rawXSpeed) < kDeadband) ? 0.0 : rawXSpeed;
    double rot = (Math.abs(rawRot) < kDeadband) ? 0.0 : rawRot;

    // Apply slew rate limiting and scaling
    xSpeed = m_speedLimiter.calculate(xSpeed) * Drivetrain.kMaxSpeed;
    rot = m_rotLimiter.calculate(rot) * Drivetrain.kMaxAngularSpeed;

    m_drive.drive(xSpeed, rot);
  }
}