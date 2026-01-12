package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double flywheelSetpointRPS = 0.0;
  private double hoodSetpointDeg = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/FlywheelSetpointRPM", flywheelSetpointRPS);
    Logger.recordOutput("Shooter/HoodSetpointDeg", hoodSetpointDeg);
  }

  /** Sets shooter flywheel velocity (RPM). */
  public void setVelocity(double rps) {
    flywheelSetpointRPS = rps;
    io.setFlywheelVelocity(rps);
  }

  /** Sets hood/backplate angle (degrees). */
  public void setHoodAngle(double deg) {
    hoodSetpointDeg = deg;
    io.setHoodAngleDeg(deg);
  }

  public void stop() {
    flywheelSetpointRPS = 0.0;
    io.stop();
  }
}
