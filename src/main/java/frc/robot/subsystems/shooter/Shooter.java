package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double flywheelSetpointRPS = 0.0;
  private double flywheelAccelSetpointRpsPerSec = 0.0;
  private double hoodSetpointDeg = 0.0;
  private double hoodVelSetpointDegPerSec = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/FlywheelSetpointRPM", flywheelSetpointRPS);
    Logger.recordOutput("Shooter/FlywheelAccelSetpointRpsPerSec", flywheelAccelSetpointRpsPerSec);
    Logger.recordOutput("Shooter/HoodSetpointDeg", hoodSetpointDeg);
    Logger.recordOutput("Shooter/HoodVelSetpointDegPerSec", hoodVelSetpointDegPerSec);
  }

  /** Sets shooter flywheel velocity (RPM). */
  public void setVelocity(double rps) {

    flywheelSetpointRPS = rps;
    flywheelAccelSetpointRpsPerSec = 0.0;
    io.setFlywheelVelocity(rps);
  }

  /** Sets shooter flywheel velocity with acceleration feedforward (RPS and RPS/s). */
  public void setVelocity(double rps, double accelRpsPerSec) {
    flywheelSetpointRPS = rps;
    flywheelAccelSetpointRpsPerSec = accelRpsPerSec;
    io.setFlywheelVelocity(rps, accelRpsPerSec);
  }

  /** Sets hood/backplate angle (degrees). */
  public void setHoodAngle(double deg) {
    hoodSetpointDeg = deg;
    hoodVelSetpointDegPerSec = 0.0;
    io.setHoodAngleDeg(deg);
  }

  /** Sets hood/backplate angle with velocity feedforward (deg and deg/s). */
  public void setHoodAngle(double deg, double velDegPerSec) {
    hoodSetpointDeg = deg;
    hoodVelSetpointDegPerSec = velDegPerSec;
    io.setHoodAngleDeg(deg, velDegPerSec);
  }

  public void stop() {
    flywheelSetpointRPS = 0.0;
    flywheelAccelSetpointRpsPerSec = 0.0;
    hoodVelSetpointDegPerSec = 0.0;
    io.stop();
  }

  /** Returns flywheel leader velocity (mechanism RPS). */
  public double getFlywheelVelocityRps() {
    return inputs.flywheelLeaderVelocityRotationPerSec;
  }

  /** Returns hood/backplate angle (mechanism degrees). */
  public double getHoodAngleDeg() {
    return inputs.hoodAngleDeg;
  }

  /** Returns whether shooter IO is connected/healthy. */
  public boolean isConnected() {
    return inputs.connected;
  }
}
