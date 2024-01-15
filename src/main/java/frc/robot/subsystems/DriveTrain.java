// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Drive;
// import edu.wpi.first.wpilibj.drive.MecanumDrive;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
// import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class DriveTrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

    private final static DriveTrain instance = DriveTrain.getInstance();

    public final CANSparkMax rFMotor = new CANSparkMax(Constants.MotorPorts.rFMotorPort, CANSparkLowLevel.MotorType.kBrushless);
    public final CANSparkMax rRMotor = new CANSparkMax(Constants.MotorPorts.rRMotorPort, CANSparkLowLevel.MotorType.kBrushless);
    public final CANSparkMax lFMotor = new CANSparkMax(Constants.MotorPorts.lFMotorPort, CANSparkLowLevel.MotorType.kBrushless);
    public final CANSparkMax lRMotor = new CANSparkMax(Constants.MotorPorts.lRMotorPort, CANSparkLowLevel.MotorType.kBrushless);
    public final MecanumDrive mecanum = new MecanumDrive(lFMotor, lRMotor, rFMotor, rRMotor);

    DigitalInput rFLimit = new DigitalInput(0);
    DigitalInput rRLimit = new DigitalInput(1);
    DigitalInput lFLimit = new DigitalInput(2);
    DigitalInput lRLimit = new DigitalInput(3);
  public DriveTrain() {}


  /**
   * Example command factory method.
   *
   * @return a command
   */
  // public Command exampleMethodCommand() {
  //   // Inline construction of command goes here.
  //   // Subsystem::RunOnce implicitly requires `this` subsystem.
  //   return runOnce(
  //       () -> {
  //         /* one-time action goes here */
  //       });
  // }

    void drive(double throttle, double strafe, double rot) {
      mecanum.driveCartesian(throttle, strafe, rot);
    }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public static DriveTrain getInstance(){
    if(instance==null)
      return new DriveTrain();
    return instance;
  }
}

