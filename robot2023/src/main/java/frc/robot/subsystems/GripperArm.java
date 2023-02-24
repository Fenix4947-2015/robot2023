  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  package frc.robot.subsystems;

  import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

  import edu.wpi.first.wpilibj2.command.CommandBase;
  import edu.wpi.first.wpilibj2.command.SubsystemBase;

  public class GripperArm extends SubsystemBase {
    private final CANSparkMax m_foreArmLeader = new CANSparkMax(42, MotorType.kBrushed);
    private final CANSparkMax m_foreArmFollower = new CANSparkMax(43, MotorType.kBrushed);


    /** Creates a new GripperArm. */
    public GripperArm() {
      m_foreArmLeader.restoreFactoryDefaults();
      m_foreArmFollower.restoreFactoryDefaults();

      m_foreArmLeader.setIdleMode(IdleMode.kBrake);
      m_foreArmFollower.setIdleMode(IdleMode.kBrake);

      m_foreArmFollower.follow(m_foreArmLeader);
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public CommandBase exampleMethodCommand() {
      // Inline construction of command goes here.
      // Subsystem::RunOnce implicitly requires `this` subsystem.
      return runOnce(
          () -> {
            /* one-time action goes here */
          });
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

    public void upForearm() {
      m_foreArmLeader.set(0.25);
    }

    public void downForeArm() {
      m_foreArmLeader.set(-0.25);
    }

    public void stopForearm() {
      m_foreArmLeader.set(0.0);
    }
  }
