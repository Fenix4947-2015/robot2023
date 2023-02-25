  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  package frc.robot.subsystems;

  import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

  import edu.wpi.first.wpilibj.PneumaticsModuleType;
  import edu.wpi.first.wpilibj.Solenoid;
  import edu.wpi.first.wpilibj2.command.CommandBase;
  import edu.wpi.first.wpilibj2.command.SubsystemBase;

  import static edu.wpi.first.wpilibj.PneumaticsModuleType.CTREPCM;

  public class GripperArm extends SubsystemBase {
    private final CANSparkMax m_foreArmLeader = new CANSparkMax(42, MotorType.kBrushed);
    private final CANSparkMax m_foreArmFollower = new CANSparkMax(43, MotorType.kBrushed);

    private final Solenoid m_lockElbow = new Solenoid(CTREPCM, 0);
    private final Solenoid m_verticalArmStage1 = new Solenoid(CTREPCM, 1);
    private final Solenoid m_verticalArmStage2 = new Solenoid(CTREPCM, 2);
    private final Solenoid m_gripper = new Solenoid(CTREPCM, 6);


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
      m_foreArmLeader.set(-0.25);
    }

    public void downForeArm() {
      m_foreArmLeader.set(0.25);
    }

    public void stopForearm() {
      m_foreArmLeader.set(0.0);
    }

    public void lockElbow() {
      m_lockElbow.set(true);
    }

    public void unlockElbow() {
      m_lockElbow.set(false);
    }

    public void enableStage1() {
      m_verticalArmStage1.set(true);
    }

    public void disableStage1() {
      m_verticalArmStage1.set(false);
    }

    public void toggleStage1() {
      m_verticalArmStage1.toggle();
    }

    public void enableStage2() {
      m_verticalArmStage2.set(true);
    }

    public void disableStage2() {
      m_verticalArmStage2.set(false);
    }

    public void toggleStage2() {
      m_verticalArmStage2.toggle();
    }
  }
