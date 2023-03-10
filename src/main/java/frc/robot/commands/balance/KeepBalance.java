package frc.robot.commands.balance;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class KeepBalance extends CommandBase {
    private final PIDController m_pidController = new PIDController(0.05, 0, 0.001);
    private double m_pitch;
    private double m_vel;
    private static GenericEntry pitchEntry;

    public KeepBalance() {
        addRequirements(Drive.getInstance());
        m_pidController.setTolerance(2);
        var tab = Shuffleboard.getTab("pid values");
        if (pitchEntry == null)
            pitchEntry = tab.add("bitch", 0).getEntry();
    }
    
    double previousPitch;
    double previousVel;

    @Override
    public void initialize() {
        previousPitch = Drive.getInstance().getPitch();
        previousVel = 0;
    }

    @Override
    public void execute() {
        m_pitch = Drive.getInstance().getPitch();
        pitchEntry.setDouble(m_pitch);

        if (m_pitch < 3) {
            Drive.getInstance().stopAndBreak();
            return;
        }

        m_vel = m_pidController.calculate(m_pitch);
        m_vel = MathUtil.clamp(m_vel, -0.1, 0.1);
        Drive.getInstance().setMotorPercent(m_vel, m_vel);
    }

    @Override
    public void end(boolean interrupted) {
        Drive.getInstance().releaseBrake();
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
