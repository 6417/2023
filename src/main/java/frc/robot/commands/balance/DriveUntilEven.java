package frc.robot.commands.balance;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class DriveUntilEven extends CommandBase {
    private final PIDController m_pidController = new PIDController(0.05, 0, 0.001);
    private double m_pitch;
    private double m_vel;
    private static GenericEntry pitchEntry;

    public DriveUntilEven() {
        addRequirements(Drive.getInstance());
        m_pidController.setTolerance(2);
        var tab = Shuffleboard.getTab("pid values");
        if (pitchEntry == null)
            pitchEntry = tab.add("pitch", 0).getEntry();
    }
    
    Timer timer;
    double previousPitch;
    double previousVel;

    @Override
    public void initialize() {
        previousPitch = Drive.getInstance().getPitch();
        previousVel = 0;
        timer = new Timer();
        timer.start();
    }

    @Override
    public void execute() {
        m_pitch = Drive.getInstance().getPitch();
        pitchEntry.setDouble(m_pitch);
        m_vel = m_pidController.calculate(m_pitch);

        m_vel = MathUtil.clamp(m_vel, -0.15, 0.15);
        Drive.getInstance().setMotorPercent(m_vel, m_vel);
    }

    @Override
    public void end(boolean interrupted) {
        Drive.getInstance().stopAndBreak();
    }


    @Override
    public boolean isFinished() {
        double currentPitch = Drive.getInstance().getPitch();

        double angularVel = (currentPitch - previousPitch) / timer.get();
        double angularAccel = (angularVel - previousVel) / timer.get();
        previousVel = angularVel;

        if (Math.abs(Drive.getInstance().getPitch()) < 8) {
            System.out.print("pitch: " + m_pitch + " ");
            System.out.printf("AngularVel: %f, AngularAccel: %f\n", angularVel, angularAccel);
        }
        previousPitch = currentPitch;
        timer.reset();
        timer.start();
        return Math.abs(angularAccel) > 1000 && Math.abs(currentPitch) < 7;
        // return Math.abs(angularVel) > 15 && Math.abs(currentPitch) < 4.5;
    }
}
