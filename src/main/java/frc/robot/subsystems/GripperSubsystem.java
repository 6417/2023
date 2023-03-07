package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.fridowpi.joystick.Binding;
import frc.fridowpi.pneumatics.FridoDoubleSolenoid;
import frc.robot.Constants;
import frc.robot.commands.GripperCommand;
import frc.robot.subsystems.base.GripperSubsystemBase;
import java.util.List;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class GripperSubsystem extends GripperSubsystemBase {

  private FridoDoubleSolenoid gripperSolenoid;
  private static GripperSubsystemBase instance;

  public GripperSubsystem() {
    gripperSolenoid =
      new FridoDoubleSolenoid(
        Constants.Gripper.DoubleSolenoid.idLower,
        Constants.Gripper.DoubleSolenoid.idHigher
      );
  }

  @Override
  public void init() {
    // TODO Auto-generated method stub
    gripperSolenoid.init();
    super.init();
  }

  public List<Binding> getMappings() {
    return List.of(
      new Binding(
        Constants.Joysticks.drive,
        Constants.Gripper.ButtonIds.openClose,
        Button::toggleOnTrue,
        new GripperCommand()
      )
    );
  }

  @Override
  public void openGripper() {
    gripperSolenoid.set(Constants.Gripper.DoubleSolenoid.gripperOpen);
  }

  @Override
  public void closeGripper() {
    gripperSolenoid.set(Constants.Gripper.DoubleSolenoid.gripperClosed);
  }

  private static Logger logger = LogManager.getLogger(GripperSubsystem.class);

  @Override
  public boolean isGripperOpen() {
    Value gripperState = gripperSolenoid.get();

    if (gripperState == Constants.Gripper.DoubleSolenoid.gripperOpen) {
      return true;
    } else if (gripperState == Constants.Gripper.DoubleSolenoid.gripperClosed) {
      return false;
    } else {
      logger.warn("DoubleSolenoid on Gripper is kOff");
      return true;
    }
  }

  public static GripperSubsystemBase getInstance() {
    if (instance == null) {
      if (Constants.Gripper.enabled) {
        instance = new GripperSubsystem();
      } else {
        instance = new GripperSubsystemBase();
      }
    }
    return instance;
  }
}
