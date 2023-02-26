package frc.robot.subsystems.base;

import frc.fridowpi.module.Module;
import frc.robot.ArmModel;
import frc.robot.ArmModel.Cargo;

public class ArmBase extends Module {
    public double baseAngle() {
        return Math.PI / 2.0;
    }

    public double jointAngle() {
        return 0.0;
    }
    
    public ArmModel getCalculator() {
        return new ArmModel(new ArmModel.ArmStateSupplier(this::baseAngle, this::jointAngle, () -> 0.0, () -> 0.0), Cargo.None);
    }

    public void setBaseAmpsLimit(double amps) {
        
    }
    
    public void setPercentBase(double percent) {

    }
    
    public void resetEncodersBase() {
        
    }

}
