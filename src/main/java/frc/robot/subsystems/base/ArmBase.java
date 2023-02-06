package frc.robot.subsystems.base;

import frc.fridowpi.module.Module;
import frc.robot.ArmCalculator;
import frc.robot.ArmCalculator.Cargo;

public class ArmBase extends Module {
    public double baseAngle() {
        return Math.PI / 2.0;
    }

    public double jointAngle() {
        return 0.0;
    }
    
    public ArmCalculator getCalculator() {
        return new ArmCalculator(new ArmCalculator.ArmState(new ArmCalculator.AnglesSupplier(this::baseAngle, this::jointAngle), ArmCalculator.Cargo.None));
    }

    public void setBaseAmpsLimit(double amps) {
        
    }
    
    public void setPercentBase(double percent) {

    }
    
    public void resetEncodersBase() {
        
    }

}
