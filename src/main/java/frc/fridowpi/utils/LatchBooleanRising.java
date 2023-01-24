package frc.fridowpi.utils;

public class LatchBooleanRising implements LatchedBoolean{
    private boolean previous;
    private boolean currentState = false;
    public LatchBooleanRising(boolean initial) {
        previous = initial;
    }

    @Override
    public boolean get() {
        return currentState;
    }

    @Override
    public void update(boolean val) {
        currentState = !previous && val;
        previous = val;
    }
}
