package frc.fridowpi.initializer;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public interface Initialisable {
    void init();

    boolean isInitialized();

    default void requires(Initialisable initialisable) {
        Initializer.getInstance().after(initialisable, this);
    }
}
