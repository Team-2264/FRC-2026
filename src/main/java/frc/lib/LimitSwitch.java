package frc.lib;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch {
    private DigitalInput di;
    private boolean activeHigh;

    public LimitSwitch(int dioIndex, boolean activeHigh) {
        this.di = new DigitalInput(dioIndex);
        this.activeHigh = activeHigh;
    }

    /**
     * Get the current state of the limit switch
     * 
     * @return true if the switch is activated, otherwise false
     */
    public boolean get() {
        boolean di_state = di.get();
        boolean active = di_state;
        
        if (!activeHigh)
        {
            active = !active;
        }

        return active;
    }
}
