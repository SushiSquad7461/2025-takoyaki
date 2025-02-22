package frc.robot.util.control.nt;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Class for a tunable number. Gets value from NetworkTables in tuning mode, returns
 * default if not or value not in NetworkTables.
 */
public class TunableNumber {
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("TunableNumbers");
    
    private final DoublePublisher publisher;
    private final DoubleSubscriber subscriber;
    private double defaultValue;
    private double lastHasChangedValue = defaultValue;
    private final boolean tuningMode;

    /**
     * Create a new TunableNumber
     * 
     * @param key Key in NetworkTables
     * @param defaultValue Default value to use if not in tuning mode
     * @param tuningMode Whether to use NetworkTables value (true) or default value (false)
     */
    public TunableNumber(String key, double defaultValue, boolean tuningMode) {
        this.publisher = table.getDoubleTopic(key).publish();
        this.subscriber = table.getDoubleTopic(key).subscribe(defaultValue);
        this.tuningMode = tuningMode;
        setDefault(defaultValue);
    }

    /**
     * Get the default value for the number that has been set
     * 
     * @return The default value
     */
    public double getDefault() {
        return defaultValue;
    }

    /**
     * Set the default value of the number
     * 
     * @param defaultValue The default value
     */
    public void setDefault(double defaultValue) {
        this.defaultValue = defaultValue;
        this.lastHasChangedValue = defaultValue;
        
        if (tuningMode) {
            // Only publish if we're in tuning mode and there's no existing value
            if (subscriber.get() == this.defaultValue) {
                publisher.set(defaultValue);
            }
        } else {
            // If not in tuning mode, close the publisher
            publisher.close();
        }
    }

    /**
     * Get the current value, from NetworkTables if available and in tuning mode
     * 
     * @return The current value
     */
    public double get() {
        return tuningMode ? subscriber.get() : defaultValue;
    }

    /**
     * Checks whether the number has changed since our last check
     * 
     * @return True if the number has changed since the last time this method was
     *         called, false otherwise
     */
    public boolean hasChanged() {
        double currentValue = get();
        if (currentValue != lastHasChangedValue) {
            lastHasChangedValue = currentValue;
            return true;
        }
        return false;
    }

    /**
     * Clean up NetworkTables publishers and subscribers
     */
    public void close() {
        publisher.close();
        subscriber.close();
    }
}