package org.frc5687.pitracker2017;

/**
 * Created by Ben Bernard on 4/15/2017.
 */
public class Stopwatch {
    private String _label;
    private long _start;
    private long _total;

    public Stopwatch(String label) {
        _label = label;
        _start = 0;
        _total = 0;
    }

    public void start() {
        _start = System.currentTimeMillis();
    }

    public void stop() {
        _total += System.currentTimeMillis() - _start;
    }

    public long elapsed() {
        return _total;
    }
}
