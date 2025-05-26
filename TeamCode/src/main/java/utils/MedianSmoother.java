package utils;

import java.util.*;

public class MedianSmoother {
    private int windowSize = 500;
    private final Deque<Double> xVals = new ArrayDeque<>();
    private final Deque<Double> yVals = new ArrayDeque<>();
    private final Deque<Double> angleVals = new ArrayDeque<>();

    public MedianSmoother(){

    }
    public MedianSmoother(int windowSize) {
        this.windowSize = windowSize;
    }

    public void setWindowSize(int windowSize) {
        this.windowSize = windowSize;
    }

    public void update(double x, double y, double angleDegrees) {
        addValue(xVals, x);
        addValue(yVals, y);
        addValue(angleVals, angleDegrees);
    }

    public void clearVals(){
        xVals.clear();
        yVals.clear();
        angleVals.clear();
    }

    public double getSmoothedX() {
        return median(xVals);
    }

    public double getSmoothedY() {
        return median(yVals);
    }

    public double getSmoothedAngle() {
        return median(angleVals);
    }

    private void addValue(Deque<Double> deque, double value) {
        if(value == 0.0) return;
        if (deque.size() >= windowSize) {
            deque.pollFirst();
        }
        deque.addLast(value);
    }

    public int getSize(){
        return xVals.size();
    }

    private double median(Collection<Double> values) {
        List<Double> sorted = new ArrayList<>(values);
        sorted.sort(Double::compare);
        int n = sorted.size();
        if (n % 2 == 0) {
            return (sorted.get(n / 2 - 1) + sorted.get(n / 2)) / 2.0;
        } else {
            return sorted.get(n / 2);
        }
    }
}
