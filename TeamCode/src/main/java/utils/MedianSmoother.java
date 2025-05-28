package utils;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Deque;
import java.util.List;

public class MedianSmoother {
    public static class Sample {
        private final double x, y, angle;

        public Sample(double x, double y, double angle) {
            this.x = x;
            this.y = y;
            this.angle = angle;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getAngle() {
            return angle;
        }
    }

    private int sizeLimit;
    private final Deque<Sample> samples;

    public MedianSmoother(int sizeLimit) {
        this.sizeLimit = sizeLimit;
        this.samples = new ArrayDeque<>();
    }

    public void add(double x, double y, double angle) {
        if (x == 0 && y == 0 && angle == 0) return;

        if (samples.size() >= sizeLimit) {
            samples.removeFirst();
        }
        samples.addLast(new Sample(x, y, angle));
    }

    public Sample getMedian() {
        if (samples.isEmpty()) return new Sample(0, 0, 0);

        List<Sample> sorted = new ArrayList<>(samples);
        sorted.sort(Comparator.comparingDouble(Sample::getX));
        return sorted.get(sorted.size() / 2);
    }

    public int getSize() {
        return samples.size();
    }

    public void setSizeLimit(int newSize) {
        this.sizeLimit = newSize;
        while (samples.size() > sizeLimit) {
            samples.removeFirst();
        }
    }

    public void clear() {
        samples.clear();
    }
}

