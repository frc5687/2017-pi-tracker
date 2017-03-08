package org.frc5687.pitracker2017;

import java.util.Comparator;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

/**
 * Created by Ben Bernard on 1/27/2017.
 */
public class ContourComparator implements Comparator<MatOfPoint> {

    @Override
    public int compare(MatOfPoint o1, MatOfPoint o2) {
        double x = Imgproc.contourArea(o1);
        double y = Imgproc.contourArea(o2);
        if (x < y) { return -1; }
        if (x > y) { return 1; }
        return 0;
    }
}
