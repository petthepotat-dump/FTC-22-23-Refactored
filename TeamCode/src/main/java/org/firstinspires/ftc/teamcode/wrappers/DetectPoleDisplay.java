package org.firstinspires.ftc.teamcode.wrappers;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class DetectPoleDisplay extends OpenCvPipeline {
    private static final Scalar lower_yellow = new Scalar(15,130,130), upper_yellow = new Scalar(25,255,255);
    private Mat hsv = new Mat(), res = new Mat(), mask = new Mat(), edges = new Mat(), lines = new Mat();
    private final double polePosition = 83, targetWidth = 32.5;
    private double maxWidth=0, m=0;
    private int max_x=0, max_y=0;
    public double error=0, widthError=0;
    public int noPole = 0;
    private void quicksort(Mat lines, int low, int high) {
        if (low < high) {
            int pi = partition(lines, low, high);
            quicksort(lines, low, pi-1);
            quicksort(lines, pi+1, high);
        }
    }
    private int partition(Mat lines, int low, int high) {
        double pivot = lines.get(high, 0)[0] + lines.get(high, 0)[2];
        int i = low-1, j = high+1;
        while (true) {
            do {
                i++;
            } while (lines.get(i, 0)[0] + lines.get(i, 0)[2] < pivot);
            do {
                j--;
            } while (lines.get(j, 0)[0] + lines.get(j, 0)[2] > pivot);
            if (i >= j) {
                return j;
            }
            double[] temp = lines.get(i, 0);
            lines.put(i, 0, lines.get(j, 0));
            lines.put(j, 0, temp);
        }
    }
    @Override
    public Mat processFrame(Mat input) {
        // clear the edges
        res.setTo(new Scalar(0));
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lower_yellow, upper_yellow, mask);
        Core.bitwise_and(hsv, hsv, res, mask);
        Imgproc.Canny(res, edges, 50, 160, 3);
        Imgproc.HoughLinesP(edges, lines, 1, Math.PI/180, 100, 20, 100);
        maxWidth = 0;
        m = 0;
        max_x = 0;
        max_y = 0;


        if (lines.rows() > 0) {
            quicksort(lines, 0, lines.rows()-1);
            int maxIndex = -1;
            // loop through lines
            for (int i = 0; i < lines.rows(); i++) {
                // draw the lines
                double[] l = lines.get(i, 0);
                Imgproc.line(input, new Point(l[0], l[1]), new Point(l[2], l[3]), new Scalar(0, 255, 0), 2);
            }

            for (int i = 0; i < lines.rows()-1; i++) {
                double[] line1 = lines.get(i, 0), line2 = lines.get(i+1, 0);
                double x1 = line1[0], y1 = line1[1], x2 = line1[2], y2 = line1[3], x3 = line2[0], y3 = line2[1], x4 = line2[2], y4 = line2[3];
                if (mask.get((int)((y1+y2+y3+y4)/4), (int)((x1+x2+x3+x4)/4))[0] > 0) {
                    double dx = x2-x1, dy = y2-y1;
                    double width = Math.abs(dx*y1-dy*x1-dx*y3+dy*x3)/Math.sqrt(dx*dx+dy*dy);
                    if (width > maxWidth) {
                        maxWidth = width;
                        max_x = (int)(x1+x2+x3+x4)/4;
                        max_y = (int)(y1+y2+y3+y4)/4;
                        maxIndex = i;
                        m = dy/dx;
                    }
                }
            }
            if (maxIndex != -1) {
                error = max_x - polePosition;
                widthError = maxWidth - targetWidth;
                double angle = Math.atan(-1/m);
                Imgproc.line(input, new Point(max_x+maxWidth*Math.cos(angle)*0.5,max_y+maxWidth*Math.sin(angle)*0.5), new Point(max_x-maxWidth*Math.cos(angle)*0.5,max_y-maxWidth*Math.sin(angle)*0.5), new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(max_x, max_y), 3, new Scalar(0, 0, 255), 2);
            } else {
                noPole++;
                error = 0;
                widthError = 0;
            }
        } else {
            noPole++;
            error = 0;
            widthError = 0;
        }
        return input;
    }
}