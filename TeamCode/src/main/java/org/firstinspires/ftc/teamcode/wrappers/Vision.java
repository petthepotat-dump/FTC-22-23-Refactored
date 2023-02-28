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

public class Vision extends OpenCvPipeline {

    private static final Scalar
            lower_yellow = new Scalar(25,120,140),
            upper_yellow = new Scalar(40,220,255),
            lower_cyan = new Scalar(93,90,100),
            upper_cyan = new Scalar(113,170,255),
            lower_magenta = new Scalar(160,100,180),
            upper_magenta = new Scalar(170,180,255);
    private Mat yellow = new Mat(),
            hsv = new Mat(),
            cyan = new Mat(),
            magenta = new Mat(),
            mask = new Mat(),
            hierarchy = new Mat(),
            buffer = new Mat();
    public double[] hsvColor = new double[3];

    private List<MatOfPoint> contours = new java.util.ArrayList<>();
    public String route = "OH SHIT!";

    // ============================================================================== //

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lower_yellow, upper_yellow, yellow);
        Core.inRange(hsv, lower_cyan, upper_cyan, cyan);
        Core.inRange(hsv, lower_magenta, upper_magenta, magenta);
        Core.bitwise_or(yellow, cyan, mask);
        Core.bitwise_or(mask, magenta, mask);
        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        // draw circle around center
        Imgproc.circle(input, new Point(input.width() / 2, input.height() / 2), 5, new Scalar(0, 0, 255), 2);
        if (contours.size() > 0) {
            double maxArea = 0;
            int maxAreaIdx = -1;
            for (int idx = 0; idx < contours.size(); idx++) {
                Mat contour = contours.get(idx);
                double contourArea = Imgproc.contourArea(contour);
                Rect rect = Imgproc.boundingRect(contour);
                if (contourArea > maxArea && rect.height<2.5*rect.width && rect.height>rect.width) {
                    maxArea = contourArea;
                    maxAreaIdx = idx;
                }
            }
            if (maxArea<80) {
                route = "OH SHIT!";
                return input;
            }
            Rect rect = Imgproc.boundingRect(contours.get(maxAreaIdx));
            Point center = new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
            double[] color = hsv.get((int) center.y, (int) center.x);
            // yellow
            if (color[0] >= 25 && color[0] <= 40) {
                Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(255, 255, 0), 2);
                route = "LEFT";
            }
            // cyan
            else if (color[0] >= 95 && color[0] <= 105) {
                Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 255), 2);
                route = "CENTER";
            }
            // magenta
            else if (color[0] >= 160 && color[0] <= 170) {
                Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(255, 0, 255), 2);
                route = "RIGHT";
            }
        }
        return input;
    }
}
