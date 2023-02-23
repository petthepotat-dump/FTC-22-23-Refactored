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
            lower_yellow = new Scalar(20,140,140),
            upper_yellow = new Scalar(30,255,255),
            lower_cyan = new Scalar(95,100,100),
            upper_cyan = new Scalar(105,255,255),
            lower_magenta = new Scalar(160,100,100),
            upper_magenta = new Scalar(170,255,255);
    private Mat yellow = new Mat(), cyan = new Mat(), magenta = new Mat(), mask = new Mat(), hierarchy = new Mat();
    private List<MatOfPoint> contours = new java.util.ArrayList<>();
    public String route = "OH SHIT!";
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        Core.inRange(input, lower_yellow, upper_yellow, yellow);
        Core.inRange(input, lower_cyan, upper_cyan, cyan);
        Core.inRange(input, lower_magenta, upper_magenta, magenta);
        Core.bitwise_or(yellow, cyan, mask);
        Core.bitwise_or(mask, magenta, mask);
        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
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
            double[] color = input.get((int) center.y, (int) center.x);
            if (color[0] >= 20 && color[0] <= 30) route = "LEFT";
            else if (color[0] >= 95 && color[0] <= 105) route = "CENTER";
            else if (color[0] >= 160 && color[0] <= 170) route = "RIGHT";
            else route = "OH SHIT!";
        }
        return input;
    }
}
