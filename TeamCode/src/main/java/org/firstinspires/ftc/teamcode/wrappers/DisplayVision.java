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

public class DisplayVision extends OpenCvPipeline {
    private static final Scalar
            lower_orange1 = new Scalar(175,60,100),
            upper_orange1 = new Scalar(180,235,255),
            lower_orange2 = new Scalar(0, 60, 100),
            upper_orange2 = new Scalar(5, 100, 100),
            lower_blue = new Scalar(95,40,40),
            upper_blue = new Scalar(105,235,255),
            lower_purple = new Scalar(140,100,100),
            upper_purple = new Scalar(150,255,255);
    private Mat hsv = new Mat(), colour = new Mat(), mask = new Mat(), hierarchy = new Mat();
    private List<MatOfPoint> contours = new java.util.ArrayList<>();
    public String route = "NONE";
    public double[] hsvColor = new double[3];
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lower_orange1, upper_orange1, mask);
        Core.inRange(hsv, lower_orange2, upper_orange2, colour);
        Core.bitwise_or(colour, mask, mask);
        Core.inRange(hsv, lower_blue, upper_blue, colour);
        Core.bitwise_or(colour, mask, mask);
        Core.inRange(hsv, lower_purple, upper_purple, colour);
        Core.bitwise_or(colour, mask, mask);
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() > 0) {
            double maxArea = 0;
            int maxAreaIdx = -1;
            for (int idx = 0; idx < contours.size(); idx++) {
                Mat contour = contours.get(idx);
                double contourArea = Imgproc.contourArea(contour);
                if (contourArea > maxArea) {
                    maxArea = contourArea;
                    maxAreaIdx = idx;
                }
            }
            if (maxArea>=50) {
                Rect rect = Imgproc.boundingRect(contours.get(maxAreaIdx));
                Point center = new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
                double[] color = hsv.get((int) center.y, (int) center.x);
                if ((color[0] >= 0 && color[0] <= 5) || (color[0]>=175 && color[0]<=180)) {
                    Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(255, 255, 0), 2);
                    route = "LEFT";
                } else if (color[0] >= 90 && color[0] <= 110) {
                    Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 255), 2);
                    route = "CENTER";
                } else if (color[0] >= 140 && color[0] <= 150) {
                    Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(255, 0, 255), 2);
                    route = "RIGHT";
                }
            }
        }
        contours.clear();
        // center pixel
        Imgproc.circle(input, new Point(input.width() / 2, input.height() / 2), 5, new Scalar(0, 0, 255), 2);
        // find hsv value of center pixel
        hsvColor = hsv.get(input.height() / 2, input.width() / 2);
        return input;
    }
}
