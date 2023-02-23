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
            lower_yellow = new Scalar(10,100,100),
            upper_yellow = new Scalar(30,255,255),
            lower_cyan = new Scalar(95,100,100),
            upper_cyan = new Scalar(105,255,255),
            lower_magenta = new Scalar(160,100,100),
            upper_magenta = new Scalar(170,255,255);
    private Mat hsv = new Mat(), yellow = new Mat(), cyan = new Mat(), magenta = new Mat(), mask = new Mat(), hierarchy = new Mat();
    private List<MatOfPoint> contours = new java.util.ArrayList<>();
    public int route = 0;
    public double[] hsvColor = new double[3];
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lower_yellow, upper_yellow, yellow);
        Core.inRange(hsv, lower_cyan, upper_cyan, cyan);
        Core.inRange(hsv, lower_magenta, upper_magenta, magenta);
        Core.bitwise_or(yellow, cyan, mask);
        Core.bitwise_or(mask, magenta, mask);
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
            if (maxArea<50) return input;
            Rect rect = Imgproc.boundingRect(contours.get(maxAreaIdx));
            Point center = new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
            double[] color = hsv.get((int) center.y, (int) center.x);
            if (color[0] >= 20 && color[0] <= 30) {
                Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(255, 255, 0), 2);
                route = 0;
            } else if (color[0] >= 95 && color[0] <= 105) {
                Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 255), 2);
                route = 1;
            } else if (color[0] >= 160 && color[0] <= 170) {
                Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(255, 0, 255), 2);
                route = 2;
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
