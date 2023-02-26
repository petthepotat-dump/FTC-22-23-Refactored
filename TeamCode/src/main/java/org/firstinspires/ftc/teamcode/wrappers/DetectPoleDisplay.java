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
    private static final Scalar lower_yellow = new Scalar(15,130,70),
                                upper_yellow = new Scalar(35,255,245),
                                lower_red1 = new Scalar(0, 80, 80),
                                upper_red1 = new Scalar(5, 255, 255),
                                lower_red2 = new Scalar(175, 80, 80),
                                upper_red2 = new Scalar(180, 255, 255),
                                lower_blue = new Scalar(95, 80, 80),
                                upper_blue = new Scalar(125, 255, 255);
    private Mat hsv = new Mat(), mask = new Mat(), color = new Mat();
    private List<MatOfPoint> contours = new java.util.ArrayList<>();
    private Mat hierarchy = new Mat();
    private boolean cone = false;
    public double x, y, width, height;
    public void poleHasCone(boolean cone) {
        this.cone = cone;
    }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        if (cone) {
            Core.inRange(hsv, lower_red1, upper_red1, mask);
            Core.inRange(hsv, lower_red2, upper_red2, color);
            Core.bitwise_or(mask, color, mask);
            Core.inRange(hsv, lower_blue, upper_blue, color);
            Core.bitwise_or(mask, color, mask);
        } else {
            Core.inRange(hsv, lower_yellow, upper_yellow, mask);
        }
        contours.clear();
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
            Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 0, 255), 2);
            x = rect.x+rect.width/2;
            y = rect.y+rect.height/2;
            width = rect.width;
            height = rect.height;
        }
        return input;
    }
}