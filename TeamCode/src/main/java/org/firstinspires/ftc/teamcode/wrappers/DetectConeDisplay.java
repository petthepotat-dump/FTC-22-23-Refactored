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

public class DetectConeDisplay extends OpenCvPipeline {
    private static final Scalar
            lower_blue = new Scalar(100,60,80),
            upper_blue = new Scalar(125,255,255),
            lower_red1 = new Scalar(165,80,100),
            upper_red1 = new Scalar(180,255,255),
            lower_red2 = new Scalar(0, 80, 100),
            upper_red2 = new Scalar(5, 80, 100);
    private Mat hsv = new Mat(), mask = new Mat(), hierarchy = new Mat();
    private List<MatOfPoint> contours = new java.util.ArrayList<>();
    private boolean blueTeam;
    public double x, y, width, height, left, right;
    public double[] hsvColor = new double[3];
    public DetectConeDisplay(boolean blue_team) { blueTeam = blue_team;}
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        if (blueTeam) Core.inRange(hsv, lower_blue, upper_blue, mask);
        else {
            Mat red = new Mat();
            Core.inRange(hsv, lower_red1, upper_red1, red);
            Core.inRange(hsv, lower_red2, upper_red2, mask);
            Core.bitwise_or(red, mask, mask);
        }
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.circle(input, new Point(input.width() / 2, input.height() / 2), 5, new Scalar(0, 0, 255), 2);
        hsvColor = hsv.get(input.height() / 2, input.width() / 2);
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
            if (blueTeam) Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 0, 255), 2);
            else Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(255, 0, 0), 2);
            x = rect.x+rect.width/2;
            y = rect.y+rect.height/2;
            width = rect.width;
            height = rect.height;
            left = rect.x;
            right = rect.x+rect.width;
        }
        contours.clear();
        return input;
    }
}
