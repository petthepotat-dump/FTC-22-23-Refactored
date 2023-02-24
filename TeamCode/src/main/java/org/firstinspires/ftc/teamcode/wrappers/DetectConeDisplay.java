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
            lower_blue = new Scalar(100,100,100),
            upper_blue = new Scalar(120,255,255),
            lower_red = new Scalar(90,100,100),
            upper_red = new Scalar(110,255,255);
    private Mat hsv = new Mat(), mask = new Mat(), hierarchy = new Mat();
    private List<MatOfPoint> contours = new java.util.ArrayList<>();
    private boolean blueTeam;
    public double x, y, width, height;
    public double[] hsvColor = new double[3];
    public DetectConeDisplay(boolean blue_team) { blueTeam = blue_team;}
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        if (blueTeam) Core.inRange(hsv, lower_blue, upper_blue, mask);
        else Core.inRange(hsv, lower_red, upper_red, mask);
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
        }
        contours.clear();
        return input;
    }
}
