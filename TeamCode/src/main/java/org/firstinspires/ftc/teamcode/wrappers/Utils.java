package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Timer;

public class Utils {

    public static class Timer{
        public long start, end, delta, cstart;
        public Timer(){
            // get start time
            start = System.currentTimeMillis();
            cstart = start;
        }
        // update function
        public void update(){
            end = System.currentTimeMillis();
            delta = end - start;
            start = end;
        }
        public int timePassedAbs(){
            update();
            return (int) (end - cstart);
        }
    }

    public static class TimerThread extends Thread {
        private Timer timer;
        private long target;
        public boolean done;
        private static long AUTON_TIME = 30*1000;
        public TimerThread(long targetTime){
            timer = new Timer();
            target = targetTime;
            done = false;
            // start timer
            start();
        }

        @Override
        public void run() {
            try {
                while (AUTON_TIME - timer.timePassedAbs() > target) {
                    sleep(10);
                }
            } catch (InterruptedException e) {}
            done = true;
        }
    }


    public static class PylonStackTracker{
        int stack = 28, counter = 5, cone = 5;
        public PylonStackTracker(){
        }
        public void removePylon(){
            counter--;
        }
        public int getPylonStackHeight(){
            return stack - counter * cone;
        }

    }

    public static class TrackPad {
        private Gamepad gamepad;
        public TrackPad(Gamepad gamepad){
            this.gamepad = gamepad;
        }
        public double getX1(){
            // trackpad distance
            return gamepad.touchpad_finger_1_x;
        }
        public double getY1(){
            // trackpad distance
            return gamepad.touchpad_finger_1_y;
        }
    }

    public static class RotateWithTrackpad{
        private static double center = 0;
        TrackPad trackPad;
        private double x1;
        public RotateWithTrackpad(TrackPad trackPad){
            this.trackPad = trackPad;
        }
        public double getMoveX(){
            x1 = trackPad.getX1();
            // assuming center = 0
            return (center - x1);
        }
    }
}
