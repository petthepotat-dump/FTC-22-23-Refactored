package org.firstinspires.ftc.teamcode.wrappers;

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
        public long timePassedAbs(){
            update();
            return (end - cstart);
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
            run();
        }

        @Override
        public void run() {
            try {
                while (AUTON_TIME - timer.timePassedAbs() > target) {
                    sleep(10);
                }
            } catch (InterruptedException e) {
            }
            done = true;
        }
    }


    public static class PylonStackTracker{
        int left = 5, base = 10, pheight = 18;
        public PylonStackTracker(){

        }
        public void removePylon(){
            left--;
        }
        public int getPylonStackHeight(){
            return (base + (left-1) * pheight);
        }

    }

}
