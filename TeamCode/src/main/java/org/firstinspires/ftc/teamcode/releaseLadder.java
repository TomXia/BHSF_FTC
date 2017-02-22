package org.firstinspires.ftc.teamcode;

/**
 * Created by Administrator on 2017/2/22.
 */

public class releaseLadder extends Thread {
    public HardwarePushbot robot;
    public releaseLadder(HardwarePushbot rob){
        robot=rob;
    }
    @Override
    public void run() {
        robot.isReleased=true;
        robot.Mladder.setPower(-0.4);
        try {
            Thread.sleep(250);
        }catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.Mladder.setPower(0.4);
        try {
            Thread.sleep(275);
        }catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.Mladder.setPower(0.0);
    }
}
