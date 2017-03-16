package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor  Mladder     = null;
    public Servo wrench = null;
    public Servo pushLight = null;
    public TouchSensor TCH =null;
    public GyroSensor gyro = null;
    public double powerl = 0.0, powerr = 0.0;
    public DcMotor r1 = null, r2 = null;
    public DcMotor l1 = null, l2 = null;
    public DcMotor wipeYellow = null;
    public DcMotor miniGun = null, collector = null;
    public UltrasonicSensor eye = null;
    public UltrasonicSensor uls = null;
    public LightSensor ls = null;
    public ColorSensor cs = null;

    public boolean isLoaded = false;
    public boolean isReleased =false;
    public int LOP=-1;
    public boolean isLoading = false;

    //  public Servo    leftClaw    = null;
  //  public Servo    rightClaw   = null;

//    public static final double MID_SERVO       =  0.5 ;
//    public static final double ARM_UP_POWER    =  0.45 ;
//    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    private HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot(){

    }
    public  void setRunMode(DcMotor.RunMode mode)
    {
        l1.setMode(mode);
        l2.setMode(mode);
        r1.setMode(mode);
        r2.setMode(mode);
    }
    public  void TargetPosition(Telemetry TP)
    {
        TP.addData("P of L1","%d",l1.getCurrentPosition());
        TP.addData("P of L2","%d",l2.getCurrentPosition());
        TP.addData("P of R1","%d",r1.getCurrentPosition());
        TP.addData("P of R2","%d",r2.getCurrentPosition());
    }
    public  void TargetPosition(int TP)
    {
        l1.setTargetPosition(-TP);
        l2.setTargetPosition(TP);
        r1.setTargetPosition(TP);
        r2.setTargetPosition(-TP);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        l1 = hwMap.dcMotor.get("L1");
        l2 = hwMap.dcMotor.get("L2");
        r1 = hwMap.dcMotor.get("R1");
        r2 = hwMap.dcMotor.get("R2");
        wipeYellow = hwMap.dcMotor.get("wy");
        Mladder = hwMap.dcMotor.get("ladder");
        wrench = hwMap.servo.get("wrench");
        collector = hwMap.dcMotor.get("collector");
        miniGun = hwMap.dcMotor.get("shooter");
        TCH = hwMap.touchSensor.get("TCH");
        eye = hwMap.ultrasonicSensor.get("eyes");
        gyro = hwMap.gyroSensor.get("gyro");
        uls = hwMap.ultrasonicSensor.get("uls");
        ls = hwMap.lightSensor.get("ls");
        cs = hwMap.colorSensor.get("cs");
        pushLight = hwMap.servo.get("push");

//        rightMotor  = hwMap.dcMotor.get("right_drive");
//        armMotor    = hwMap.dcMotor.get("left_arm");
//        TMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        wipeYellow.setPower(0);
        l1.setPower(0);
        l2.setPower(0);
        r1.setPower(0);
        r2.setPower(0);
        collector.setPower(0.0);
        pushLight.setPosition(1.0);
        Mladder.setPower(0);
        miniGun.setPower(0);
        miniGun.setPower(0);
        wrench.scaleRange(0.45,1);
        wrench.setPosition(1.0);
        gyro.calibrate();
        ls.enableLed(true);
        cs.enableLed(true);
//        rightMotor.setPower(0);
//        armMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        TMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        TMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        miniGun.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
//        leftClaw = hwMap.servo.get("left_hand");
//        rightClaw = hwMap.servo.get("right_hand");
//        leftClaw.setPosition(MID_SERVO);
//        rightClaw.setPosition(MID_SERVO);
    }


    private void checkl()
    {
        if(powerl > 1.0)
        {
            powerl = 1.0;
        }
        if(powerl < -1.0)
        {
            powerl = -1.0;
        }
    }
    private void checkr()
    {
        if(powerr > 1.0)
        {
            powerr = 1.0;
        }
        if(powerr < -1.0)
        {
            powerr = -1.0;
        }
    }
    public void pushOnebyOne(double l,double r)
    {
        l*=-1.0;
        l1.setPower(l);
        r2.setPower(l);
        l2.setPower(r);
        r1.setPower(r);
    }
    public void pushGamepad(double x, double y)
    {
        powerl = x-y;
        powerr = x+y;

        checkr();
        checkl();
        powerr *= -1;
        powerl *= -1;
        l1.setPower(powerl);
        l2.setPower(powerr);
        r1.setPower(powerr);
        r2.setPower(powerl);

    }
    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period..
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public void TurnByGyro(int deg, Telemetry T)
    {
        DcMotor.RunMode Temp = l1.getMode();
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pushGamepad(0, 0);
        if(deg==0 || Math.abs(deg) > 180) return ;
        gyro.resetZAxisIntegrator();
        while(gyro.isCalibrating()){}
        pushGamepad(0.3 * (Math.abs(deg)/deg), 0);

        while(true) {
            if ( deg > 0 && (Math.abs(deg) - (gyro.getHeading())) <= 2 ) break;
            if ( deg < 0 && (Math.abs(deg) - (360 - gyro.getHeading())) <= 2 ) break;
            T.addData("Heading: ", "%d : %d", /*Math.abs(deg) - */ (Math.abs(deg) - (Math.abs(deg)/deg * gyro.getHeading())) % 360, gyro.getHeading());
        }
        pushGamepad(0,0);
        setRunMode(Temp);
    }

    public void stop() {
        wipeYellow.setPower(0);
        r1.setPower(0);
        r2.setPower(0);
        l1.setPower(0);
        l2.setPower(0);
        Mladder.setPower(0);
        miniGun.setPower(0);
        collector.setPower(0);
        wrench.setPosition(1.0);//0.4
    }
}

