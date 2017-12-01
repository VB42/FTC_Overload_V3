
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * This class allows the driver controlled operation of the robot.
 * It controls 4 mechanum wheels and allows for strafing,
 * in addition to the normal controls of turning and forward and backward movement
 *V 1.0
 *
 *
 */

@TeleOp(name="TankDrive", group="Test")
public class TankDrive extends OpMode
{

    private DcMotorController control;
    private DcMotorController control2;
    private ServoController servoController;
    //private DcMotorController control3;
    private DcMotor left_front;
    private DcMotor right_front;
    private DcMotor left_back;
    private DcMotor right_back;
    //private DcMotor lift;
    private Servo ClawL;
    private Servo ClawR;
    private ColorSensor color_sensor;
    private Servo colorservo;
    public static double threshold = 0.2;
    @Override
    public void init()
    {
        control = hardwareMap.dcMotorController.get("drive_controller");
        servoController = hardwareMap.servoController.get("servo_controller");
        left_front = hardwareMap.dcMotor.get("left_front");
        right_front = hardwareMap.dcMotor.get("right_front");

        control2 = hardwareMap.dcMotorController.get("drive_controller2");


        left_back = hardwareMap.dcMotor.get("left_back");
        right_back = hardwareMap.dcMotor.get("right_back");

        //control3 = hardwareMap.dcMotorController.get("drive_controller3");
        ClawL=hardwareMap.servo.get("ClawL");
        ClawR=hardwareMap.servo.get("ClawR");

        //liftL=hardwareMap.dcMotor.get("liftl");
        //liftR=hardwareMap.dcMotor.get("liftr");
        color_sensor = hardwareMap.colorSensor.get("color");
        color_sensor.enableLed(true);

        colorservo = hardwareMap.servo.get("colorservo");

        colorservo.setPosition(0.7);

    }
    //Helper method for resetting all motors to a stop
    public void resetMotors(){
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
    }

    //Helper method for logging values to screens
    public void log(String main, String val){
        telemetry.addData(main, val);
    }

    @Override
    public void loop() throws IllegalArgumentException
    {


        if(Math.abs(gamepad2.left_stick_y) > threshold){
            //liftL.setPower(Math.power(-gamepad2.left_stick_y, 3));
        }
        if(Math.abs(gamepad2.right_stick_y) > threshold){
            //rightL.setPower(Math.power(gamepad2.right_stick_y, 3));
        }

        if(gamepad2.dpad_up){
            //liftL.setPower(1);
            //rightL.setPower(-1);
        }

        else if(gamepad2.dpad_down){
            //liftL.setPower(-1);
            //rightL.setPower(1);
        }

        else{
            //liftL.setPower(0);
            //rightL.setPower(0);
        }

        if(gamepad2.x){
            ClawL.setPosition(1);
            ClawR.setPosition(0);
        }

        else if(gamepad2.y){
            ClawL.setPosition(0);
            ClawR.setPosition(1);
        }

         /*
          *  Checks if the y value is forward or backwards.
          *  The y values are reversed so we have to negate it to logically use it.
          *
          */
        if(Math.abs(gamepad1.left_stick_y) > threshold){
            left_front.setPower(Math.pow(-gamepad1.left_stick_y, 3));
            left_back.setPower(Math.pow(-gamepad1.left_stick_y, 3));
            log("Left Stick Y", Float.toString(-gamepad1.left_stick_y));

        }


         /*
          *  Checks if the y value is forward or backwards.
          *
          */
        if(Math.abs(gamepad1.right_stick_y) > threshold){
            right_front.setPower(Math.pow(gamepad1.right_stick_y, 3));
            right_back.setPower(Math.pow(gamepad1.right_stick_y, 3));
            log("Right Stick Y", Float.toString(gamepad1.right_stick_y));

        }


        /*
         *  Used for strafing, sets the power to allow sideways movement when shifted to the right
         *
         */
        if(Math.abs(gamepad1.left_stick_x) > threshold && Math.abs(gamepad1.right_stick_x) > threshold){
            left_front.setPower(Math.pow(-gamepad1.left_stick_x, 3));
            left_back.setPower(Math.pow(gamepad1.left_stick_x, 3));
            right_front.setPower(Math.pow(-gamepad1.right_stick_x, 3));
            right_back.setPower(Math.pow(gamepad1.right_stick_x, 3));
            log("Sideways", Float.toString(gamepad1.right_stick_x));
            log("Sideways", Float.toString(gamepad1.left_stick_x));

        }
        /*
         *  Resets motors when the control isn't moving
         *
         */
        if(Math.abs(gamepad1.left_stick_x) < threshold && Math.abs(gamepad1.right_stick_x) < threshold
                && Math.abs(gamepad1.left_stick_y) < threshold && Math.abs(gamepad1.right_stick_y) < threshold){
            resetMotors();
        }
        /*
         * Code for opening and closing the claw
         *
         */
        if(gamepad1.x)
        {
            ClawL.setPosition(1);
            ClawR.setPosition(0);
        }
        else if(gamepad1.y)
        {
            ClawL.setPosition(0);
            ClawR.setPosition(1);
        }
        else
        {
            ClawL.setPosition(0.5);
            ClawR.setPosition(0.5);
        }




    }
}

