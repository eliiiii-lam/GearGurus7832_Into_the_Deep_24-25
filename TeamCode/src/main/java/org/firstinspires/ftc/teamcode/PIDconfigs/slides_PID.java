package org.firstinspires.ftc.teamcode.PIDconfigs;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class slides_PID extends OpMode {
    private PIDController controller1;
    public static double p1 = 0, i1 = 0, d1 = 0;
    public static double f1 = 0;
    public static int target1;

    private final double ticks_in_degrees1 = 1425.1/360.0;; //change the 360 back to 180 if no work



    private DcMotorEx uppies ;



    @Override
    public void init(){

        controller1 = new PIDController(p1,i1,d1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        uppies = hardwareMap.get(DcMotorEx.class, "uppies");






    }


    @Override
    public void loop(){

        controller1.setPID(p1,i1,d1);
        int armPos1 = uppies.getCurrentPosition();
        double pid1 = controller1.calculate(armPos1, target1);
        double ff1 = Math.cos(Math.toRadians(target1/ticks_in_degrees1)) * f1;

        double power1 = pid1 + ff1;

        int error1 = target1 - armPos1;





        uppies.setPower(power1); //setting the motor to the desired position


        telemetry.addData("pos ", armPos1);
        telemetry.addData("target ", target1);
//        telemetry.addData("error", error);
//        telemetry.addData("PID", pid);
//        telemetry.addData("maxPow", max_actual_power);
        telemetry.update();


    }
}