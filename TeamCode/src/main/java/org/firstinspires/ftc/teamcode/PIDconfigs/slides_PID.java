package org.firstinspires.ftc.teamcode.PIDconfigs;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class slides_PID extends OpMode {
    private PIDController controller1;
    public static double p1 = 0.06, i1 = 0.01, d1 = 0.0001;
    public static double f1 = 0.09;
    public static int target1;

    private final double ticks_in_degrees1 = 14.64; //change the 360 back to 180 if no work



    private DcMotorEx uppies ;
    private DcMotorEx uppies1 ;



    @Override
    public void init(){

        controller1 = new PIDController(p1,i1,d1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        uppies = hardwareMap.get(DcMotorEx.class, "uppies");
        uppies1 = hardwareMap.get(DcMotorEx.class, "uppies1");

        uppies1.setDirection(DcMotorSimple.Direction.REVERSE);





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
        uppies1.setPower(power1);


        telemetry.addData("pos ", armPos1);
        telemetry.addData("target ", target1);
//        telemetry.addData("error", error);
//        telemetry.addData("PID", pid);
//        telemetry.addData("maxPow", max_actual_power);
        telemetry.update();


    }
}