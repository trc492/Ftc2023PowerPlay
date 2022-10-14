package teamcode;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcNotifier;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcFtcLib.ftclib.FtcMotorActuator;

public class Turret{
    public enum ZeroCalStates{
        ZERO_ARM_AND_ELEVATOR,
        RAISE_ARM,
        ZERO_TURRET,
        DONE

    }

    TrcPidActuator pidTurret;
    public TrcDbgTrace globalTracer;
    Robot robot;
    boolean armCalibrationDone = false;
    boolean elevatorCalibrationDone = false;
    final TrcTaskMgr.TaskObject zeroCalTaskObj;
    private final TrcStateMachine<ZeroCalStates> sm;
    //state machine where both setPosition and setPower use it. check if elevator and arm below level, otherwise move it up above safe level
    //once you are above elevator and arm, then get out of the safemachine
    private final TrcEvent armEvent;
    private final TrcEvent elevatorEvent;
    private final TrcEvent turretEvent;


    public Turret(Robot robot){
            this.robot = robot;
            armEvent = new TrcEvent("arm event");
            elevatorEvent = new TrcEvent("elevator event");
            turretEvent = new TrcEvent("turret event");

        sm = new TrcStateMachine<>("zeroCalStateMachine");
            zeroCalTaskObj = TrcTaskMgr.createTask("turret zeroCal task", this::zeroCalTask);
//            globalTracer = new TrcDbgTrace().getGlobalTracer();
            final FtcMotorActuator.MotorParams motorParams = new FtcMotorActuator.MotorParams(
                    RobotParams.TURRET_MOTOR_INVERTED,
                    RobotParams.TURRET_HAS_LOWER_LIMIT_SWITCH, RobotParams.TURRET_LOWER_LIMIT_INVERTED,
                    RobotParams.TURRET_HAS_UPPER_LIMIT_SWITCH, RobotParams.TURRET_UPPER_LIMIT_INVERTED);
            final TrcPidActuator.Parameters turretParams = new TrcPidActuator.Parameters()
                    .setPosRange(RobotParams.TURRET_MIN_POS, RobotParams.TURRET_MAX_POS)
                    .setScaleOffset(RobotParams.TURRET_DEG_PER_COUNT, RobotParams.TURRET_OFFSET)
                    .setPidParams(new TrcPidController.PidParameters(
                            RobotParams.TURRET_KP, RobotParams.TURRET_KI, RobotParams.TURRET_KD,
                            RobotParams.TURRET_TOLERANCE))
                    .setStallProtectionParams(
                            RobotParams.TURRET_STALL_MIN_POWER, RobotParams.TURRET_STALL_TOLERANCE,
                            RobotParams.TURRET_STALL_TIMEOUT, RobotParams.TURRET_RESET_TIMEOUT)
                    .setZeroCalibratePower(RobotParams.TURRET_CAL_POWER)
                    .setPosPresets(RobotParams.TURRET_PRESET_LEVELS);
            pidTurret = new FtcMotorActuator(
                    RobotParams.HWNAME_TURRET, motorParams, turretParams).getPidActuator();
//            pidTurret.setMsgTracer(globalTracer);
//            pidTurret.setBeep(androidTone);
    }
//    TrcNotifier.Receiver\
    //assumption - when arm, elevator come down won't hit anything
    public void zeroCalibrate(){
        sm.start(ZeroCalStates.ZERO_ARM_AND_ELEVATOR);
        zeroCalTaskObj.registerTask(TrcTaskMgr.TaskType.FAST_POSTPERIODIC_TASK);

    }

    //TODO: setPosition() and setPower()

    //before it calls, setPosition, check if arm above a certain threshold
    //if they are below threshold, raise them before turning
    public void setTarget(double position){
//        if(elevator.getPosition() < Robotparams.)
    }
    public void setPower(double power){

    }


    public void zeroCalTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode) {
        ZeroCalStates state = sm.checkReadyAndGetState();

        if (state == null) {
            robot.dashboard.displayPrintf(1, "State: disabled or waiting...");
        } else {
            boolean traceState = true;
            String msg;

            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state) {

                case ZERO_ARM_AND_ELEVATOR:
                    sm.addEvent(armEvent);
                    sm.addEvent(elevatorEvent);
                    robot.arm.zeroCalibrate(RobotParams.ARM_CAL_POWER, armEvent);
                    robot.elevator.zeroCalibrate(RobotParams.ELEVATOR_CAL_POWER, elevatorEvent);
                    sm.waitForEvents(ZeroCalStates.RAISE_ARM, 0, true);
                    break;
                case RAISE_ARM:
                    robot.arm.setPresetPosition(2, armEvent, null);
                    sm.waitForSingleEvent(armEvent, ZeroCalStates.ZERO_TURRET);
                    break;
                case ZERO_TURRET:
                    pidTurret.zeroCalibrate(RobotParams.TURRET_CAL_POWER, turretEvent);
                    sm.waitForSingleEvent(turretEvent, ZeroCalStates.DONE);
                    break;
                default:
                case DONE:
                    cancel();
                    break;
            }

        }

    }
    public void cancel () {
        sm.stop();
        zeroCalTaskObj.unregisterTask();
        robot.arm.cancel();
        robot.elevator.cancel();
        pidTurret.cancel();
    }

}
