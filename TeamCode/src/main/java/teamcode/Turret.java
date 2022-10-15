/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcFtcLib.ftclib.FtcMotorActuator;

public class Turret
{
    private enum ZeroCalStates
    {
        ZERO_ARM_AND_ELEVATOR,
        RAISE_ARM,
        ZERO_TURRET,
        DONE
    }   //enum ZeroCalStates

    private enum SetPositionStates
    {
        WAIT_FOR_ARM_RAISE,
        SET_TURRET_POSITION,
        DONE
    }   //enum SetPositionStates

    private final Robot robot;
    private final TrcPidActuator pidTurret;

    private final TrcEvent armEvent;
    private final TrcEvent elevatorEvent;
    private final TrcEvent turretEvent;
    private final TrcStateMachine<ZeroCalStates> zeroCalSm;
    private final TrcStateMachine<SetPositionStates> setPosSm;
    private final TrcTaskMgr.TaskObject zeroCalTaskObj;
    private final TrcTaskMgr.TaskObject setPosTaskObj;
    private double turretTargetPosition;

    //state machine where both setPosition and setPower use it. check if elevator and arm below level, otherwise move it up above safe level
    //once you are above elevator and arm, then get out of the safemachine

    public Turret(Robot robot)
    {
        this.robot = robot;

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

        armEvent = new TrcEvent("armEvent");
        elevatorEvent = new TrcEvent("elevatorEvent");
        turretEvent = new TrcEvent("turretEvent");
        zeroCalSm = new TrcStateMachine<>("zeroCalStateMachine");
        setPosSm = new TrcStateMachine<>("setPositionStateMachine");

        zeroCalTaskObj = TrcTaskMgr.createTask("turret zeroCal task", this::zeroCalTask);
        setPosTaskObj = TrcTaskMgr.createTask("turret setPos task", this::setPosTask);
//            globalTracer = new TrcDbgTrace().getGlobalTracer();
//            pidTurret.setMsgTracer(globalTracer);
//            pidTurret.setBeep(androidTone);
    }   //Turret

    //assumption - when arm, elevator come down won't hit anything
    public void zeroCalibrate()
    {
        // CodeReview: after thinking about the arm and turret configuration, zero calibration may be simpler than
        // we thought. Since we just need to raise the arm out of the way before we turn the turret, here is what I
        // think we should do:
        // 1. zero calibrate the arm and the elevator simultaneously. Note: arm calibrates upward.
        // 2. Wait for arm calibrate done (don't care about elevator calibration).
        // 3. zero calibrate the turret.
        //
        // Since there are only 3 steps, we should use notifier callback instead. In this method, we should:
        // 1. zero calibrate the elevator.
        // 2. zero calibrate the arm and chain to a armZeroCalDone callback.
        // 3. In the armZeroCalDone callback, do a fire and forget zero calibrate on the turret.
        zeroCalSm.start(ZeroCalStates.ZERO_ARM_AND_ELEVATOR);
        zeroCalTaskObj.registerTask(TrcTaskMgr.TaskType.FAST_POSTPERIODIC_TASK);
    }   //zeroCalibrate

    //TODO: setPosition() and setPower()

    //before it calls, setPosition, check if arm above a certain threshold
    //if they are below threshold, raise them before turning
    public void setTarget(double position)
    {
//        if(elevator.getPosition() < Robotparams.)
        if (robot.arm.getPosition() >= RobotParams.ARM_MIN_POS_FOR_TURRET)
        {
            // We are safe to turn the turret, do it.
            pidTurret.setTarget(position);
        }
        else
        {
            setPosSm.start(SetPositionStates.WAIT_FOR_ARM_RAISE);
            setPosTaskObj.registerTask(TrcTaskMgr.TaskType.FAST_POSTPERIODIC_TASK);
            turretTargetPosition = position;
        }
    }
    public void setPower(double power){
        if (robot.arm.getPosition() < RobotParams.ARM_MIN_POS_FOR_TURRET){
            setPosSm.start(SetPositionStates.WAIT_FOR_ARM_RAISE);
            setPosTaskObj.registerTask(TrcTaskMgr.TaskType.FAST_POSTPERIODIC_TASK);
        }
        else{
            pidTurret.setPower(power);
        }

    }


    public void zeroCalTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        ZeroCalStates state = zeroCalSm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(1, "State: disabled or waiting...");
        }
        else
        {
            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state)
            {
                case ZERO_ARM_AND_ELEVATOR:
                    zeroCalSm.addEvent(armEvent);
                    zeroCalSm.addEvent(elevatorEvent);
                    robot.arm.zeroCalibrate(RobotParams.ARM_CAL_POWER, armEvent);
                    robot.elevator.zeroCalibrate(RobotParams.ELEVATOR_CAL_POWER, elevatorEvent);
                    zeroCalSm.waitForEvents(ZeroCalStates.RAISE_ARM, 0, true);
                    break;

                case RAISE_ARM:
                    robot.arm.setPresetPosition(2, armEvent, null);
                    zeroCalSm.waitForSingleEvent(armEvent, ZeroCalStates.ZERO_TURRET);
                    break;
                case ZERO_TURRET:
                    pidTurret.zeroCalibrate(RobotParams.TURRET_CAL_POWER, turretEvent);
                    zeroCalSm.waitForSingleEvent(turretEvent, ZeroCalStates.DONE);
                    break;
                default:
                case DONE:
                    cancel();
                    break;
            }

        }

    }

    public void setPosTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode) {
        SetPositionStates state = setPosSm.checkReadyAndGetState();

        if (state == null) {
            robot.dashboard.displayPrintf(1, "State: disabled or waiting...");
        } else {
            boolean traceState = true;
            String msg;

            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state) {
                case WAIT_FOR_ARM_RAISE:
                    robot.arm.setPresetPosition(2, armEvent, null);
                    if (turretTargetPosition != 0) {
                        setPosSm.waitForSingleEvent(armEvent, SetPositionStates.DONE);
                    }
                    else {
                        setPosSm.setState(SetPositionStates.DONE);
                    }
                    break;
                case SET_TURRET_POSITION:
                    pidTurret.setTarget(turretTargetPosition, false, 1.0, turretEvent);
                    turretTargetPosition = 0;
                    setPosSm.waitForSingleEvent(turretEvent, SetPositionStates.DONE);
                    break;
                default:
                case DONE:
                    cancelSetPosition();
                    break;
            }
        }
    }
    public void cancel () {
        zeroCalSm.stop();
        zeroCalTaskObj.unregisterTask();
        robot.arm.cancel();
        robot.elevator.cancel();
        pidTurret.cancel();
    }

    public void cancelSetPosition () {
        setPosSm.stop();
        setPosTaskObj.unregisterTask();
        robot.arm.cancel();
        robot.elevator.cancel();
        pidTurret.cancel();
    }

}
