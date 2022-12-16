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

import TrcCommonLib.trclib.TrcAnalogSensorTrigger;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcFtcLib.ftclib.FtcDistanceSensor;
import TrcCommonLib.trclib.TrcServoGrabber;
import TrcFtcLib.ftclib.FtcServo;

public class Grabber
{
    private final TrcDbgTrace msgTracer;
    private final TrcServoGrabber grabber;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param msgTracer specifies the tracer to used for logging events, can be null if not provided.
     */
    public Grabber(String instanceName, TrcDbgTrace msgTracer)
    {
        final TrcServoGrabber.Parameters grabberParams = new TrcServoGrabber.Parameters()
            .setStepParams(
                RobotParams.GRABBER_MAX_STEPRATE, RobotParams.GRABBER_MIN_POS, RobotParams.GRABBER_MAX_POS)
            .setServoInverted(RobotParams.GRABBER_LSERVO_INVERTED, RobotParams.GRABBER_RSERVO_INVERTED)
            .setTriggerInverted(RobotParams.GRABBER_TRIGGER_INVERTED)
            .setThresholds(RobotParams.GRABBER_TRIGGER_THRESHOLD, RobotParams.GRABBER_HAS_OBJECT_THRESHOLD)
            .setOpenParams(RobotParams.GRABBER_OPEN_POS, RobotParams.GRABBER_OPEN_TIME)
            .setCloseParams(RobotParams.GRABBER_CLOSE_POS, RobotParams.GRABBER_CLOSE_TIME)
            .setMsgTracer(msgTracer);

        this.msgTracer = msgTracer;
        FtcServo leftServo = new FtcServo(instanceName + ".left");
        FtcServo rightServo = new FtcServo(instanceName + ".right");
        TrcAnalogSensorTrigger<FtcDistanceSensor.DataType> analogTrigger = null;

        if (RobotParams.Preferences.hasGrabberSensor)
        {
            FtcDistanceSensor sensor = new FtcDistanceSensor(instanceName + ".sensor");
            analogTrigger = new TrcAnalogSensorTrigger<>(
                instanceName + ".analogTrigger", sensor, 0, FtcDistanceSensor.DataType.DISTANCE_INCH,
                new double[]{grabberParams.triggerThreshold}, false, this::analogTriggerEvent);
        }

        grabber = new TrcServoGrabber(instanceName, leftServo, rightServo, grabberParams, analogTrigger);
        grabber.close();
    }   //Grabber

    /**
     * This method returns the TrcServoGrabber object.
     *
     * @return TrcServoGrabber object.
     */
    public TrcServoGrabber getServoGrabber()
    {
        return grabber;
    }   //getServoGrabber

    /**
     * This method is called when an analog sensor threshold has been crossed.
     *
     * @param context specifies the callback context.
     */
    private void analogTriggerEvent(Object context)
    {
        final String funcName = "analogTriggerEvent";
        TrcAnalogSensorTrigger.CallbackContext callbackContext = (TrcAnalogSensorTrigger.CallbackContext) context;

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "Zone=%d->%d, value=%.3f",
                callbackContext.prevZone, callbackContext.currZone, callbackContext.sensorValue);
        }

        if (grabber.isAutoAssistActive() && callbackContext.prevZone != -1)
        {
            boolean inProximity = grabber.objectInProximity();

            if (msgTracer != null)
            {
                msgTracer.traceInfo(funcName, "Trigger: inProximity=%s", inProximity);
            }

            if (inProximity)
            {
                grabber.close();
            }
        }
    }   //analogTriggerEvent

}   //class Grabber