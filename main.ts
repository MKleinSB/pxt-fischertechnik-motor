/* @brief motor drive makecode library for 
 * calliopemini fischertechnik motorboard by Marcel André
 * 
 * @n this library uses parts of https://github.com/DFRobot/pxt-motor
 * @n This is the special motor drive library, which realizes control 
 *    of the eight-channel steering gear, two-step motor and four-way dc motor.
 *
 * @copyright	[DFRobot](http://www.dfrobot.com), 2016 M. Klein 2020
 * @copyright	GNU Lesser General Public License
 */

let lastLightLevel = 0

const enum ftPins {
    C16 = 9,   //C16
    C17 = 15,  //C17
    P1 = 7,   //P1
    P2 = 8,   //P2
    P0 = 19, //P0
    P3 = 23, //P3 
    C4 = 10,  //C4
    C5 = 11, //C5
    C6 = 17, //C6
    C7 = 20,  //C7
    C8 = 21,  //C8
    C9 = 22,  //C9
    C10 = 16, //C10
    C11 = 14, //C11
    C12 = 13 //C12
}
enum pushType {
    //% block="gedrückt"
    down = 4, // PulseValue.High,
    //% block="losgelassen"
    up = 5    // PulseValue.Low
}
const enum zustand {
    //% block="interrupted"
    unterbrochen = 20,
    //% block="not interrupted"
    nicht_unterbrochen = 40
}

let LightEventID = 3100
let Empfindlichkeit = 20

//% weight=100 color=#0080FF  icon="\uf085"
//% block="FT-Motor" category="FT-Motor" 
//% groups='["Pins","Motor","Stepper","Phototransistor"]'

namespace motor {
    const PCA9685_ADDRESS = 0x40
    const MODE1 = 0x00
    const MODE2 = 0x01
    const SUBADR1 = 0x02
    const SUBADR2 = 0x03
    const SUBADR3 = 0x04
    const PRESCALE = 0xFE
    const LED0_ON_L = 0x06
    const LED0_ON_H = 0x07
    const LED0_OFF_L = 0x08
    const LED0_OFF_H = 0x09
    const ALL_LED_ON_L = 0xFA
    const ALL_LED_ON_H = 0xFB
    const ALL_LED_OFF_L = 0xFC
    const ALL_LED_OFF_H = 0xFD

    const STP_CHA_L = 2047
    const STP_CHA_H = 4095

    const STP_CHB_L = 1
    const STP_CHB_H = 2047

    const STP_CHC_L = 1023
    const STP_CHC_H = 3071

    const STP_CHD_L = 3071
    const STP_CHD_H = 1023


    const BYG_CHA_L = 3071
    const BYG_CHA_H = 1023

    const BYG_CHB_L = 1023
    const BYG_CHB_H = 3071

    const BYG_CHC_L = 4095
    const BYG_CHC_H = 2047

    const BYG_CHD_L = 2047
    const BYG_CHD_H = 4095

    /**
     * The user can choose the step motor model.
     */
    export enum Stepper {
        //% block="42"
        Ste1 = 1,
        //% block="28"
        Ste2 = 2
    }

    /**
     * The user can select the 8 steering gear controller.
     */
    export enum Servos {
        S1 = 0x01,
        S2 = 0x02,
        S3 = 0x03,
        S4 = 0x04,
        S5 = 0x05,
        S6 = 0x06,
        S7 = 0x07,
        S8 = 0x08
    }

    /**
     * The user selects the 4-way dc motor.
     */
    export enum Motors {
        M1 = 0x1,
        M2 = 0x2,
        M3 = 0x3,
        M4 = 0x4
    }

    /**
     * The user defines the motor rotation direction.
     */
    export enum Dir {
        //% blockId="CW" block="CW"
        CW = 1,
        //% blockId="CCW" block="CCW"
        CCW = -1,
    }

    /**
     * The user can select a two-path stepper motor controller.
     */
    export enum Steppers {
        M1_M2 = 0x1,
        M3_M4 = 0x2
    }

    let initialized = false

    function i2cWrite(addr: number, reg: number, value: number) {
        let buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2cCmd(addr: number, value: number) {
        let buf2 = pins.createBuffer(1)
        buf2[0] = value
        pins.i2cWriteBuffer(addr, buf2)
    }

    function i2cRead(addr: number, reg: number) {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
        return val;
    }

    function initPCA9685(): void {
        i2cWrite(PCA9685_ADDRESS, MODE1, 0x00)
        setFreq(50);
        initialized = true
    }

    function setFreq(freq: number): void {
        // Constrain the frequency
        let prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        let prescale = prescaleval;//Math.floor(prescaleval + 0.5);
        let oldmode = i2cRead(PCA9685_ADDRESS, MODE1);
        let newmode = (oldmode & 0x7F) | 0x10; // sleep
        i2cWrite(PCA9685_ADDRESS, MODE1, newmode); // go to sleep
        i2cWrite(PCA9685_ADDRESS, PRESCALE, prescale); // set the prescaler
        i2cWrite(PCA9685_ADDRESS, MODE1, oldmode);
        control.waitMicros(5000);
        i2cWrite(PCA9685_ADDRESS, MODE1, oldmode | 0xa1);
    }

    function setPwm(channel: number, on: number, off: number): void {
        if (channel < 0 || channel > 15)
            return;

        let buf3 = pins.createBuffer(5);
        buf3[0] = LED0_ON_L + 4 * channel;
        buf3[1] = on & 0xff;
        buf3[2] = (on >> 8) & 0xff;
        buf3[3] = off & 0xff;
        buf3[4] = (off >> 8) & 0xff;
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf3);
    }


    function setStepper_28(index: number, dir: boolean): void {
        if (index == 1) {
            if (dir) {
                setPwm(4, STP_CHA_L, STP_CHA_H);
                setPwm(6, STP_CHB_L, STP_CHB_H);
                setPwm(5, STP_CHC_L, STP_CHC_H);
                setPwm(7, STP_CHD_L, STP_CHD_H);
            } else {
                setPwm(7, STP_CHA_L, STP_CHA_H);
                setPwm(5, STP_CHB_L, STP_CHB_H);
                setPwm(6, STP_CHC_L, STP_CHC_H);
                setPwm(4, STP_CHD_L, STP_CHD_H);
            }
        } else {
            if (dir) {
                setPwm(0, STP_CHA_L, STP_CHA_H);
                setPwm(2, STP_CHB_L, STP_CHB_H);
                setPwm(1, STP_CHC_L, STP_CHC_H);
                setPwm(3, STP_CHD_L, STP_CHD_H);
            } else {
                setPwm(3, STP_CHA_L, STP_CHA_H);
                setPwm(1, STP_CHB_L, STP_CHB_H);
                setPwm(2, STP_CHC_L, STP_CHC_H);
                setPwm(0, STP_CHD_L, STP_CHD_H);
            }
        }
    }


    function setStepper_42(index: number, dir: boolean): void {
        if (index == 1) {
            if (dir) {
                setPwm(7, BYG_CHA_L, BYG_CHA_H);
                setPwm(6, BYG_CHB_L, BYG_CHB_H);
                setPwm(5, BYG_CHC_L, BYG_CHC_H);
                setPwm(4, BYG_CHD_L, BYG_CHD_H);
            } else {
                setPwm(7, BYG_CHC_L, BYG_CHC_H);
                setPwm(6, BYG_CHD_L, BYG_CHD_H);
                setPwm(5, BYG_CHA_L, BYG_CHA_H);
                setPwm(4, BYG_CHB_L, BYG_CHB_H);
            }
        } else {
            if (dir) {
                setPwm(3, BYG_CHA_L, BYG_CHA_H);
                setPwm(2, BYG_CHB_L, BYG_CHB_H);
                setPwm(1, BYG_CHC_L, BYG_CHC_H);
                setPwm(0, BYG_CHD_L, BYG_CHD_H);
            } else {
                setPwm(3, BYG_CHC_L, BYG_CHC_H);
                setPwm(2, BYG_CHD_L, BYG_CHD_H);
                setPwm(1, BYG_CHA_L, BYG_CHA_H);
                setPwm(0, BYG_CHB_L, BYG_CHB_H);
            }
        }
    }


    /**
	 * Steering gear control function.
     * S1~S8.
     * 0°~180°.
	*/
    //% blockId=motor_servo block="Servo|%index|degree|%degree|°"
    //% weight=100
    //% degree.min=0 degree.max=180
    //% degree.shadow="protractorPicker"
    //% index.fieldEditor="gridpicker" index.fieldOptions.columns=4
    //% group="Motor"
    export function servo(index: Servos, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        // 50hz
        let v_us = (degree * 1800 / 180 + 600) // 0.6ms ~ 2.4ms
        let value = v_us * 4096 / 20000
        setPwm(index + 7, 0, value)
    }

    /**
	 * Execute a motor
     * M1~M4.
     * speed(0~255).
    */
    //% weight=90
    //% blockId=motor_MotorRun block="Motor|%index|dir|%Dir|speed|%speed|%"
    //% speed.min=0 speed.max=100
    //% index.fieldEditor="gridpicker" index.fieldOptions.columns=2
    //% direction.fieldEditor="gridpicker" direction.fieldOptions.columns=2
    //% speed.shadow="speedPicker"
    //% group="Motor"
    export function MotorRun(index: Motors, direction: Dir, speed: number): void {
        if (!initialized) {
            initPCA9685()
        }
        speed = speed * 41 * direction; // map 100 to 4096 (naja 4100)
        if (speed >= 4096) {
            speed = 4095
        }
        if (speed <= -4096) {
            speed = -4095
        }
        if (index > 4 || index <= 0)
            return
        let pn = (4 - index) * 2
        let pp = (4 - index) * 2 + 1
        if (speed >= 0) {
            setPwm(pp, 0, speed)
            setPwm(pn, 0, 0)
        } else {
            setPwm(pp, 0, 0)
            setPwm(pn, 0, -speed)
        }
    }

    /**
	 * Execute a 42BYGH1861A-C step motor(Degree).
     * M1_M2/M3_M4.
    */
    //% weight=80
    //% blockId=motor_stepperDegree_42 block="Stepper 42|%index|dir|%direction|degree|%degree"
    //% index.fieldEditor="gridpicker" index.fieldOptions.columns=2
    //% direction.fieldEditor="gridpicker" direction.fieldOptions.columns=2
    //% group="Stepper"
    export function stepperDegree_42(index: Steppers, direction: Dir, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        // let Degree = Math.abs(degree);
        // Degree = Degree * direction;
        //setFreq(100);
        setStepper_42(index, direction > 0);
        if (degree == 0) {
            return;
        }
        let Degree = Math.abs(degree);
        basic.pause((50000 * Degree) / (360 * 100));  //100hz
        if (index == 1) {
            motorStop(1)
            motorStop(2)
        } else {
            motorStop(3)
            motorStop(4)
        }
        //setFreq(50);
    }

    /**
	 * Execute a 42BYGH1861A-C step motor(Turn).
     * M1_M2/M3_M4.
    */
    //% weight=70
    //% blockId=motor_stepperTurn_42 block="Stepper 42|%index|dir|%direction|turn|%turn"
    //% index.fieldEditor="gridpicker" index.fieldOptions.columns=2
    //% direction.fieldEditor="gridpicker" direction.fieldOptions.columns=2
    //% group="Stepper"
    export function stepperTurn_42(index: Steppers, direction: Dir, turn: number): void {
        if (turn == 0) {
            return;
        }
        let degree = turn * 360;
        stepperDegree_42(index, direction, degree);
    }

    /**
	 * Execute a 28BYJ-48 step motor(Degree).
     * M1_M2/M3_M4.
    */
    //% weight=60
    //% blockId=motor_stepperDegree_28 block="Stepper 28|%index|dir|%direction|degree|%degree"
    //% index.fieldEditor="gridpicker" index.fieldOptions.columns=2
    //% direction.fieldEditor="gridpicker" direction.fieldOptions.columns=2
    //% group="Stepper"
    export function stepperDegree_28(index: Steppers, direction: Dir, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        if (degree == 0) {
            return;
        }
        let Degree2 = Math.abs(degree);
        Degree2 = Degree2 * direction;
        //setFreq(100);
        setStepper_28(index, Degree2 > 0);
        Degree2 = Math.abs(Degree2);
        basic.pause((1000 * Degree2) / 360);
        if (index == 1) {
            motorStop(1)
            motorStop(2)
        } else {
            motorStop(3)
            motorStop(4)
        }
        //setFreq(50);
    }

    /**
	 * Execute a 28BYJ-48 step motor(Turn).
     * M1_M2/M3_M4.
    */
    //% weight=50
    //% blockId=motor_stepperTurn_28 block="Stepper 28|%index|dir|%direction|turn|%turn"
    //% index.fieldEditor="gridpicker" index.fieldOptions.columns=2
    //% direction.fieldEditor="gridpicker" direction.fieldOptions.columns=2
    //% group="Stepper"
    export function stepperTurn_28(index: Steppers, direction: Dir, turn: number): void {
        if (turn == 0) {
            return;
        }
        let degree2 = turn * 360;
        stepperDegree_28(index, direction, degree2);
    }


    /**
	 * Stop the dc motor.
    */
    //% weight=85
    //% blockId=motor_motorStop block="Motor stop|%index"
    //% index.fieldEditor="gridpicker" index.fieldOptions.columns=2 
    //% group="Motor"
    export function motorStop(index: Motors) {
        setPwm((4 - index) * 2, 0, 0);
        setPwm((4 - index) * 2 + 1, 0, 0);
    }

    /**
	 * Stop all motors
    */
    //% weight=84
    //% blockId=motor_motorStopAll block="Motor Stop All"
    //% group="Motor"
    export function motorStopAll(): void {
        for (let idx = 1; idx <= 4; idx++) {
            motorStop(idx);
        }
    }

    /**
    * Check if a specific pin is pressed
    * @param pin to be checked
    */
    //% blockId="PinIsPressed" block="Pin %ftpin|gedrückt"
    // Gridpicker und 5 in eine Reihe
    //% ftpin.fieldEditor="gridpicker" ftpin.fieldOptions.columns=5
    // hohes Gewicht d.h. Block nach oben
    //% weight=95 blockGap=8
    // vorgegebener Pin C16
    //% ftpin.defl=ftPins.C16
    //% group="Pins"
    export function PinIsPressed(ftpin: ftPins): boolean {
        const pin = <DigitalPin><number>ftpin;
        pins.setPull(pin, PinPullMode.PullUp);
        return pins.digitalReadPin(pin) == 0;
    }
    /**
    * Check if a specific pin is released
    * @param pin to be checked
    */
    //% blockId="PinIsReleased" block="Pin %ftpin|losgelassen"
    //% ftpin.fieldEditor="gridpicker" ftpin.fieldOptions.columns=5
    //% weight=94 blockGap=8
    //% group="Pins"
    export function PinIsReleased(ftpin: ftPins): boolean {
        const pin2 = <DigitalPin><number>ftpin;
        pins.setPull(pin2, PinPullMode.PullUp);
        return pins.digitalReadPin(pin2) == 1;
    }
    /**
    * Define a pin to send up/down event messages.
    * only necessary for OnPinPressed and OnPinReleased.
    * Sets the pullmode for the pin Up.
    * @param pin to be used as switch
    */
    //% blockId="PinAsSwitch" block="lege Pin %ftpin| als Schalter fest"
    //% ftpin.fieldEditor="gridpicker" ftpin.fieldOptions.columns=5
    //% weight=83 blockGap=8
    //% group="Pins"
    export function PinAsSwitch(ftpin: ftPins) {
        const pin3 = <DigitalPin><number>ftpin;
        pins.setPull(pin3, PinPullMode.PullUp);
        return;
    }
    /**
    * Do something when one of the pins is pressed.
    * Use PinAsSwitch first!
    * @param pin to be checked
    */
    //% blockId="OnPinPressed" block="wenn Pin %ftpin | gedrückt"
    //% ftpin.fieldEditor="gridpicker" ftpin.fieldOptions.columns=5
    //% weight=82 blockGap=8
    //% group="Pins"
    export function OnPinPressed(ftpin: ftPins, handler: Action) {
        const pin4 = <DigitalPin><number>ftpin;
        //        pins.setPull(pin, PinPullMode.PullUp);
        pins.onPulsed(pin4, <number>pushType.down, handler);
    }

    /**
    * Do something when one of the pins is released.
    * Use PinAsSwitch first!
    * @param pin to be checked
    */
    //% blockId="OnPinReleased" block="wenn Pin %ftpin | losgelassen"
    //% ftpin.fieldEditor="gridpicker" ftpin.fieldOptions.columns=5
    //% weight=81 blockGap=8
    //% group="Pins"
    export function OnPinReleased(ftpin: ftPins, handler: Action) {
        const pin5 = <DigitalPin><number>ftpin;
        //       pins.setPull(pin, PinPullMode.PullUp);
        pins.onPulsed(pin5, <number>pushType.up, handler);
    }

    /**
     * Do something when the phototransistor at the specified pin is interrupted or not.
     * @param pin with phototransistor connected
     * @param LSzustand interrupted or not
     */
    //% group="Phototransistor"
    //% pin.fieldEditor="gridpicker" 
    //% pin.fieldOptions.columns=4
    //% blockId=onLightLevel block="phototransistor at pin %pin | is | %LSzustand"
    export function onLightLevel(pin: AnalogPin, LSzustand: zustand, handler: () => void) {
        control.onEvent(LightEventID + pin + LSzustand, EventBusValue.MICROBIT_EVT_ANY, handler);
        control.inBackground(() => {
            while (true) {
                const LLevel = pins.analogReadPin(pin);
                if ((LLevel > Empfindlichkeit) && (lastLightLevel <= Empfindlichkeit) && (LSzustand == zustand.unterbrochen)) {
                    control.raiseEvent(LightEventID + pin + LSzustand, pin);
                } else if ((LLevel <= Empfindlichkeit) && (lastLightLevel > Empfindlichkeit) && (LSzustand == zustand.nicht_unterbrochen)) {
                    control.raiseEvent(LightEventID + pin + LSzustand, pin);
                }
                basic.pause(200);
                lastLightLevel = LLevel
            }
        })
    }

    /**
    * The read phototransistor block reads the pin that a Phototransistor is 
    * connected to and returns true or false when the lightlevel ist lower or 
    * higher than the lightsensitivity
    * @param pin - is the pin which a phototransistor is connected to
    * @param LSzustand - if interrupted or not
    */
    //% group="Phototransistor"
    //% blockId=readPhototransistor
    //% block="Phototransistor at %pin| is | %LSzustand"

    export function readPhototransistor(pin: AnalogPin, LSzustand: zustand) {
        const LiLevel = pins.analogReadPin(pin);
        let Ergebnis = false;
        if ((LiLevel > Empfindlichkeit) && !(LSzustand == zustand.nicht_unterbrochen)) {
            Ergebnis = true;
        } else if ((LiLevel <= Empfindlichkeit) && !(LSzustand == zustand.unterbrochen)) {
            Ergebnis = true;
        }
        return Ergebnis
    }

    /**
     * Set the sensitivity (analogvalue) of the photocell when 
     * it´s interrupted. Normaly on Calliope Mini it´s
     * not necessary with original 
     * fischertechnik parts. The predefined value is 20.
     * On micro:bit the lightsensitivity should be 500
     *  
     * @param value - (analogvalue)
     */
    //% group="Phototransistor"
    //% value.defl=20
    //% value.min=5 value.max=1023
    //% blockId="SetLightSensitivity" block="set lightsensitivity to %value"
    export function SetLightSensitivity(value: number): void {
        Empfindlichkeit = value;
    }


}
