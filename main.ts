/*
Robô Seguidor de linha versão 0.2.0
*/
const enum DistanceUnit {
    //% block="cm"
    CM = 58, // Duração do eco de ida e volta em microssegundos (uS) para dois centímetros, 343 m/s ao nível do mar e 20°C
    //% block="Polegada"
    INCH = 148, // Duração da viagem de ida e volta do eco em microssegundos (uS) para duas polegadas, 343 m/s ao nível do mar e 20°C
}

//% color="#369ddd"  icon="\uf1b9" block="Seguidor de Linha"
namespace Seguidor_de_Linha {
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

    // HT16K33 commands
    const HT16K33_ADDRESS = 0x70
    const HT16K33_BLINK_CMD = 0x80
    const HT16K33_BLINK_DISPLAYON = 0x01
    const HT16K33_BLINK_OFF = 0
    const HT16K33_BLINK_2HZ = 1
    const HT16K33_BLINK_1HZ = 2
    const HT16K33_BLINK_HALFHZ = 3
    const HT16K33_CMD_BRIGHTNESS = 0xE0

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

    export enum Motors {
        M1A = 0x1,
        M1B = 0x2,
        M2A = 0x3,
        M2B = 0x4
    }

    export enum Steppers {
        M1 = 0x1,
        M2 = 0x2
    }

    export enum SonarVersion {
        V1 = 0x1,
        V2 = 0x2
    }

    export enum Turns {
        //% blockId="T1B4" block="1/4"
        T1B4 = 90,
        //% blockId="T1B2" block="1/2"
        T1B2 = 180,
        //% blockId="T1B0" block="1"
        T1B0 = 360,
        //% blockId="T2B0" block="2"
        T2B0 = 720,
        //% blockId="T3B0" block="3"
        T3B0 = 1080,
        //% blockId="T4B0" block="4"
        T4B0 = 1440,
        //% blockId="T5B0" block="5"
        T5B0 = 1800
    }

    export enum ValueUnit {
        //% block="mm"
        Millimetros,
        //% block="cm"
        Centimetros
    }

    let initialized = false
    let initializedMatrix = false
    let matBuf = pins.createBuffer(17);
    let distanceBuf = 0;

    function i2cwrite(addr: number, reg: number, value: number) {
        let buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2ccmd(addr: number, value: number) {
        let buf = pins.createBuffer(1)
        buf[0] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2cread(addr: number, reg: number) {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
        return val;
    }

    function initPCA9685(): void {
        i2cwrite(PCA9685_ADDRESS, MODE1, 0x00)
        setFreq(50);
        for (let idx = 0; idx < 16; idx++) {
            setPwm(idx, 0, 0);
        }
        initialized = true
    }

    function setFreq(freq: number): void {
        // Constrain the frequency
        let prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        let prescale = prescaleval; //Math.Floor(prescaleval + 0.5);
        let oldmode = i2cread(PCA9685_ADDRESS, MODE1);
        let newmode = (oldmode & 0x7F) | 0x10; // sleep
        i2cwrite(PCA9685_ADDRESS, MODE1, newmode); // go to sleep
        i2cwrite(PCA9685_ADDRESS, PRESCALE, prescale); // set the prescaler
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode);
        control.waitMicros(5000);
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode | 0xa1);
    }

    function setPwm(channel: number, on: number, off: number): void {
        if (channel < 0 || channel > 15)
            return;
        //serial.writeValue("ch", channel)
        //serial.writeValue("on", on)
        //serial.writeValue("off", off)

        let buf = pins.createBuffer(5);
        buf[0] = LED0_ON_L + 4 * channel;
        buf[1] = on & 0xff;
        buf[2] = (on >> 8) & 0xff;
        buf[3] = off & 0xff;
        buf[4] = (off >> 8) & 0xff;
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf);
    }


    function setStepper(index: number, dir: boolean): void {
        if (index == 1) {
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
        } else {
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
        }
    }

    function stopMotor(index: number) {
        setPwm((index - 1) * 2, 0, 0);
        setPwm((index - 1) * 2 + 1, 0, 0);
    }

    function matrixInit() {
        i2ccmd(HT16K33_ADDRESS, 0x21);// turn on oscillator
        i2ccmd(HT16K33_ADDRESS, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (0 << 1));
        i2ccmd(HT16K33_ADDRESS, HT16K33_CMD_BRIGHTNESS | 0xF);
    }

    function matrixShow() {
        matBuf[0] = 0x00;
        pins.i2cWriteBuffer(HT16K33_ADDRESS, matBuf);
    }

    //% blockId=robotbit_motor_run block="Motor|%index|velocidade %speed"
    //% group="Motores" weight=59
    //% speed.min=-255 speed.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function MotorRun(index: Motors, speed: number): void {
        if (!initialized) {
            initPCA9685()
        }
        speed = speed * 16; // map 255 to 4096
        if (speed >= 4096) {
            speed = 4095
        }
        if (speed <= -4096) {
            speed = -4095
        }
        if (index > 4 || index <= 0)
            return
        let pp = (index - 1) * 2
        let pn = (index - 1) * 2 + 1
        if (speed >= 0) {
            setPwm(pp, 0, speed)
            setPwm(pn, 0, 0)
        } else {
            setPwm(pp, 0, 0)
            setPwm(pn, 0, -speed)
        }
    }


    /**
     * Execute two motors at the same time
     * @param motor1 First Motor; eg: M1A, M1B
     * @param speed1 [-255-255] speed of motor; eg: 150, -150
     * @param motor2 Second Motor; eg: M2A, M2B
     * @param speed2 [-255-255] speed of motor; eg: 150, -150
    */
    //% blockId=robotbit_motor_dual block="Motores|%motor1|velocidade %speed1|%motor2|velocidade %speed2"
    //% group="Motores" weight=58
    //% speed1.min=-255 speed1.max=255
    //% speed2.min=-255 speed2.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function MotorRunDual(motor1: Motors, speed1: number, motor2: Motors, speed2: number): void {
        MotorRun(motor1, speed1);
        MotorRun(motor2, speed2);
    }
    /**
     * Execute motores únicos com atraso
     * @param index Motor Index; eg: M1A, M1B, M2A, M2B
     * @param speed [-255-255] speed of motor; eg: 150, -150
     * @param delay seconde delay to stop; eg: 1
    */
    //% blockId=robotbit_motor_rundelay block="Motor|%index|velocidade %speed|espera %delay|s"
    //% group="Motores" weight=57
    //% speed.min=-255 speed.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function MotorRunDelay(index: Motors, speed: number, delay: number): void {
        MotorRun(index, speed);
        basic.pause(delay * 1000);
        MotorRun(index, 0);
    }



    //% blockId=robotbit_stop block="Parar Motor|%index|"
    //% group="Motores" weight=56
    export function MotorStop(index: Motors): void {
        MotorRun(index, 0);
    }

    //% blockId=robotbit_stop_all block="Parando todos os motores"
    //% group="Motores" weight=55
    //% blockGap=50
    export function MotorStopAll(): void {
        if (!initialized) {
            initPCA9685()
        }
        for (let idx = 1; idx <= 4; idx++) {
            stopMotor(idx);
        }
    }

    const MICROBIT_LABCODE_ULTRASONIC_OBJECT_DETECTED_ID = 798;
    const MAX_ULTRASONIC_TRAVEL_TIME = 300 * DistanceUnit.CM;
    const ULTRASONIC_MEASUREMENTS = 3;

    interface UltrasonicRoundTrip {
        ts: number;
        rtt: number;
    }

    interface UltrasonicDevice {
        trig: DigitalPin | undefined;
        roundTrips: UltrasonicRoundTrip[];
        medianRoundTrip: number;
        travelTimeObservers: number[];
    }

    let ultrasonicState: UltrasonicDevice;



    /**
        * Acquiring ultrasonic data
        * @param trig trig pin selection enumeration, eg:DigitalPin.P13
        * @param echo echo pin selection enumeration, eg:DigitalPin.P14
        */
//% group="Ultrassônico"
    //% blockId="labcode_ultrasonico_conectado"
    //% block="Sensor Ultrassônico  TRIG %trig  ECHO %echo Distância unit.CM"
    //% weight=94
    export function readUltrassonic(trig: DigitalPin, echo: DigitalPin): number {
        let data;
        pins.digitalWritePin(trig, 1);
        basic.pause(1);
        pins.digitalWritePin(trig, 0)
        if (pins.digitalReadPin(echo) == 0) {
            pins.digitalWritePin(trig, 0);
            pins.digitalWritePin(trig, 1);
            basic.pause(20);
            pins.digitalWritePin(trig, 0);
            data = pins.pulseIn(echo, PulseValue.High, 500 * 58);
        } else {
            pins.digitalWritePin(trig, 1);
            pins.digitalWritePin(trig, 0);
            basic.pause(20);
            pins.digitalWritePin(trig, 0);
            data = pins.pulseIn(echo, PulseValue.High, 500 * 58)
        }
        data = data / 59;
        if (data <= 0)
            return 0;
        if (data > 500)
            return 500;
        return Math.round(data);
    }





    /**
     * Configures the ultrasonic distance sensor and measures continuously in the background.
     * @param trig pin connected to trig, eg: DigitalPin.P12
     * @param echo pin connected to echo, eg: DigitalPin.P13
     */
    //% group="Ultrassônico"
    //% blockId="labcode_ultrasonico_connectado"
    //% block="Sensor de distancia ultrassônica | Trig em %trig | e Echo em %echo"
    //% trig.fieldEditor="gridpicker"
    //% trig.fieldOptions.columns=4
    //% trig.fieldOptions.tooltips="false"
    //% echo.fieldEditor="gridpicker"
    //% echo.fieldOptions.columns=4
    //% echo.fieldOptions.tooltips="false"
    //% weight=80
    export function connectUltrasonicDistanceSensor(
        trig: DigitalPin,
        echo: DigitalPin
    ): void {
        if (ultrasonicState && ultrasonicState.trig) {
            return;
        }

        if (!ultrasonicState) {
            ultrasonicState = {
                trig: trig,
                roundTrips: [{ ts: 0, rtt: MAX_ULTRASONIC_TRAVEL_TIME }],
                medianRoundTrip: MAX_ULTRASONIC_TRAVEL_TIME,
                travelTimeObservers: [],
            };
        } else {
            ultrasonicState.trig = trig;
        }

        pins.onPulsed(echo, PulseValue.High, () => {
            if (
                pins.pulseDuration() < MAX_ULTRASONIC_TRAVEL_TIME &&
                ultrasonicState.roundTrips.length <= ULTRASONIC_MEASUREMENTS
            ) {
                ultrasonicState.roundTrips.push({
                    ts: input.runningTime(),
                    rtt: pins.pulseDuration(),
                });
            }
        });

        control.inBackground(measureInBackground);
    }

    /**
     * Faça algo quando um objeto for detectado pela primeira vez dentro de um intervalo especificado.
     * @param distance distance to object, eg: 20
     * @param unit unit of distance, eg: DistanceUnit.CM
     * @param handler body code to run when the event is raised
     */
    //% group="Ultrassônico"
    //% blockId=labcode_ultrasonic_on_object_detected
    //% block="Objeto detectado a | %distance | %unit"
    //% weight=69
    export function onUltrasonicObjectDetected(
        distance: number,
        unit: DistanceUnit,
        handler: () => void
    ) {
        if (distance <= 0) {
            return;
        }

        if (!ultrasonicState) {
            ultrasonicState = {
                trig: undefined,
                roundTrips: [{ ts: 0, rtt: MAX_ULTRASONIC_TRAVEL_TIME }],
                medianRoundTrip: MAX_ULTRASONIC_TRAVEL_TIME,
                travelTimeObservers: [],
            };
        }

        const travelTimeThreshold = Math.imul(distance, unit);

        ultrasonicState.travelTimeObservers.push(travelTimeThreshold);

        control.onEvent(
            MICROBIT_LABCODE_ULTRASONIC_OBJECT_DETECTED_ID,
            travelTimeThreshold,
            () => {
                handler();
            }
        );
    }

    /**
     * Retorna a distância até um objeto no intervalo de 1 a 300 centímetros ou até 118 polegadas.
     * O valor máximo é retornado para indicar quando nenhum objeto foi detectado.
     * -1 é retornado quando o dispositivo não está conectado.
     * @param unit unit of distance, eg: DistanceUnit.CM
     */
    //% group="Ultrassônico"
    //% blockId="labcode_ultrasonic_distance"
    //% block="A distância é %unit"
    //% weight=60
    export function getUltrasonicDistance(unit: DistanceUnit): number {
        if (!ultrasonicState) {
            return -1;
        }
        basic.pause(0); // yield to allow background processing when called in a tight loop
        return Math.idiv(ultrasonicState.medianRoundTrip, unit);
    }


    /**
     * Returns `true` if an object is within the specified distance. `false` otherwise
     * @param distance distance to object, eg: 20
     * @param unit unit of distance, eg: DistanceUnit.CM
     */
    //% group="Ultrassônico"
    //% blockId="labcode_ultrasonic_less_than"
    //% block="A distância é menor que | %distance | %unit"
    //% weight=50
    export function isUltrasonicDistanceLessThan(
        distance: number,
        unit: DistanceUnit
    ): boolean {
        if (!ultrasonicState) {
            return false;
        }
        basic.pause(0); // rendimento para permitir o processamento em segundo plano quando chamado em um loop apertado
        return Math.idiv(ultrasonicState.medianRoundTrip, unit) < distance;
    }

    function triggerPulse() {
        // Reseta o pino trigger
        pins.setPull(ultrasonicState.trig, PinPullMode.PullNone);
        pins.digitalWritePin(ultrasonicState.trig, 0);
        control.waitMicros(2);

        // Trigger pulso
        pins.digitalWritePin(ultrasonicState.trig, 1);
        control.waitMicros(10);
        pins.digitalWritePin(ultrasonicState.trig, 0);
    }

    function getMedianRRT(roundTrips: UltrasonicRoundTrip[]) {
        const roundTripTimes = roundTrips.map((urt) => urt.rtt);
        return median(roundTripTimes);
    }

    // Retorna o valor mediano da entrada não vazia
    function median(values: number[]) {
        values.sort((a, b) => {
            return a - b;
        });
        return values[(values.length - 1) >> 1];
    }

    function measureInBackground() {
        const trips = ultrasonicState.roundTrips;
        const TIME_BETWEEN_PULSE_MS = 145;

        while (true) {
            const now = input.runningTime();

            if (trips[trips.length - 1].ts < now - TIME_BETWEEN_PULSE_MS - 10) {
                ultrasonicState.roundTrips.push({
                    ts: now,
                    rtt: MAX_ULTRASONIC_TRAVEL_TIME,
                });
            }

            while (trips.length > ULTRASONIC_MEASUREMENTS) {
                trips.shift();
            }

            ultrasonicState.medianRoundTrip = getMedianRRT(
                ultrasonicState.roundTrips
            );

            for (let i = 0; i < ultrasonicState.travelTimeObservers.length; i++) {
                const threshold = ultrasonicState.travelTimeObservers[i];
                if (threshold > 0 && ultrasonicState.medianRoundTrip <= threshold) {
                    control.raiseEvent(
                        MICROBIT_LABCODE_ULTRASONIC_OBJECT_DETECTED_ID,
                        threshold
                    );
                    // use sinal negativo para indicar que notificamos o evento
                    ultrasonicState.travelTimeObservers[i] = -threshold;
                } else if (
                    threshold < 0 &&
                    ultrasonicState.medianRoundTrip > -threshold
                ) {
                    // o objeto está fora do limite de detecção -> reativar o observador
                    ultrasonicState.travelTimeObservers[i] = -threshold;
                }
            }

            triggerPulse();
            basic.pause(TIME_BETWEEN_PULSE_MS);
        }
    }
    //% group="Linha"
    
    /**
     * Get line sensor state [0-1]
    */
    //% blockId=LineRead
    
    //% block="sensor Digital de Linha|%pin|"
    //% group="Sensores de linha"
    export function detectline(pin: DigitalPin): number {
        return pins.digitalReadPin(pin);
    }
    /**
         * Get line sensor state [0-1023]
        */
    //% blockId=LineAnalogRead

    //% block="Sensor Analógico de Linha|%pin|"
    //% group="Sensores de linha"
    export function detecetlinha(pin: AnalogPin): number {
        return pins.analogReadPin(pin);
    }
    /**
             * Get line sensor state [0-1023]
            */
    //% blockId=LineAnalogleft

    //% block="Sensor Esquerdo Analógico|%pin|"
    //% group="Sensores de linha"
    export function esquerdoanalog(pin: AnalogPin): number {
        return pins.analogReadPin(pin);
    }
    /**
            * Get line sensor state [0-1023]
           */
    //% blockId=LineAnalogRigth
    //% block="Sensor Direito Analógico|%pin|"
    //% group="Sensores de linha"
    export function direitoanalog(pin: AnalogPin): number {
        return pins.analogReadPin(pin);
    }
    /**
            * Get line sensor state [0-1023]
           */
    //% blockId=LineAnalogcenter
    //% block="Sensor central Analógico|%pin|"
    //% group="Sensores de linha"
    export function centralanalog(pin: AnalogPin): number {
        return pins.analogReadPin(pin);
    }
    /**
         * Get line sensor state [0-1]
        */
    //% blockId=Linhaesquerda
     //%param esquerdodig pin connected, eg: DigitalPin.P0
    //% block="sensor Digital Esquerdo|%esquerdodig|"
    //% group="Sensores de linha"
    export function esquerdodig(pin: DigitalPin): number {
        return pins.digitalReadPin(pin);
    }
    /**
        * Get line sensor state [0-1]
       */
    //% blockId=Linhacentral
     //%param centraldig pin connected, eg: DigitalPin.P1
    //% block="sensor Digital Central|%centraldig|"
    //% group="Sensores de linha"
    export function centraldig(pin: DigitalPin): number {
        return pins.digitalReadPin(pin);
    }
    /**
        * Get line sensor state [0-1]
       */
    
    //% blockId=linhadireita
     //%param direitodig direitodig pin connected, eg: DigitalPin.P2
    //% block="sensor Digital Direito|%direitodig|"
           //% group="Sensores de linha"
    export function direitodig(pin: DigitalPin): number {
        return pins.digitalReadPin(pin);
    }
}