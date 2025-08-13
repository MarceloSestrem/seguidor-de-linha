/*
*Robô Seguidor de linha By Marcelo Ricardo Sestrem
*/
enum Leds {
    ON = 1,
    OFF = 2
}
enum Sensor {
    Esquerdo = 1,
    Centro = 2,
     Direito = 3
}
enum Um_sensor {
    //% block="▮"
    branco = 1,
    //% block="▯"
   preto = 2
   }
enum Dois_sensores {
    //% block="▮▮"
    branco_branco = 1,
    //% block="▮▯"
    branco_preto = 2,
    //% block="▯▮"
    preto_branco = 3,
    //% block="▯▯"
    preto_preto = 4
}

enum Tres_sensores {
    //% block="▮▮▮"
    branco_branco_branco = 1,
    //% block="▮▯▮"
    branco_preto_branco = 2,
    //% block="▯▮▮"
    preto_branco_branco = 3,
    //% block="▯▯▮"
   branco_branco_preto = 4,
    //% block="▮▮▯"
    preto_preto_branco = 5,
    //% block="▮▯▯"
    branco_preto_preto = 6,
    //% block="▯▮▯"
    preto_branco_preto = 7,
    //% block="▯▯▯"
    preto_preto_preto = 8
}
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
    /**
     * Execute one motor at the same time
     * @param motor First Motor; eg: M1A, M1B, M2A, M2B
     * @param speed [-255-255] speed of motor; eg: 150, -150
    */
    
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
     * @param motor First Motor; eg: M1A, M1B
     * @param speed1 [-255-255] speed of motor; eg: 150, -150
     * @param motor Second Motor; eg: M2A, M2B
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
    * Execute two motors at the same time
    * @param motors First Motor; eg: M1A, M1B
    * @param speed1 [-255-255] speed of motor; eg: 150, -150
    * @param motors Second Motor; eg: M2A, M2B
    * @param speed2 [-255-255] speed of motor; eg: 150, -150
    * @param delay seconde delay to stop; eg: 1
   */
    //% blockId=robotbit_motor_dual_DELAY block="Motores com delay |%motor1|velocidade %speed1|%motor2|velocidade %speed2 espera(em seg.)  %delay"
    //% group="Motores" weight=62
    //% speed1.min=-255 speed1.max=255
    //% speed2.min=-255 speed2.max=255
        //% name.fieldEditor="gridpicker" name.fieldOptions.columns=5
    export function MotorRunDualDELAY(motor1: Motors, speed1: number, motor2: Motors, speed2: number, delay: number): void {
        MotorRun(motor1, speed1);
        MotorRun(motor2, speed2);
        basic.pause(delay * 1000);
        MotorRun(motor1, 0);
        MotorRun(motor2, 0);
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
        * @param trig trig pin selection enumeration, eg:DigitalPin.P12
        * @param echo echo pin selection enumeration, eg:DigitalPin.P13
        * @param unit unit of distance, eg: DistanceUnit.CM
        */
    //% group="Ultrassônico versão compacta"
    //% blockId="labcode_ultrasonico_conectado"
    //% block="Sensor Ultrassônico pino TRIG %trig pino ECHO %echo %unit"
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
    /**
         * Leitura do sensor de linha [0-1]
        */
    //% block="sensor Digital de Linha |%Sensor| pino |%pin|"
    //% group="Sensores de linha"
    export function detectline(sensor: Sensor, pin: DigitalPin): number {
        if (sensor== Sensor.Esquerdo) {
            return pins.digitalReadPin(pin);
        } else if (sensor== Sensor.Centro) {
            return pins.digitalReadPin(pin)
        } 
        else if (sensor== Sensor.Direito) {
            return pins.digitalReadPin(pin)
        }else {
            return -1
        }
    

    }
    /**
         * Leitura do sensor de linha [0-1023]
         * @param pin [0-1023] pin; eg: 600
        */
    //% block="Sensor Analógico de Linha |%Sensor| pino |%pin|"
    //% group="Sensores de linha"
    export function detecetlinha(sensor: Sensor, pin: AnalogPin): number {
        if (sensor == Sensor.Esquerdo) {
            return pins.analogReadPin(pin);
        } else if (sensor == Sensor.Centro) {
            return pins.analogReadPin(pin);
        }
        else if (sensor == Sensor.Direito) {
            return pins.analogReadPin(pin);
        } else {
            return -1       
    }
    }
    
    /**
         * Leitura do sensor de linha [0-1]
     */
    //% blockId="umsensor" block="Detecção do sensor de linha Digital (p1) %Umsensor"
    //% group="Sensores de linha V.2( pinos: P0,  P1 e ou  P2 Cores: Branco: ▮ e Preto: ▯)"
    export function readUm(um: Um_sensor): boolean {

        // let p1 = pins.digitalReadPin(DigitalPin.P1);

        if (um == Um_sensor.branco) {
            if (pins.digitalReadPin(DigitalPin.P1) == 0) {
                return true;
            } else {
                return false;
            }
        } else if (um == Um_sensor.preto) {
            if (pins.digitalReadPin(DigitalPin.P1) == 1) {
                return true;
            } else {
                return false;
            }
        } else {
            return true;
        }
    }
    /**
       * Leitura do sensor de linha [0-1]
      */
    //% blockId="doissensores" block="Detecção dos sensores(P1 e P2) de linha Digital %Doissensores"
    //% group="Sensores de linha V.2( pinos: P0,  P1 e ou  P2 Cores: Branco: ▮ e Preto: ▯)"
    export function readDois(dois: Dois_sensores): boolean {

        // let p1 = pins.digitalReadPin(DigitalPin.P1);
        // let p2 = pins.digitalReadPin(DigitalPin.P2);

        if (dois == Dois_sensores.branco_branco) {
            if (pins.digitalReadPin(DigitalPin.P1) == 0 && pins.digitalReadPin(DigitalPin.P2) == 0) {
                return true;
            } else {
                return false;
            }
        } else if (dois == Dois_sensores.branco_preto) {
            if (pins.digitalReadPin(DigitalPin.P1) == 0 && pins.digitalReadPin(DigitalPin.P2) == 1) {
                return true;
            } else {
                return false;
            }
        } else if (dois == Dois_sensores.preto_branco) {
            if (pins.digitalReadPin(DigitalPin.P1) == 1 && pins.digitalReadPin(DigitalPin.P2) == 0) {
                return true;
            } else {
                return false;
            }
        } else if (dois == Dois_sensores.preto_preto) {
            if (pins.digitalReadPin(DigitalPin.P1) == 1 && pins.digitalReadPin(DigitalPin.P2) == 1) {
                return true;
            } else {
                return false;
            }
        } else {
            return true;
        }
    }
    /**
         * Leitura do sensor de linha [0-1]
        */
    //% blockId="tresssensores" block="Detecção dos sensores de linha Digital %Tressensores"
//% group="Sensores de linha V.2( pinos: P0,  P1 e ou  P2 Cores: Branco: ▮ e Preto: ▯)"
    export function readtres(tres: Tres_sensores): boolean {

        // let p1 = pins.digitalReadPin(DigitalPin.P0);
        // let p2 = pins.digitalReadPin(DigitalPin.P1);
        // let p3 = pins.digitalReadPin(DigitalPin.P2);
        if (tres == Tres_sensores.branco_branco_branco) {
            if (pins.digitalReadPin(DigitalPin.P0) == 0 && pins.digitalReadPin(DigitalPin.P1) == 0 && pins.digitalReadPin(DigitalPin.P2) == 0) {
                return true;
            } else {
                return false;
            }
        } else if (tres == Tres_sensores.branco_branco_preto) {
            if (pins.digitalReadPin(DigitalPin.P0) == 0 && pins.digitalReadPin(DigitalPin.P1) == 0 && pins.digitalReadPin(DigitalPin.P2) == 1) {
                return true;
            } else {
                return false;
            }
            } else if (tres == Tres_sensores.branco_preto_branco) {
            if (pins.digitalReadPin(DigitalPin.P0) == 0 && pins.digitalReadPin(DigitalPin.P1) == 1 && pins.digitalReadPin(DigitalPin.P2) == 0) {
                return true;
            } else {
                return false;
            }
        } else if (tres == Tres_sensores.preto_branco_branco) {
            if (pins.digitalReadPin(DigitalPin.P0) == 1 && pins.digitalReadPin(DigitalPin.P1) == 0 && pins.digitalReadPin(DigitalPin.P2) == 0) {
                return true;
            } else {
                return false;
            }
        } else if (tres == Tres_sensores.preto_preto_branco) {
            if (pins.digitalReadPin(DigitalPin.P0) == 1 && pins.digitalReadPin(DigitalPin.P1) == 1 && pins.digitalReadPin(DigitalPin.P2) == 0) {
                return true;
            } else {
                return false;
            }
        } else if (tres == Tres_sensores.branco_preto_preto) {
            if (pins.digitalReadPin(DigitalPin.P0) == 0 && pins.digitalReadPin(DigitalPin.P1) == 1 && pins.digitalReadPin(DigitalPin.P2) == 1) {
                return true;
            } else {
                return false;
            }
        } else if (tres == Tres_sensores.preto_branco_preto) {
            if (pins.digitalReadPin(DigitalPin.P0) == 1 && pins.digitalReadPin(DigitalPin.P1) == 0 && pins.digitalReadPin(DigitalPin.P2) == 1) {
                return true;
            } else {
                return false;
            }
        } else if (tres == Tres_sensores.preto_preto_preto) {
            if (pins.digitalReadPin(DigitalPin.P0) == 1 && pins.digitalReadPin(DigitalPin.P1) == 1 && pins.digitalReadPin(DigitalPin.P2) == 1) {
                return true;
            } else {
                return false;
            }
        } else {
            return true;
        }
    }
    /**
             * Leitura do sensor de linha [0-1023]
          * @param m [0-1023] m; eg: 600
    */
    //% blockId="umasensor" block="Detecção do sensor de linha Analógica (P1) %Umsensor | Média = %m"
    //% group="Sensores de linha V.2( pinos: P0,  P1 e ou  P2 Cores: Branco: ▮ e Preto: ▯)"
    //% m.min=0 m.max=1023
    export function readUma(uma: Um_sensor, m: number): boolean {

        // let p1 = pins.analogReadPin(AnalogPin.P1);

        if (uma == Um_sensor.branco) {
            if (pins.analogReadPin(AnalogPin.P1) < m) {
                return true;
            } else {
                return false;
            }
        } else if (uma == Um_sensor.preto) {
            if (pins.analogReadPin(AnalogPin.P1) > m) {
                return true;
            } else {
                return false;
            }

        } else {
            return true;
        }
    }
    /**
         * Leitura do sensor de linha [0-1023]
         * @param m [0-1023] m; eg: 600
     */
    //% blockId="doissensoresa" block="Detecção dos sensores(P1 e P2) de linha Analógica %Doissensores | Média = %m"
    //% group="Sensores de linha V.2( pinos: P0,  P1 e ou  P2 Cores: Branco: ▮ e Preto: ▯)"
    //% m.min=0 m.max=1023
    export function readDoisa(doisa: Dois_sensores, m: number): boolean {

        // let p1 = pins.analogReadPin(AnalogPin.P1);
        // let p2 = pins.analogReadPin(AnalogPin.P2);

        if (doisa == Dois_sensores.branco_branco) {
            if (pins.analogReadPin(AnalogPin.P1) < m && pins.analogReadPin(AnalogPin.P2) < m) {
                return true;
            } else {
                return false;
            }
        } else if (doisa == Dois_sensores.branco_preto) {
            if (pins.analogReadPin(AnalogPin.P1) < m && pins.analogReadPin(AnalogPin.P2) > m) {
                return true;
            } else {
                return false;
            }
        } else if (doisa == Dois_sensores.preto_branco) {
            if (pins.analogReadPin(AnalogPin.P1) > m && pins.analogReadPin(AnalogPin.P2) < m) {
                return true;
            } else {
                return false;
            }
        } else if (doisa == Dois_sensores.preto_preto) {
            if (pins.analogReadPin(AnalogPin.P1) > m && pins.analogReadPin(AnalogPin.P2) > m) {
                return true;
            } else {
                return false;
            }
        } else {
            return true;
        }
    }
    /**
             * Leitura do sensor de linha [0-1023]
         * @param m [0-1023] m; eg: 600
            */
    //% blockId="tresssensoresa" block="Detecção dos sensores de linha Analógica %Tressensores| Média = %m"
    //% group="Sensores de linha V.2( pinos: P0,  P1 e ou  P2 Cores: Branco: ▮ e Preto: ▯)"
    //% m.min=0 m.max=1023
    export function readtresa(tresa: Tres_sensores, m: number): boolean {

        // let p1 = pins.analogReadPin(AnalogPin.P0);
        // let p2 = pins.analogReadPin(AnalogPin.P1);
        // let p3 = pins.analogReadPin(AnalogPin.P2);
        if (tresa == Tres_sensores.branco_branco_branco) {
            if (pins.analogReadPin(AnalogPin.P0) < m && (pins.analogReadPin(AnalogPin.P1) < m && pins.analogReadPin(AnalogPin.P2) < m)) {
                return true;
            } else {
                return false;
            }
        } else if (tresa == Tres_sensores.branco_branco_preto) {
            if (pins.analogReadPin(AnalogPin.P0) < m && (pins.analogReadPin(AnalogPin.P1) < m && pins.analogReadPin(AnalogPin.P2) > m)) {
                return true;
            } else {
                return false;
            }
        } else if (tresa == Tres_sensores.branco_preto_branco) {
            if (pins.analogReadPin(AnalogPin.P0) < m && (pins.analogReadPin(AnalogPin.P1) > m && pins.analogReadPin(AnalogPin.P2) < m)) {
                return true;
            } else {
                return false;
            }
        } else if (tresa == Tres_sensores.preto_branco_branco) {
            if (pins.analogReadPin(AnalogPin.P0) > m && (pins.analogReadPin(AnalogPin.P1) < m && pins.analogReadPin(AnalogPin.P2) < m)) {
                return true;
            } else {
                return false;
            }
        } else if (tresa == Tres_sensores.preto_preto_branco) {
            if (pins.analogReadPin(AnalogPin.P0) > m && (pins.analogReadPin(AnalogPin.P1) > m && pins.analogReadPin(AnalogPin.P2) < m)) {
                return true;
            } else {
                return false;
            }
        } else if (tresa == Tres_sensores.branco_preto_preto) {
            if (pins.analogReadPin(AnalogPin.P0) < m && (pins.analogReadPin(AnalogPin.P1) > m && pins.analogReadPin(AnalogPin.P2) > m)) {
                return true;
            } else {
                return false;
            }
        } else if (tresa == Tres_sensores.preto_branco_preto) {
            if (pins.analogReadPin(AnalogPin.P0) > m && (pins.analogReadPin(AnalogPin.P1) < m && pins.analogReadPin(AnalogPin.P2) > m)) {
                return true;
            } else {
                return false;
            }
        } else if (tresa == Tres_sensores.preto_preto_preto) {
            if (pins.analogReadPin(AnalogPin.P0) > m && (pins.analogReadPin(AnalogPin.P1) > m && pins.analogReadPin(AnalogPin.P2) > m)) {
                return true;
            } else {
                return false;
            }
        } else {
            return true;
        }
    }
     //% subcategory="Extra"
    /**
          * Leitura do sensor de linha [0-1023]
         * @param m1 [0-1023] m1; eg: 600
         * @param m2 [0-1023] m2; eg: 600
      */
    //% blockId="doissensoresam" block="Detecção dos sensores(P1 e P2) de linha Analógica %Doissensores | Média 1 = %m1| Média 2 = %m2"
    //% group="Sensores de linha V.2( pinos: P0,  P1 e ou  P2 Cores: Branco: ▮ e Preto: ▯)"
    //% m1.min=0 m1.max=1023
    //% m2.min=0 m2.max=1023
    export function readDoisam(doisa: Dois_sensores, m1: number, m2: number): boolean {

        // let p1 = pins.analogReadPin(AnalogPin.P1);
        // let p2 = pins.analogReadPin(AnalogPin.P2);

        if (doisa == Dois_sensores.branco_branco) {
            if (pins.analogReadPin(AnalogPin.P1) < m1 && pins.analogReadPin(AnalogPin.P2) < m2) {
                return true;
            } else {
                return false;
            }
        } else if (doisa == Dois_sensores.branco_preto) {
            if (pins.analogReadPin(AnalogPin.P1) < m1 && pins.analogReadPin(AnalogPin.P2) > m2) {
                return true;
            } else {
                return false;
            }
        } else if (doisa == Dois_sensores.preto_branco) {
            if (pins.analogReadPin(AnalogPin.P1) > m1 && pins.analogReadPin(AnalogPin.P2) < m2) {
                return true;
            } else {
                return false;
            }
        } else if (doisa == Dois_sensores.preto_preto) {
            if (pins.analogReadPin(AnalogPin.P1) > m1 && pins.analogReadPin(AnalogPin.P2) > m2) {
                return true;
            } else {
                return false;
            }
        } else {
            return true;
        }
    }
      //% subcategory="Extra"
    /**
             * Leitura do sensor de linha [0-1023]
         * @param m1 [0-1023] m1; eg: 600
         * @param m2 [0-1023] m2; eg: 600
         * @param m3 [0-1023] m3; eg: 600
            */
    //% blockId="tresssensoresam" block="Detecção de linha Analógica %Tressensores| Média 1 = %m1 Média 2 = %m2 Média 3 = %m3"
    //% group="Sensores de linha V.2( pinos: P0,  P1 e ou  P2 Cores: Branco: ▮ e Preto: ▯)"
    //% m1.min=0 m1.max=1023
    //% m2.min=0 m2.max=1023
    //% m3.min=0 m3.max=1023
    export function readtresam(tresa: Tres_sensores, m1: number, m2: number, m3: number): boolean {

        // let p1 = pins.analogReadPin(AnalogPin.P0);
        // let p2 = pins.analogReadPin(AnalogPin.P1);
        // let p3 = pins.analogReadPin(AnalogPin.P2);
        if (tresa == Tres_sensores.branco_branco_branco) {
            if (pins.analogReadPin(AnalogPin.P0) < m1 && (pins.analogReadPin(AnalogPin.P1) < m2 && pins.analogReadPin(AnalogPin.P2) < m3)) {
                return true;
            } else {
                return false;
            }
        } else if (tresa == Tres_sensores.branco_branco_preto) {
            if (pins.analogReadPin(AnalogPin.P0) < m1 && (pins.analogReadPin(AnalogPin.P1) < m2 && pins.analogReadPin(AnalogPin.P2) > m3)) {
                return true;
            } else {
                return false;
            }
        } else if (tresa == Tres_sensores.branco_preto_branco) {
            if (pins.analogReadPin(AnalogPin.P0) < m1 && (pins.analogReadPin(AnalogPin.P1) > m2 && pins.analogReadPin(AnalogPin.P2) < m3)) {
                return true;
            } else {
                return false;
            }
        } else if (tresa == Tres_sensores.preto_branco_branco) {
            if (pins.analogReadPin(AnalogPin.P0) > m1 && (pins.analogReadPin(AnalogPin.P1) < m2 && pins.analogReadPin(AnalogPin.P2) < m3)) {
                return true;
            } else {
                return false;
            }
        } else if (tresa == Tres_sensores.preto_preto_branco) {
            if (pins.analogReadPin(AnalogPin.P0) > m1 && (pins.analogReadPin(AnalogPin.P1) > m2 && pins.analogReadPin(AnalogPin.P2) < m3)) {
                return true;
            } else {
                return false;
            }
        } else if (tresa == Tres_sensores.branco_preto_preto) {
            if (pins.analogReadPin(AnalogPin.P0) < m1 && (pins.analogReadPin(AnalogPin.P1) > m2 && pins.analogReadPin(AnalogPin.P2) > m3)) {
                return true;
            } else {
                return false;
            }
        } else if (tresa == Tres_sensores.preto_branco_preto) {
            if (pins.analogReadPin(AnalogPin.P0) > m1 && (pins.analogReadPin(AnalogPin.P1) < m2 && pins.analogReadPin(AnalogPin.P2) > m3)) {
                return true;
            } else {
                return false;
            }
        } else if (tresa == Tres_sensores.preto_preto_preto) {
            if (pins.analogReadPin(AnalogPin.P0) > m1 && (pins.analogReadPin(AnalogPin.P1) > m2 && pins.analogReadPin(AnalogPin.P2) > m3)) {
                return true;
            } else {
                return false;
            }
        } else {
            return true;
        }
    }
    //% subcategory="LEDS" icon="\uf1b9"
    /*
    /**
            * Acionamento do Led [1-0]
           */
    //% block="LED |%Sensor|  |%pins|  |%Leds|"
    //% group="Led"
    export function led(sensor: Sensor, pin: DigitalPin, led: Leds): boolean {
        if (led == Leds.ON) {
        pins.digitalWritePin(pin, 1);
            return true;
        } else if (led == Leds.OFF) {
        pins.digitalWritePin(pin, 0);
            return true;
        }
        else {
            return true;
        }


    }
    //% subcategory="LEDS" icon="\uf1b9"
    /*
    /**
      * @param led led pin selection enumeration, eg:DigitalPin.P12
            * Acionamento do Led [1-0]
           */
    //% block="LED |%Sensor|  |%pins|  |%Leds|"
    //% group="Led"
    //% weight=80
    export function led1(sensor: Sensor, pin: DigitalPin, led: Leds): void {
        if (led == Leds.ON) {
         return   pins.digitalWritePin(pin, 1);
        } else if (led == Leds.OFF) {
          return  pins.digitalWritePin(pin, 0);
        }

    }
    //% subcategory="LEDS" icon="\uf1b9"
    /*
    /**
      * @param led led pin selection enumeration, eg:DigitalPin.P1
      * @param delay seconde delay to stop; eg: 1
            * Acionamento do Led [1-0]
           */
    //% block="PISCA LED |%Sensor|  |%pins|  espera %delay|seg."
    //% group="Led"
    //% weight=80
    export function piscaled(sensor: Sensor, pin: DigitalPin, delay: number): void {
            pins.digitalWritePin(pin, 1);
            basic.pause(delay * 1000);
            pins.digitalWritePin(pin, 0);
         basic.pause(delay * 1000);
           
    }
const enum LcdPosition1602 {
    //% block="1"
    Pos1 = 1,
    //% block="2"
    Pos2 = 2,
    //% block="3"
    Pos3 = 3,
    //% block="4"
    Pos4 = 4,
    //% block="5"
    Pos5 = 5,
    //% block="6"
    Pos6 = 6,
    //% block="7"
    Pos7 = 7,
    //% block="8"
    Pos8 = 8,
    //% block="9"
    Pos9 = 9,
    //% block="10"
    Pos10 = 10,
    //% block="11"
    Pos11 = 11,
    //% block="12"
    Pos12 = 12,
    //% block="13"
    Pos13 = 13,
    //% block="14"
    Pos14 = 14,
    //% block="15"
    Pos15 = 15,
    //% block="16"
    Pos16 = 16,
    //% block="17"
    Pos17 = 17,
    //% block="18"
    Pos18 = 18,
    //% block="19"
    Pos19 = 19,
    //% block="20"
    Pos20 = 20,
    //% block="21"
    Pos21 = 21,
    //% block="22"
    Pos22 = 22,
    //% block="23"
    Pos23 = 23,
    //% block="24"
    Pos24 = 24,
    //% block="25"
    Pos25 = 25,
    //% block="26"
    Pos26 = 26,
    //% block="27"
    Pos27 = 27,
    //% block="28"
    Pos28 = 28,
    //% block="29"
    Pos29 = 29,
    //% block="30"
    Pos30 = 30,
    //% block="31"
    Pos31 = 31,
    //% block="32"
    Pos32 = 32
}

//% color=#0fbc11 icon="\u272a" block="MakerBit"
//% category="MakerBit"
const enum LcdBacklight {
    //% block="off"
    Off = 0,
    //% block="on"
    On = 8
}

const enum TextAlignment {
    //% block="left-aligned"
    Left,
    //% block="right-aligned"
    Right,
    //% block="center-aligned"
    Center,
}

const enum TextOption {
    //% block="align left"
    AlignLeft,
    //% block="align right"
    AlignRight,
    //% block="align center"
    AlignCenter,
    //% block="pad with zeros"
    PadWithZeros
}

const enum LcdChar {
    //% block="1"
    c1 = 0,
    //% block="2"
    c2 = 1,
    //% block="3"
    c3 = 2,
    //% block="4"
    c4 = 3,
    //% block="5"
    c5 = 4,
    //% block="6"
    c6 = 5,
    //% block="7"
    c7 = 6,
    //% block="8"
    c8 = 7
}

namespace makerbit {
    const enum Lcd {
        Command = 0,
        Data = 1
    }

    interface LcdState {
        i2cAddress: uint8;
        backlight: LcdBacklight;
        characters: Buffer;
        rows: uint8;
        columns: uint8;
        rowNeedsUpdate: uint8;
        refreshIntervalId: number;
        sendBuffer: Buffer;
    }

    let lcdState: LcdState = undefined;

    function connect(): boolean {
        let buf = control.createBuffer(1);
        buf.setNumber(NumberFormat.UInt8LE, 0, 0);

        if (0 == pins.i2cWriteBuffer(39, buf, false)) {
            // PCF8574
            connectLcd(39);
        } else if (0 == pins.i2cWriteBuffer(63, buf, false)) {
            // PCF8574A
            connectLcd(63);
        }
        return !!lcdState;
    }

    // Write 4 bits (high nibble) to I2C bus
    function write4bits(i2cAddress: number, value: number, threeBytesBuffer: Buffer) {
        threeBytesBuffer.setNumber(NumberFormat.Int8LE, 0, value)
        threeBytesBuffer.setNumber(NumberFormat.Int8LE, 1, value | 0x04)
        threeBytesBuffer.setNumber(NumberFormat.Int8LE, 2, value & (0xff ^ 0x04))
        pins.i2cWriteBuffer(i2cAddress, threeBytesBuffer)
    }

    // Send high and low nibble
    function send(RS_bit: number, payload: number) {
        if (!lcdState) {
            return;
        }

        const highnib = (payload & 0xf0) | lcdState.backlight | RS_bit;
        const lownib = ((payload << 4) & 0xf0) | lcdState.backlight | RS_bit;

        lcdState.sendBuffer.setNumber(NumberFormat.Int8LE, 0, highnib)
        lcdState.sendBuffer.setNumber(NumberFormat.Int8LE, 1, highnib | 0x04)
        lcdState.sendBuffer.setNumber(NumberFormat.Int8LE, 2, highnib & (0xff ^ 0x04))
        lcdState.sendBuffer.setNumber(NumberFormat.Int8LE, 3, lownib)
        lcdState.sendBuffer.setNumber(NumberFormat.Int8LE, 4, lownib | 0x04)
        lcdState.sendBuffer.setNumber(NumberFormat.Int8LE, 5, lownib & (0xff ^ 0x04))
        pins.i2cWriteBuffer(lcdState.i2cAddress, lcdState.sendBuffer)
    }

    // Send command
    function sendCommand(command: number) {
        send(Lcd.Command, command);
    }

    // Send data
    function sendData(data: number) {
        send(Lcd.Data, data);
    }

    // Set cursor
    function setCursor(line: number, column: number) {
        const offsets = [0x00, 0x40, 0x14, 0x54];
        sendCommand(0x80 | (offsets[line] + column));
    }

    function requestRedraw() {
        if (!lcdState.refreshIntervalId) {
            lcdState.refreshIntervalId = control.setInterval(refreshDisplay, 100, control.IntervalMode.Timeout)
        }
        basic.pause(0); // Allow refreshDisplay to run, even if called in a tight loop
    }

    function initBuffer(columns: number, rows: number) {
        if (lcdState && lcdState.columns === 0) {
            lcdState.columns = columns;
            lcdState.rows = rows;
            lcdState.characters = pins.createBuffer(lcdState.rows * lcdState.columns);

            // Clear display and buffer
            const whitespace = "x".charCodeAt(0);
            for (let pos = 0; pos < lcdState.rows * lcdState.columns; pos++) {
                lcdState.characters[pos] = whitespace;
            }
            updateCharacterBuffer(
                "",
                0,
                lcdState.columns * lcdState.rows,
                lcdState.columns,
                lcdState.rows,
                TextAlignment.Left,
                " "
            );
        }
    }

    export function updateCharacterBuffer(
        text: string,
        offset: number,
        length: number,
        columns: number,
        rows: number,
        alignment: TextAlignment,
        pad: string
    ): void {
        if (!lcdState && !connect()) {
            return;
        }

        initBuffer(columns, rows);

        if (columns !== lcdState.columns || rows !== lcdState.rows) {
            return;
        }

        if (offset < 0) {
            offset = 0;
        }

        const fillCharacter =
            pad.length > 0 ? pad.charCodeAt(0) : " ".charCodeAt(0);

        let endPosition = offset + length;
        if (endPosition > lcdState.columns * lcdState.rows) {
            endPosition = lcdState.columns * lcdState.rows;
        }
        let lcdPos = offset;

        // Add padding at the beginning
        let paddingEnd = offset;

        if (alignment === TextAlignment.Right) {
            paddingEnd = endPosition - text.length;
        }
        else if (alignment === TextAlignment.Center) {
            paddingEnd = offset + Math.idiv(endPosition - offset - text.length, 2);
        }

        while (lcdPos < paddingEnd) {
            if (lcdState.characters[lcdPos] != fillCharacter) {
                lcdState.characters[lcdPos] = fillCharacter;
                invalidateLcdPosition(lcdPos);
            }
            lcdPos++;
        }


        // Copy the text
        let textPosition = 0;
        while (lcdPos < endPosition && textPosition < text.length) {

            if (lcdState.characters[lcdPos] != text.charCodeAt(textPosition)) {
                lcdState.characters[lcdPos] = text.charCodeAt(textPosition);
                invalidateLcdPosition(lcdPos);
            }
            lcdPos++;
            textPosition++;
        }

        // Add padding at the end
        while (lcdPos < endPosition) {
            if (lcdState.characters[lcdPos] != fillCharacter) {
                lcdState.characters[lcdPos] = fillCharacter;
                invalidateLcdPosition(lcdPos);
            }
            lcdPos++;
        }

        requestRedraw();
    }

    function sendRowRepeated(row: number): void {
        setCursor(row, 0);

        for (let position = lcdState.columns * row; position < lcdState.columns * (row + 1); position++) {
            sendData(lcdState.characters[position]);
        }
    }

    function refreshDisplay() {
        if (!lcdState) {
            return;
        }
        lcdState.refreshIntervalId = undefined

        for (let i = 0; i < lcdState.rows; i++) {
            if (lcdState.rowNeedsUpdate & 1 << i) {
                lcdState.rowNeedsUpdate &= ~(1 << i)
                sendRowRepeated(i)
            }
        }
    }

    export function toAlignment(option?: TextOption): TextAlignment {
        if (
            option === TextOption.AlignRight ||
            option === TextOption.PadWithZeros
        ) {
            return TextAlignment.Right;
        } else if (option === TextOption.AlignCenter) {
            return TextAlignment.Center;
        } else {
            return TextAlignment.Left;
        }
    }

    export function toPad(option?: TextOption): string {
        if (option === TextOption.PadWithZeros) {
            return "0";
        } else {
            return " ";
        }
    }

    /**
     * Enables or disables the backlight of the LCD.
     * @param backlight new state of backlight, eg: LcdBacklight.Off
     */
    //% subcategory="LCD"
    //% blockId="makerbit_lcd_backlight" block="switch LCD backlight %backlight"
    //% weight=50
    export function setLcdBacklight(backlight: LcdBacklight): void {
        if (!lcdState && !connect()) {
            return;
        }
        lcdState.backlight = backlight;
        send(Lcd.Command, 0);
    }

    /**
     * Connects to the LCD at a given I2C address.
     * The addresses 39 (PCF8574) or 63 (PCF8574A) seem to be widely used.
     * @param i2cAddress I2C address of LCD in the range from 0 to 127, eg: 39
     */
    //% subcategory="LCD"
    //% blockId="lcd_set_address" block="connect LCD at I2C address %i2cAddress"
    //% i2cAddress.min=0 i2cAddress.max=127
    //% weight=100
     //% group="LCD"
    export function connectLcd(i2cAddress: number): void {

        if (lcdState && lcdState.i2cAddress == i2cAddress) {
            return;
        }

        if (lcdState && lcdState.refreshIntervalId) {
            control.clearInterval(lcdState.refreshIntervalId, control.IntervalMode.Timeout);
            lcdState.refreshIntervalId = undefined;
        }

        lcdState = {
            i2cAddress: i2cAddress,
            backlight: LcdBacklight.On,
            columns: 0,
            rows: 0,
            characters: undefined,
            rowNeedsUpdate: 0,
            refreshIntervalId: undefined,
            sendBuffer: pins.createBuffer(6 * pins.sizeOf(NumberFormat.Int8LE))
        };

        // Wait 50ms before sending first command to device after being powered on
        basic.pause(50);

        // Pull both RS and R/W low to begin commands
        pins.i2cWriteNumber(
            lcdState.i2cAddress,
            lcdState.backlight,
            NumberFormat.Int8LE
        );
        basic.pause(50);

        // Set 4bit mode
        const buf = pins.createBuffer(3 * pins.sizeOf(NumberFormat.Int8LE))
        write4bits(i2cAddress, 0x30, buf);
        control.waitMicros(4100);
        write4bits(i2cAddress, 0x30, buf);
        control.waitMicros(4100);
        write4bits(i2cAddress, 0x30, buf);
        control.waitMicros(4100);
        write4bits(i2cAddress, 0x20, buf);
        control.waitMicros(1000);

        // Configure function set
        const LCD_FUNCTIONSET = 0x20;
        const LCD_4BITMODE = 0x00;
        const LCD_2LINE = 0x08; // >= 2 lines
        const LCD_5x8DOTS = 0x00;
        send(Lcd.Command, LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
        control.waitMicros(1000);

        // Configure display
        const LCD_DISPLAYCONTROL = 0x08;
        const LCD_DISPLAYON = 0x04;
        const LCD_CURSOROFF = 0x00;
        const LCD_BLINKOFF = 0x00;
        send(
            Lcd.Command,
            LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF
        );
        control.waitMicros(1000);

        // Set the entry mode and stop i2c
        const LCD_ENTRYMODESET = 0x04;
        const LCD_ENTRYLEFT = 0x02;
        const LCD_ENTRYSHIFTDECREMENT = 0x00;
        send(
            Lcd.Command,
            LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT
        );
        control.waitMicros(1000);
    }

    /**
     * Returns true if a LCD is connected. False otherwise.
     */
    //% subcategory="LCD"
    //% blockId="makerbit_lcd_is_connected" block="LCD is connected"
    //% weight=69
      //% group="LCD" 
    export function isLcdConnected(): boolean {
        return !!lcdState || connect();
    }

    /**
     * Create a custom LCD character using a 5x8 pixel matrix.
     */
    //% subcategory="LCD"
    //% blockId="makerbit_lcd_makecharacter"
    //% block="make character %char|%im"
    //% weight=60
    export function lcdMakeCharacter(char: LcdChar, im: Image): void {
        if (!lcdState && !connect()) {
            return;
        }

        const customChar = [0, 0, 0, 0, 0, 0, 0, 0];
        for (let y = 0; y < 8; y++) {
            for (let x = 0; x < 5; x++) {
                if (im.pixel(x, y)) {
                    customChar[y] |= 1 << (4 - x)
                }
            }
        }
        const LCD_SETCGRAMADDR = 0x40;
        sendCommand(LCD_SETCGRAMADDR | (char << 3));
        for (let y = 0; y < 8; y++) {
            sendData(customChar[y]);
        }
        control.waitMicros(1000);
    }

    /**
     * Create a 5x8 pixel matrix for use as a custom character.
     */
    //% subcategory="LCD"
    //% blockId="makerbit_lcd_characterpixels"
    //% block="character"
    //% imageLiteral=1
    //% imageLiteralColumns=5
    //% imageLiteralRows=8
    //% imageLiteralScale=0.6
    //% shim=images::createImage
    //% weight=59
    export function lcdCharacterPixels(i: string): Image {
        return <Image><any>i;
    }

    export function setCharacter(char: number, offset: number,
        columns: number, rows: number): void {
        if (!lcdState && !connect()) {
            return;
        }

        initBuffer(columns, rows);

        if (columns !== lcdState.columns || rows !== lcdState.rows) {
            return;
        }

        if (offset < 0 || offset >= lcdState.rows * lcdState.columns) {
            return;
        }

        lcdState.characters[offset] = char;
        invalidateLcdPosition(offset);
        requestRedraw();
    }

    function invalidateLcdPosition(lcdPos: number) {
        lcdState.rowNeedsUpdate |= (1 << Math.idiv(lcdPos, lcdState.columns))
    }
}

namespace makerbit {

    /**
     * Displays a text on a LCD1602 in the given position range.
     * The text will be cropped if it is longer than the provided length.
     * If there is space left, it will be filled with pad characters.
     * @param text the text to show, eg: "MakerBit"
     * @param startPosition the start position on the LCD, [1 - 32]
     * @param length the maximum space used on the LCD, eg: 16
     * @param option configures padding and alignment, eg: TextOption.Left
     */
    //% subcategory="LCD"
    //% blockId="makerbit_lcd_show_string_on_1602"
    //% block="LCD1602 show %text | at position %startPosition=makerbit_lcd_position_1602 with length %length || and %option"
    //% text.shadowOptions.toString=true
    //% length.min=1 length.max=32 length.fieldOptions.precision=1
    //% expandableArgumentMode="toggle"
    //% inlineInputMode="inline"
    //% weight=90
    export function showStringOnLcd1602(
        text: string,
        startPosition: number,
        length: number,
        option?: TextOption
    ): void {
        updateCharacterBuffer(
            text,
            startPosition - 1,
            length,
            16,
            2,
            toAlignment(option),
            toPad(option)
        );
    }

    /**
     * Clears the LCD1602 completely.
     */
    //% subcategory="LCD"
    //% blockId="makerbit_lcd_clear_1602" block="LCD1602 clear display"
    //% weight=89
    export function clearLcd1602(): void {
        showStringOnLcd1602("", 1, 32);
    }

    /**
     * Turns a LCD position into a number.
     * @param pos the LCD position, eg: LcdPosition1602.Pos1
     */
    //% subcategory="LCD"
    //% blockId=makerbit_lcd_position_1602
    //% block="%pos"
    //% pos.fieldEditor="gridpicker"
    //% pos.fieldOptions.columns=16
    //% blockHidden=true
    export function position1602(pos: LcdPosition1602): number {
        return pos;
    }

    /**
     * Display a custom character at a specified LCD position.
     */
    //% subcategory="LCD"
    //% blockId="makerbit_lcd_showchararacter1602"
    //% block="LCD1602 show character %char|at position %position=makerbit_lcd_position_1602"
    //% weight=58
    export function lcdShowCharacter1602(char: LcdChar, position: number): void {
        setCharacter(char, position - 1, 16, 2);
    }

}
}