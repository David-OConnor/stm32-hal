//! This module handles the procedures used for measuring conductivity, by adjusting
//! multiplexed Gain, setting DAC output voltage, reading ADC input voltage, and converting
//! to a conductivity reading. Based on Analog Devices CN-0411.
//!
//! This module does not handle calibration or temperature compensation; the intent is to handle
//! this on the host device this module connects to.
//!
//! References:
//! https://www.analog.com/en/design-center/reference-designs/circuits-from-the-lab/cn0411.html
//! https://wiki.analog.com/resources/eval/user-guides/eval-adicup360/reference_designs/demo_cn0411
//! https://github.com/analogdevicesinc/EVAL-ADICUP360/blob/master/projects/ADuCM360_demo_cn0411/src/CN0411.c

use hal::{
    dac::{Dac, DacChannel},
    gpio::Pin,
    i2c::I2c,
    pac::{DAC1, I2C1, TIM2},
    timer::{TimChannel, Timer},
};

const CFG_REG: u8 = 0x1;
const CONV_REG: u8 = 0x0;

#[derive(Debug, Clone, Copy)]
/// Data for a single EC calibration point. Like with ORP, we use a single-point,
/// linear model, with the other end of the line pinned to 0, 0. Temperature isn't
/// (currently?) used in this model. This is used by the Water Monitor.
pub struct CalPtEc {
    // todo: this struct is DRY with anyleaf-rust. Note that this isn't implemented
    // todo here directly, but is used in the Water Monitor. We maybe should use
    // todo it here directly in the future.
    pub reading: f32, // reading in μS/cm
    pub ec: f32,      // nominal EC, in μS/cm
    pub T: f32,       // in Celsius
}

// Frequencies for the PWM channels. Higher frequencies reduce polarization.
// Lower frequencies reduce capacitance effects. The frequencies we choose are a compromise
// between the two.
// todo: finer resolution
// 94Hz for uS, and 2.4khz for mS range, uS/cm
// const PWM_THRESH_HIGH: f32 = 1_200.;
// const PWM_THRESH_LOW: f32 = 400.;

const PWM_THRESH_HIGH: f32 = 0.001200;
const PWM_THRESH_LOW: f32 = 0.000400;

// `V_PROBE_TGT` is the desired applied voltage across the conductivity electrodes.
// Loose requirement from AD engineers: "100mV is good enough", and don't
// overvolt the probe.
const V_PROBE_TGT: f32 = 0.15;

// `V_EXC_INIT` is our initial excitation voltage, set by the DAC.
const V_EXC_INIT: f32 = 0.4;

// Take the average of several readings, to produce smoother results.
// A higher value of `N_SAMPLES` is more accurate, but takes longer.
const N_SAMPLES: u8 = 1; // todo: Is setting to 1 ok?

pub struct ReadError {}

#[derive(Clone, Copy, Debug, PartialEq)]
/// 94Hz for uS, and 2.4khz for mS range.
pub enum PwmFreq {
    Low,  // 94Hz. mS range
    Med,  // 700Hz. mid
    High, // 2.4Khz. uS range
}

impl PwmFreq {
    /// Return ARR and PSC values. These are precomputed as an optimization.
    /// They assume a 80Mhz timer speed, and edge-aligned.
    pub fn arr_psc(&self) -> (u32, u16) {
        match self {
            Self::Low => (920, 921),
            Self::Med => (337, 337),
            Self::High => (181, 182),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EcGain {
    // Gain 1 isn't used. We set it up this way
    // so the gains match with the S values.
    One,    // S8. Highest resistance
    Two,    // S7
    Three,  // S6
    Four,   // S5
    Five,   // S4
    Six,    // S3
    Seven,  // S2. Lowest resistance
    _Eight, // S1. No resistance. Used for calibrating DAC to ADC
}

impl EcGain {
    /// Raise by one level.
    fn raise(&self) -> Self {
        match self {
            Self::One => Self::Two,
            Self::Two => Self::Three,
            Self::Three => Self::Four,
            Self::Four => Self::Five,
            Self::Five => Self::Six,
            Self::Six => Self::Seven,
            Self::Seven => panic!("Gain is already at maximum"),
            Self::_Eight => panic!("Attempting to raise gain when set at Eight"),
        }
    }

    /// Display the resistance associated with gain, in Ω.
    /// For gains Six and Seven, we must take into account resistance of the multiplexer circuit,
    /// since it's significant compared to the resistor value. This is why they deviate
    /// from the pattern. These values are taken from measurement.
    ///
    /// todo: Re-evaluate cal values on the final board for gains 6/7.
    fn resistance(&self) -> f32 {
        // The numbers are a constant high, to take into account the multiplexor's
        // on resistance. Only significantly affects results for gains Six and Seven.
        let r = match self {
            Self::_Eight => 6,
            Self::Seven => 26,
            Self::Six => 206,
            Self::Five => 2_006,
            // todo temporary, due to a batch of boards having wrong resistor.
            // Self::Five => 3_926,
            Self::Four => 20_006,
            Self::Three => 200_006,
            Self::Two => 2_000_006,
            Self::One => 20_000_006,
        };

        r as f32
    }
}

/// Use the ADG1608 to select the right resistor.
pub struct Adg1608 {
    pin0: Pin,
    pin1: Pin,
    pin2: Pin,
}

impl Adg1608 {
    pub fn new(pin0: Pin, pin1: Pin, pin2: Pin) -> Self {
        Self { pin0, pin1, pin2 }
    }

    /// Set the multiplexer path to use, in order to use the proper gain resistor.
    pub fn set(&mut self, gain: EcGain) {
        // Enable pin must be pulled high for this to work.
        match gain {
            EcGain::_Eight => {
                self.pin0.set_low();
                self.pin1.set_low();
                self.pin2.set_low();
            }
            EcGain::Seven => {
                self.pin0.set_high();
                self.pin1.set_low();
                self.pin2.set_low();
            }
            EcGain::Six => {
                self.pin0.set_low();
                self.pin1.set_high();
                self.pin2.set_low();
            }
            EcGain::Five => {
                self.pin0.set_high();
                self.pin1.set_high();
                self.pin2.set_low();
            }
            EcGain::Four => {
                self.pin0.set_low();
                self.pin1.set_low();
                self.pin2.set_high();
            }
            EcGain::Three => {
                self.pin0.set_high();
                self.pin1.set_low();
                self.pin2.set_high();
            }
            EcGain::Two => {
                self.pin0.set_low();
                self.pin1.set_high();
                self.pin2.set_high();
            }
            EcGain::One => {
                self.pin0.set_high();
                self.pin1.set_high();
                self.pin2.set_high();
            }
        }
    }
}

/// The high-level struct representing the EC circuit.
pub struct EcSensor {
    pub dac: Dac<DAC1>,
    timer: Timer<TIM2>,
    gain_switch: Adg1608,
    pub K_cell: f32, // constant of the conductivity probe used.
    pub cal: Option<CalPtEc>,
    // Temp-compensated, in uS/cm2; post-processing. Used for determining
    // how to set the PWM freq for the next cycle.
    last_meas: PwmFreq,
    // `dac_adc_factor is percent change of ADC input from DAC output, on the 0 gain
    // setting. Used to tweak readings, to offset for differences.
    // dac_adc_factor: f32,
}

impl EcSensor {
    /// Prior to taking measurements, set up the timer at the
    /// appropriate frequency and resolution.
    pub fn new(
        dac: Dac<DAC1>,
        timer: Timer<TIM2>,
        switch_pins: (Pin, Pin, Pin),
        K_cell: f32,
    ) -> Self {
        Self {
            dac,
            timer,
            gain_switch: Adg1608::new(switch_pins.0, switch_pins.1, switch_pins.2),
            K_cell,
            cal: None,
            last_meas: PwmFreq::High,
        }
    }

    /// Set gain and excitation voltage, as an auto-ranging procedure.
    /// Return the values, for use in the computation. See CN-0411: Table 12.
    fn set_range(&mut self, addr: u8, read_cmd: u16, i2c: &mut I2c<I2C1>) -> (f32, EcGain) {
        // Set the multiplexer to highest gain resistance. (lowest gain).
        let mut gain = EcGain::One;
        self.gain_switch.set(gain);

        // Set the initial exciation voltage.
        let mut V_exc = V_EXC_INIT;
        self.dac.write_voltage(DacChannel::C1, V_exc);

        // Read ADC Input V+ and V-
        let readings = self.read_voltage(addr, read_cmd, i2c);

        // todo: Dry with the read circuit. If you end up using it here, make it its
        // todo own fn.
        let mut V_probe = (readings.0 + readings.1) / 2.;

        // Note that higher conductivities result in higher gain, due to the nature of
        // our 2-part voltage divider. Resistance one is from the gain resistor; resistance
        // 2 is from the ec probe.

        // We adjust gain until probe resistance and gain circuit resistance are
        // at the same order of magnitude. We'd like the gain resistance
        // to be higher than probe.

        // Note: You exceed full-scale ADC range when probe resistance is
        // higher than gain res

        // todo: Maintain voltage and maybe gain for next reading, or don't
        // todo set range every time!

        // We attempt to get probe and gain-circuit resistances to be on the same
        // order of magnitude. We keep gain resistance higher than probe resistance,
        // so we don't exceed the ADC fullscale range (currently set to +-2.048V). Note
        // that this assumes a 10x voltage amplification by the instrumentation amp.
        // If V_probe is set to 0.1, and probe resistance <= gain resistance,
        // we should stay within that range.

        // Initially, it's likely the gain resistance will be much higher than probe resistance;
        // Very little voltage will be read at the ADC. Increase gain until we get a voltage
        // on the order of magnitude of v_exc. This also means gain resistance and probe
        // resistance are on the same order of magnitude.
        while V_probe <= (0.3 * V_exc) && gain != EcGain::Seven {
            // todo: Dry setting gain and v_exc between here, and before the loop
            gain = gain.raise();
            self.gain_switch.set(gain);

            // .1V reading. <= 1.2V limit

            // Read ADC Input V+ and V-
            let readings = self.read_voltage(addr, read_cmd, i2c);

            V_probe = (readings.0 + readings.1) / 2.;
        }

        // Set excitation voltage to deliver a target voltage across the probe. This prevents
        // overvolting both the probe, and ADC. (Since ADC input is v_probe, amplified.)
        // See `measure` for how we
        let I = (V_exc - V_probe) / gain.resistance();
        V_exc = I * gain.resistance() + V_PROBE_TGT;
        self.dac.write_voltage(DacChannel::C1, V_exc);

        (V_exc, gain)
    }

    /// We use the PWM timer's channels 2 and 3 to trigger readings at the appropriate times in
    /// the cycle. They're configured in `util::setup_pwm`, and trigger in the middle of
    /// each of the 2 polarities.
    ///
    /// We divide the read voltage by the AMP; we're looking for a pre-amp voltage,
    /// but reading post-amp.
    pub(crate) fn read_voltage(&mut self, addr: u8, cmd: u16, i2c: &mut I2c<I2C1>) -> (f32, f32) {
        //  ignoring + vs - timings.
        // todo: Make this questionmarkable.
        let v_p = voltage_from_adc_512(take_reading(addr, cmd, i2c));
        // todo: Delay half a period?
        let v_m = voltage_from_adc_512(take_reading(addr, cmd, i2c));

        (v_p, v_m)
    }

    /// Take an uncalibrated reading, without adjusting PWM.
    /// Similar in use to others sensors' `read_voltage()`.
    pub fn measure(
        &mut self,
        addr: u8,
        read_cmd: u16,
        i2c: &mut I2c<I2C1>,
    ) -> Result<f32, ReadError>
where {
        // [dis]enabling the dac each reading should improve battery usage.
        // Do this upstream, eg `sensors.rs` on Water Monitor, or `main.rs` here.

        // todo: Set ADC sample speed here?
        let (V_exc, gain) = self.set_range(addr, read_cmd, i2c);

        let mut v_p_cum = 0.;
        let mut v_m_cum = 0.;

        for _ in 0..N_SAMPLES {
            let (v_p, v_m) = self.read_voltage(addr, read_cmd, i2c);
            v_p_cum += v_p;
            v_m_cum += v_m;
        }

        let v_p = v_p_cum / N_SAMPLES as f32;
        let v_m = v_m_cum / N_SAMPLES as f32;

        // We calculate conductivity across the probe using a voltage-divider model.
        // We know the following:
        // V_exc (set by DAC)
        // R_gain (set by multiplexer)
        // V_out (measuring by ADC), is the same as V_probe
        //
        // We solve for current, and use it to calculate conductivity:
        //
        // I = V_exc / (R_gain + R_probe) = V_exc / (R_gain + V_probe / I)
        // I(R_gain + V_probe / I) = V_exc
        // I x R_gain + V_probe = V_exc
        // I = (V_exc - V_probe) / R_gain

        let V_probe = (v_p + v_m) / 2.;
        // `I` is the current flowing through the gain circuitry, then probe.
        let I = (V_exc - V_probe) / gain.resistance();
        // K is in 1/cm. eg: K=1.0 could mean 2 1cm square plates 1cm apart.
        let Y_sol = self.K_cell * I / V_probe;

        Ok(Y_sol)
    }

    /// Take a conductivity measurement. Returns a conductivity measurement in uS/cm.
    /// Calibration, temperature compensation, and unit converstion (eg mS/cm, TDS)
    /// is performed downstream, eg in the `sensors` module here, or in the
    /// drivers for the standalone ec module.
    pub fn read(&mut self, addr: u8, read_cmd: u16, i2c: &mut I2c<I2C1>) -> Result<f32, ReadError> {
        let Y_sol = self.measure(addr, read_cmd, i2c)?;
        let result = Y_sol;

        // We set PSC and ARR directly here with cached, to save the computation.

        if result > PWM_THRESH_HIGH && self.last_meas != PwmFreq::High {
            self.timer.set_auto_reload(PwmFreq::High.arr_psc().0);
            self.timer.set_prescaler(PwmFreq::High.arr_psc().1);
            self.timer
                .set_duty(TimChannel::C1, PwmFreq::High.arr_psc().0 / 2);

            self.last_meas = PwmFreq::High;
        } else if result > PWM_THRESH_LOW
            && result < PWM_THRESH_HIGH
            && self.last_meas != PwmFreq::Med
        {
            self.timer.set_auto_reload(PwmFreq::Med.arr_psc().0);
            self.timer.set_prescaler(PwmFreq::Med.arr_psc().1);
            self.timer
                .set_duty(TimChannel::C1, PwmFreq::Med.arr_psc().0 / 2);

            self.last_meas = PwmFreq::Med;
        } else if result < PWM_THRESH_LOW && self.last_meas != PwmFreq::Low {
            self.timer.set_auto_reload(PwmFreq::Low.arr_psc().0);
            self.timer.set_prescaler(PwmFreq::Low.arr_psc().1);
            self.timer
                .set_duty(TimChannel::C1, PwmFreq::Low.arr_psc().0 / 2);

            self.last_meas = PwmFreq::Low;
        }

        Ok(result)
    }
}

/// Convert a 16-bit digital value to voltage.
/// Input ranges from +- 0.512V; this is configurable.
/// Output ranges from -32_768 to +32_767.
/// See drivers/`voltage_from_adc`
pub fn voltage_from_adc_512(digi: i16) -> f32 {
    let vref = 0.512;
    (digi as f32 / 32_768.) * vref
}

// todo: Dry between here and `anyleaf-rust`.
/// Take a measurement from the ADS1115 ADC, using the I2C connection.
pub fn take_reading(addr: u8, cmd: u16, i2c: &mut I2c<I2C1>) -> i16 {
    let mut result_buf: [u8; 2] = [0, 0];

    // Set up the cfg, and command a one-shot reading. Note that we
    // pass the command as 2 bytes.
    i2c.write(addr, &[CFG_REG, (cmd >> 8) as u8, cmd as u8])
        .ok();

    // Wait until the conversion is complete.
    let mut converting = true;
    let mut buf = [0, 0];
    while converting {
        i2c.write_read(addr, &[CFG_REG], &mut buf).ok();
        // First of 16 cfg reg bits is 0 while converting, 1 when ready. (when reading)
        converting = buf[0] >> 7 == 0;
    }

    // Read the result from the conversion register.
    i2c.write_read(addr, &[CONV_REG], &mut result_buf).ok();

    i16::from_be_bytes([result_buf[0], result_buf[1]])
}
