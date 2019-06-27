pub enum Speed {
    Speed30KHz,
    Speed125KHz,
    Speed250KHz,
    Speed1MHz,
    Speed2MHz,
    Speed2_6MHz,
    Speed4MHz,
    Speed8MHz,
}

pub enum PinOutput {
    PinOutputHiZ,
    PinOutput3_3V,
}

pub enum ClockPhase {
    ClockPhaseHigh,
    ClockPhaseLow,
}

pub enum ClockEdge {
    ClockEdgeFalling,
    ClockEdgeRising,
}

pub enum SampleTime {
    SampleTimeMiddle,
    SampleTimeEnd,
}

pub struct Config {
    pub pin_output: PinOutput,
    pub clock_idle_phase: ClockPhase,
    pub clock_edge: ClockEdge,
    pub sample_time: SampleTime,
}

pub const DEFAULT_CONFIG: Config = Config {
    pin_output: PinOutput::PinOutputHiZ,
    clock_idle_phase: ClockPhase::ClockPhaseLow,
    clock_edge: ClockEdge::ClockEdgeFalling,
    sample_time: SampleTime::SampleTimeMiddle,
};

impl Config {
    pub(crate) fn command_byte(&self) -> u8 {
        let mut cmd = 0b10000000 as u8;
        cmd = cmd
            | (match self.pin_output {
                PinOutput::PinOutputHiZ => 0,
                PinOutput::PinOutput3_3V => 1,
            } << 3);
        cmd = cmd
            | (match self.clock_idle_phase {
                ClockPhase::ClockPhaseLow => 0,
                ClockPhase::ClockPhaseHigh => 1,
            } << 2);
        cmd = cmd
            | (match self.clock_edge {
                ClockEdge::ClockEdgeRising => 0,
                ClockEdge::ClockEdgeFalling => 1,
            } << 1);
        cmd = cmd
            | (match self.sample_time {
                SampleTime::SampleTimeMiddle => 0,
                SampleTime::SampleTimeEnd => 1,
            } << 0);
        cmd
    }
}
