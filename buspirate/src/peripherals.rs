pub struct Config {
    power_supply: bool,
    pull_ups: bool,
    aux: bool,
    cs: bool,
}

impl Config {
    pub(crate) fn command_byte(&self) -> u8 {
        let mut cmd = 0b10000000 as u8;
        cmd = cmd | (if self.power_supply { 0 } else { 1 } << 3);
        cmd = cmd | (if self.pull_ups { 0 } else { 1 } << 2);
        cmd = cmd | (if self.aux { 0 } else { 1 } << 1);
        cmd = cmd | (if self.cs { 0 } else { 1 } << 0);
        cmd
    }
}
