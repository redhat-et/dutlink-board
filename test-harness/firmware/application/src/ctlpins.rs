
use stm32f4xx_hal::gpio::DynamicPin;
use embedded_hal::digital;

// create an enum with 3 possibilities: High, Low, and Floating
// this is used to set the CTL pins to a specific state
pub enum PinState {
    High,
    Low,
    Floating,
}

pub struct CTLPins {
    ctl_a: DynamicPin<'A', 5>,
    ctl_b: DynamicPin<'A', 6>,
    ctl_c: DynamicPin<'A', 7>,
    ctl_d: DynamicPin<'A', 8>,
    reset: DynamicPin<'A', 9>,
}

impl CTLPins
{
    pub fn new(ctl_a:DynamicPin<'A', 5>, ctl_b:DynamicPin<'A', 6>, ctl_c:DynamicPin<'A', 7>, ctl_d:DynamicPin<'A', 8>, reset:DynamicPin<'A', 9>) -> Self {
        let mut instance = Self{ctl_a, ctl_b, ctl_c, ctl_d, reset};
        instance.set_ctl_a(PinState::Floating);
        instance.set_ctl_b(PinState::Floating);
        instance.set_ctl_c(PinState::Floating);
        instance.set_ctl_d(PinState::Floating);
        instance.set_reset(PinState::Floating);
        instance
    }

    pub fn set_ctl_a(&mut self, state:PinState) {
        match state {
            PinState::High      => self.ctl_a.make_push_pull_output_in_state(digital::v2::PinState::High),
            PinState::Low       => self.ctl_a.make_push_pull_output_in_state(digital::v2::PinState::Low),
            PinState::Floating  => self.ctl_a.make_floating_input(),
        }
    }
    pub fn set_ctl_b(&mut self, state: PinState) {
        match state {
            PinState::High      => self.ctl_b.make_push_pull_output_in_state(digital::v2::PinState::High),
            PinState::Low       => self.ctl_b.make_push_pull_output_in_state(digital::v2::PinState::Low),
            PinState::Floating  => self.ctl_b.make_floating_input(),
        }
    }
    pub fn set_ctl_c(&mut self, state: PinState) {
        match state {
            PinState::High      => self.ctl_c.make_push_pull_output_in_state(digital::v2::PinState::High),
            PinState::Low       => self.ctl_c.make_push_pull_output_in_state(digital::v2::PinState::Low),
            PinState::Floating  => self.ctl_c.make_floating_input(),
        }
    }

    pub fn set_ctl_d(&mut self, state: PinState) {
        match state {
            PinState::High      => self.ctl_d.make_push_pull_output_in_state(digital::v2::PinState::High),
            PinState::Low       => self.ctl_d.make_push_pull_output_in_state(digital::v2::PinState::Low),
            PinState::Floating  => self.ctl_d.make_floating_input(),
        }
    }

    pub fn set_reset(&mut self, state: PinState) {
        match state {
            PinState::High      => self.reset.make_push_pull_output_in_state(digital::v2::PinState::High),
            PinState::Low       => self.reset.make_push_pull_output_in_state(digital::v2::PinState::Low),
            PinState::Floating  => self.reset.make_floating_input(),
        }
    }
}

