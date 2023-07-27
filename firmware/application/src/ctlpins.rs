
use stm32f4xx_hal::gpio::DynamicPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital;

// create an enum with 3 possibilities: High, Low, and Floating
// this is used to set the CTL pins to a specific state
#[derive(Copy, Clone)]
pub enum PinState {
    High,
    Low,
    Floating,
}

pub trait CTLPinsTrait {
    fn set_ctl_a(&mut self, state:PinState);
    fn set_ctl_b(&mut self, state:PinState);
    fn set_ctl_c(&mut self, state:PinState);
    fn set_ctl_d(&mut self, state:PinState);
    fn set_reset(&mut self, state:PinState);
    fn power_on(&mut self);
    fn power_off(&mut self);

}

pub struct CTLPins<PWPin>
where
    PWPin: OutputPin, {
    ctl_a: DynamicPin<'A', 5>,
    stored_a: PinState,
    ctl_b: DynamicPin<'A', 6>,
    stored_b: PinState,
    ctl_c: DynamicPin<'A', 7>,
    stored_c: PinState,
    ctl_d: DynamicPin<'A', 8>,
    stored_d: PinState,
    reset: DynamicPin<'A', 9>,
    stored_reset: PinState,
    power: PWPin,
    on: bool,
}

impl<PWPin> CTLPins<PWPin>
where
    PWPin: OutputPin,
{
    pub fn new(ctl_a:DynamicPin<'A', 5>,
               ctl_b:DynamicPin<'A', 6>,
               ctl_c:DynamicPin<'A', 7>,
               ctl_d:DynamicPin<'A', 8>,
               reset:DynamicPin<'A', 9>,
               power:PWPin) -> Self {
        let mut instance = Self{ctl_a, stored_a: PinState::Floating,
                                ctl_b, stored_b: PinState::Floating,
                                ctl_c, stored_c: PinState::Floating,
                                ctl_d, stored_d: PinState::Floating,
                                reset, stored_reset: PinState::Floating,
                                power, on: false};
        instance.set_ctl_a(PinState::Floating);
        instance.set_ctl_b(PinState::Floating);
        instance.set_ctl_c(PinState::Floating);
        instance.set_ctl_d(PinState::Floating);
        instance.set_reset(PinState::Floating);
        instance.power_off();
        instance
    }

    fn _set_ctl_a(&mut self, state:PinState) {
        match state {
            PinState::High      => self.ctl_a.make_push_pull_output_in_state(digital::v2::PinState::High),
            PinState::Low       => self.ctl_a.make_push_pull_output_in_state(digital::v2::PinState::Low),
            PinState::Floating  => self.ctl_a.make_floating_input(),
        }
    }

    fn _set_ctl_b(&mut self, state: PinState) {
        match state {
            PinState::High      => self.ctl_b.make_push_pull_output_in_state(digital::v2::PinState::High),
            PinState::Low       => self.ctl_b.make_push_pull_output_in_state(digital::v2::PinState::Low),
            PinState::Floating  => self.ctl_b.make_floating_input(),
        }
    }

    fn _set_ctl_c(&mut self, state: PinState) {
        match state {
            PinState::High      => self.ctl_c.make_push_pull_output_in_state(digital::v2::PinState::High),
            PinState::Low       => self.ctl_c.make_push_pull_output_in_state(digital::v2::PinState::Low),
            PinState::Floating  => self.ctl_c.make_floating_input(),
        }
    }

    fn _set_ctl_d(&mut self, state: PinState) {
        match state {
            PinState::High      => self.ctl_d.make_push_pull_output_in_state(digital::v2::PinState::High),
            PinState::Low       => self.ctl_d.make_push_pull_output_in_state(digital::v2::PinState::Low),
            PinState::Floating  => self.ctl_d.make_floating_input(),
        }
    }

    fn _set_reset(&mut self, state: PinState) {
        match state {
            PinState::High      => self.reset.make_push_pull_output_in_state(digital::v2::PinState::High),
            PinState::Low       => self.reset.make_push_pull_output_in_state(digital::v2::PinState::Low),
            PinState::Floating  => self.reset.make_floating_input(),
        }
    }

}

impl<PWPin> CTLPinsTrait for CTLPins<PWPin>
where
    PWPin: OutputPin,
{

    fn set_ctl_a(&mut self, state:PinState) {
        self.stored_a = state;
        if self.on {
            self._set_ctl_a(state)
        }
    }

    fn set_ctl_b(&mut self, state: PinState) {
        self.stored_b = state;
        if self.on {
            self._set_ctl_b(state)
        }
    }

    fn set_ctl_c(&mut self, state: PinState) {
        self.stored_c = state;
        if self.on {
            self._set_ctl_c(state)
        }
    }


    fn set_ctl_d(&mut self, state: PinState) {
        self.stored_d = state;
        if self.on {
            self._set_ctl_d(state)
        }
    }

    fn set_reset(&mut self, state: PinState) {
        self.stored_reset = state;
        if self.on {
            self._set_reset(state)
        }
    }

    fn power_on(&mut self) {
        self._set_ctl_a(self.stored_a);
        self._set_ctl_b(self.stored_b);
        self._set_ctl_c(self.stored_c);
        self._set_ctl_d(self.stored_d);
        self._set_reset(self.stored_reset);
        self.power.set_high().ok();
        self.on = true;
    }

    fn power_off(&mut self) {
        // we set the control pins to floating while in power off, so power is not drawn
        // from the output pins into the carried board
        self._set_ctl_a(PinState::Floating);
        self._set_ctl_b(PinState::Floating);
        self._set_ctl_c(PinState::Floating);
        self._set_ctl_d(PinState::Floating);
        self._set_reset(PinState::Floating);
        self.power.set_low().ok();
        self.on = false;
    }
}

