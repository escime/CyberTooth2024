from commands2 import Subsystem
from wpilib import AddressableLED, Timer
from constants import LEDConstants


class LEDs(Subsystem):
    def __init__(self, timer: Timer):
        super().__init__()
        self.timer = timer
        self.chain = AddressableLED(LEDConstants.port)
        self.chain.setLength(LEDConstants.strip_length)
        self.buffer = [AddressableLED.LEDData(0, 0, 0)] * LEDConstants.strip_length
        self.chain.setData(self.buffer)
        self.chain.start()
        self.state = "default"

        # Prepare default pattern
        self.default_pattern = [AddressableLED.LEDData(50, 149, 168)] * 5
        for i in range(0, LEDConstants.strip_length - 5):
            self.default_pattern.append(AddressableLED.LEDData(0, 0, 0))

        # Prepare gp_held pattern
        self.gp_held_pattern = [AddressableLED.LEDData(50, 149, 168)] * 10
        for i in range(0, LEDConstants.strip_length - 10):
            self.gp_held_pattern.append(AddressableLED.LEDData(255, 0, 0))

        # Prepare rainbow pattern
        self.rainbow_pattern = []
        for i in range(0, int(LEDConstants.strip_length / 5)):
            self.rainbow_pattern.append(AddressableLED.LEDData(0, 255, 0))
            self.rainbow_pattern.append(AddressableLED.LEDData(213, 255, 0))
            self.rainbow_pattern.append(AddressableLED.LEDData(255, 0, 0))
            self.rainbow_pattern.append(AddressableLED.LEDData(47, 0, 255))
            self.rainbow_pattern.append(AddressableLED.LEDData(0, 255, 238))

        # Prepare flash color settings
        self.flash_color_rate = 2
        self.flash_color_color = [255, 0, 0]
        self.flash_color_state = True

        # Prepare shoot settings
        self.shoot_pattern = [AddressableLED.LEDData(0, 0, 255)] * 5
        for i in range(0, LEDConstants.strip_length - 5):
            self.shoot_pattern.append(AddressableLED.LEDData(0, 0, 0))

        # Prepare timer lights settings
        self.timer_lights_on = True
        self.timer_lights_time = 10
        self.timer_lights_segment = int(LEDConstants.strip_length / self.timer_lights_time)
        self.timer_lights_pattern = [AddressableLED.LEDData(0, 255, 0)] * \
            (LEDConstants.strip_length - self.timer_lights_segment)
        for i in range(0, self.timer_lights_segment):
            self.timer_lights_pattern.append(AddressableLED.LEDData(0, 0, 0))

        # Prepare alignment settings
        self.align_pattern = [AddressableLED.LEDData(50, 149, 168)] * 10
        for i in range(0, LEDConstants.strip_length - 10):
            self.align_pattern.append(AddressableLED.LEDData(0, 0, 0))
        self.misalignment = 0

        self.last_time = self.timer.get()

    def set_state(self, target_state: str) -> None:
        """Set the current state of the subsystem."""
        self.state = target_state
        if target_state == "timer_lights":
            self.buffer = [AddressableLED.LEDData(0, 255, 0)] * LEDConstants.strip_length

    def periodic(self) -> None:
        if self.state == "default":
            self.default()
        elif self.state == "flash_color":
            self.flash_color()
        elif self.state == "shoot":
            self.shoot()
        elif self.state == "gp_held":
            self.gp_held()
        elif self.state == "rainbow":
            self.rainbow()
        elif self.state == "timer_lights":
            self.timer_lights()
        elif self.state == "align":
            self.align()
        else:
            self.default()
        self.chain.setData(self.buffer)

    def default(self) -> None:
        """Logic for running default animation."""
        if self.timer.get() - 0.05 > self.last_time:
            self.buffer = self.default_pattern
            self.default_pattern = self.default_pattern[1:] + self.default_pattern[:1]
            self.last_time = self.timer.get()

    def flash_color(self) -> None:
        """Flash a specified color at a specified frequency."""
        if self.timer.get() - (1 / self.flash_color_rate) > self.last_time and self.flash_color_state:
            self.buffer = [AddressableLED.LEDData(self.flash_color_color[0], self.flash_color_color[1],
                                                  self.flash_color_color[2])] * LEDConstants.strip_length
            self.flash_color_state = False
            self.last_time = self.timer.get()
        elif self.timer.get() - (1 / self.flash_color_rate) > self.last_time and not self.flash_color_state:
            self.buffer = [AddressableLED.LEDData(0, 0, 0)] * LEDConstants.strip_length
            self.flash_color_state = True
            self.last_time = self.timer.get()

    def set_flash_color_color(self, color: []) -> None:
        """Set the color for Flash Color."""
        self.flash_color_color = color

    def set_flash_color_rate(self, rate: int) -> None:
        """Set the rate for Flash Color."""
        self.flash_color_rate = rate

    def shoot(self) -> None:
        """Run the LEDs towards the shooter and then hold in place."""
        shoot_complete = False
        if self.shoot_pattern[-1].b == 255:
            shoot_complete = True
            self.buffer = self.shoot_pattern
        if self.timer.get() - 0.02 > self.last_time and not shoot_complete:
            self.buffer = self.shoot_pattern
            self.shoot_pattern = self.shoot_pattern[-1:] + self.shoot_pattern[:-1]
            self.last_time = self.timer.get()

    def reset_shoot(self) -> None:
        """Reset the shoot animation."""
        self.shoot_pattern = [AddressableLED.LEDData(0, 0, 255)] * 5
        for i in range(0, LEDConstants.strip_length - 5):
            self.shoot_pattern.append(AddressableLED.LEDData(0, 0, 0))

    def gp_held(self) -> None:
        """Run the animation for holding a game piece."""
        if self.timer.get() - 0.05 > self.last_time:
            self.buffer = self.gp_held_pattern
            self.gp_held_pattern = self.gp_held_pattern[1:] + self.gp_held_pattern[:1]
            self.last_time = self.timer.get()

    def rainbow(self) -> None:
        """Run the rainbow shift animation."""
        if self.timer.get() - 0.05 > self.last_time:
            self.buffer = self.rainbow_pattern
            self.rainbow_pattern = self.rainbow_pattern[1:] + self.rainbow_pattern[:1]
            self.last_time = self.timer.get()

    def timer_lights(self) -> None:
        """Run a timer animation on the lights for a set time."""
        self.timer_lights_on = True
        if self.timer.get() - 1 > self.last_time and self.timer_lights_pattern[0].g == 0:
            self.timer_lights_on = False
            self.buffer = self.timer_lights_pattern
        if self.timer.get() - 1 > self.last_time and self.timer_lights_on:
            self.buffer = self.timer_lights_pattern
            self.timer_lights_pattern = self.timer_lights_pattern[self.timer_lights_segment:]
            for i in range(0, self.timer_lights_segment):
                self.timer_lights_pattern.append(AddressableLED.LEDData(0, 0, 0))
            self.last_time = self.timer.get()

    def set_timer_lights_time(self, time: float) -> None:
        """Set the time for the timer lights."""
        self.timer_lights_on = True
        self.timer_lights_time = time
        self.timer_lights_segment = int(LEDConstants.strip_length / self.timer_lights_time)
        self.timer_lights_pattern = [AddressableLED.LEDData(0, 255, 0)] * \
            (LEDConstants.strip_length - self.timer_lights_segment)
        for i in range(0, self.timer_lights_segment):
            self.timer_lights_pattern.append(AddressableLED.LEDData(0, 0, 0))

    def reset_timer_lights(self) -> None:
        self.timer_lights_on = True
        self.timer_lights_pattern = [AddressableLED.LEDData(0, 255, 0)] * \
            (LEDConstants.strip_length - self.timer_lights_segment)
        for i in range(0, self.timer_lights_segment):
            self.timer_lights_pattern.append(AddressableLED.LEDData(0, 0, 0))

    def align(self) -> None:
        time_scalar = (abs(self.misalignment) * 0.95) + 0.05
        if self.timer.get() - time_scalar > self.last_time:
            if self.misalignment > 0:
                self.buffer = self.align_pattern
                self.align_pattern = self.align_pattern[1:] + self.align_pattern[:1]
            elif self.misalignment < 0:
                self.buffer = self.align_pattern
                self.align_pattern = self.align_pattern[-1:] + self.align_pattern[:-1]
            else:
                self.buffer = self.align_pattern
            self.last_time = self.timer.get()

    def set_misalignment(self, target: float, current: float) -> None:
        self.misalignment = (target - current) / 360
