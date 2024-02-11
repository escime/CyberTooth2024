import commands2
from wpilib import AddressableLED, Timer
import random


class LEDs(commands2.SubsystemBase):

    animation_delay = 50
    timer = Timer()
    m_pattern = None
    record_time = None
    rainbow_pattern = []
    default_pattern = []
    purple_pattern = []
    notifier_length = 3
    notifier_state = [AddressableLED.LEDData(255, 0, 0)] * notifier_length
    heat = []
    current_state = "purple_chaser"
    flash_state = True
    dominant_color = [255, 0, 0]
    flash_rate = 2
    shooting = False
    shooting_counter = 0
    amp_timer_on = False
    amp_index = 0

    def __init__(self, port: int, length: int, num: int, animation_speed: float, style: str) -> None:
        super().__init__()
        self.style = style
        self.m_led = AddressableLED(port)  # Connect the LED chain to the right port.
        self.length = length  # Set the length from the constructor.
        self.num_of_strips = num  # Set the number of strips from the constructor.
        self.m_led.setLength(length * num)  # Set the WPILIB length for the entire LED chain.
        self.m_ledBuffer = [AddressableLED.LEDData(0, 0, 0)] * length  # Set up the buffer.
        self.clear_pattern = [AddressableLED.LEDData(0, 0, 0)] * length  # Set up the blank pattern.
        repeat = self.num_of_strips - 1
        if repeat == 0:
            self.m_ledBuffer_complete = self.m_ledBuffer
        if repeat > 0:
            self.m_ledBuffer_complete = self.m_ledBuffer * self.num_of_strips
        self.m_led.setData(self.m_ledBuffer_complete)
        self.m_led.start()
        self.animation_delay = animation_speed
        self.timer.start()
        self.record_time = self.timer.get()

        # Setup rainbow pattern default
        if self.style == "RGB":
            for i in range(0, int(self.length / 15)):
                self.rainbow_pattern.append(AddressableLED.LEDData(255, 0, 0))
                self.rainbow_pattern.append(AddressableLED.LEDData(255, 0, 0))
                self.rainbow_pattern.append(AddressableLED.LEDData(255, 0, 0))
                self.rainbow_pattern.append(AddressableLED.LEDData(255, 213, 0))
                self.rainbow_pattern.append(AddressableLED.LEDData(255, 213, 0))
                self.rainbow_pattern.append(AddressableLED.LEDData(255, 213, 0))
                self.rainbow_pattern.append(AddressableLED.LEDData(0, 255, 0))
                self.rainbow_pattern.append(AddressableLED.LEDData(0, 255, 0))
                self.rainbow_pattern.append(AddressableLED.LEDData(0, 255, 0))
                self.rainbow_pattern.append(AddressableLED.LEDData(0, 47, 255))
                self.rainbow_pattern.append(AddressableLED.LEDData(0, 47, 255))
                self.rainbow_pattern.append(AddressableLED.LEDData(0, 47, 255))
                self.rainbow_pattern.append(AddressableLED.LEDData(255, 0, 238))
                self.rainbow_pattern.append(AddressableLED.LEDData(255, 0, 238))
                self.rainbow_pattern.append(AddressableLED.LEDData(255, 0, 238))
        elif self.style == "GRB":
            for i in range(0, int(self.length / 5)):
                self.rainbow_pattern.append(AddressableLED.LEDData(0, 255, 0))
                self.rainbow_pattern.append(AddressableLED.LEDData(213, 255, 0))
                self.rainbow_pattern.append(AddressableLED.LEDData(255, 0, 0))
                self.rainbow_pattern.append(AddressableLED.LEDData(47, 0, 255))
                self.rainbow_pattern.append(AddressableLED.LEDData(0, 255, 238))

        # Setup purple chase pattern default
        if self.style == "RGB":
            self.purple_pattern = [AddressableLED.LEDData(149, 50, 168)] * 5
        elif self.style == "GRB":
            self.purple_pattern = [AddressableLED.LEDData(50, 149, 168)] * 5
        for i in range(0, self.length - 5):
            self.purple_pattern.append(AddressableLED.LEDData(0, 0, 0))

        # Setup shoot animation default
        self.shoot_pattern = [AddressableLED.LEDData(142, 254, 15)] * 5
        for i in range(0, self.length - 5):
            self.shoot_pattern.append(AddressableLED.LEDData(0, 0, 0))

        # Setup amplification timer default.
        self.amp_length = int(self.length / 10)
        self.amp_pattern_displayed = [AddressableLED.LEDData(0, 255, 0)] * (self.length - (self.amp_length * 2))
        for i in range(0, self.amp_length * 2):
            self.amp_pattern_displayed.append(AddressableLED.LEDData(0, 0, 0))

        self.heat = [255] * self.length
        self.current_color = [AddressableLED.LEDData(0, 0, 0)] * self.length

        # Create auto_set_check patterns.
        if self.style == "RGB":
            self.condition_1_met = [AddressableLED.LEDData(255, 0, 0)] * int((self.length / 2))
            for i in range(0, int(self.length/2)):
                self.condition_1_met.append(AddressableLED.LEDData(0, 0, 0))
            self.condition_2_met = [AddressableLED.LEDData(0, 0, 255)] * int((self.length / 2))
            for i in range(0, int(self.length / 2)):
                self.condition_2_met.append(AddressableLED.LEDData(0, 0, 0))
            self.condition_all_met = [AddressableLED.LEDData(255, 0, 0)] * int((self.length / 2))
            for i in range(0, int(self.length / 2)):
                self.condition_all_met.append(AddressableLED.LEDData(0, 0, 255))
        else:
            self.condition_1_met = [AddressableLED.LEDData(0, 255, 0)] * int((self.length / 2))
            for i in range(0, int(self.length/2)):
                self.condition_1_met.append(AddressableLED.LEDData(0, 0, 0))
            self.condition_2_met = [AddressableLED.LEDData(0, 0, 255)] * int((self.length / 2))
            for i in range(0, int(self.length / 2)):
                self.condition_2_met.append(AddressableLED.LEDData(0, 0, 0))
            self.condition_all_met = [AddressableLED.LEDData(255, 0, 0)] * int((self.length / 2))
            for i in range(0, int(self.length / 2)):
                self.condition_all_met.append(AddressableLED.LEDData(0, 0, 255))

    def clear_buffer(self) -> None:
        """Clear the master buffer of all data."""
        self.m_ledBuffer = self.clear_pattern

    def set_chain(self) -> None:
        """Configure the set of LED chains with the pattern stored in the buffer."""
        repeat = self.num_of_strips - 1
        if repeat == 0:
            self.m_ledBuffer_complete = self.m_ledBuffer
        if repeat > 0:
            self.m_ledBuffer_complete = self.m_ledBuffer * self.num_of_strips
        self.m_led.setData(self.m_ledBuffer_complete)

    def set_chain_with_notifier(self):
        """Configure the chain with a modified buffer that includes a properly positioned notifier state."""
        self.m_ledBuffer[-self.notifier_length:] = self.notifier_state
        self.set_chain()

    def set_notifier(self, state: str):
        """Set the notifier state. This is intended to be a second layer of information on top of the master LED
        code that can give a secondary status update."""
        if state == "GREEN":
            if self.style == "RGB":
                self.notifier_state = [AddressableLED.LEDData(0, 255, 0)] * self.notifier_length
            elif self.style == "GRB":
                self.notifier_state = [AddressableLED.LEDData(255, 0, 0)] * self.notifier_length
        elif state == "BLUE":
            if self.style == "RGB":
                self.notifier_state = [AddressableLED.LEDData(0, 0, 255)] * self.notifier_length
            elif self.style == "GRB":
                self.notifier_state = [AddressableLED.LEDData(0, 0, 255)] * self.notifier_length
        elif state == "RED":
            if self.style == "RGB":
                self.notifier_state = [AddressableLED.LEDData(255, 0, 0)] * self.notifier_length
            elif self.style == "GRB":
                self.notifier_state = [AddressableLED.LEDData(0, 255, 0)] * self.notifier_length

    def rainbow_shift(self):
        """Configure the LED code for a rainbow wrapping around each strip."""
        if self.timer.get() - self.animation_delay > self.record_time:
            self.m_ledBuffer = self.rainbow_pattern
            self.rainbow_pattern = self.rainbow_pattern[1:] + self.rainbow_pattern[:1]
            self.record_time = self.timer.get()
        self.set_chain()
        self.current_state = "rainbow_shift"

    def flash_color(self, color: [], rate: int):
        self.dominant_color = color
        self.flash_rate = rate
        if self.timer.get() - (1 / rate) > self.record_time and self.flash_state:
            self.m_ledBuffer = [AddressableLED.LEDData(color[0], color[1], color[2])] * self.length
            self.record_time = self.timer.get()
            self.flash_state = False
        elif self.timer.get() - (1 / rate) > self.record_time and not self.flash_state:
            self.m_ledBuffer = [AddressableLED.LEDData(0, 0, 0)] * self.length
            self.record_time = self.timer.get()
            self.flash_state = True
        self.set_chain()
        self.current_state = "flash_color"

    def purple_chaser(self):
        """Configure the LED code for a purple chaser (2023 Charged Up Default)."""
        if self.timer.get() - self.animation_delay > self.record_time:
            self.m_ledBuffer = self.purple_pattern
            self.purple_pattern = self.purple_pattern[1:] + self.purple_pattern[:1]
            self.record_time = self.timer.get()
        # self.set_chain_with_notifier()
        self.set_chain()
        self.current_state = "purple_chaser"

    def heading_lock(self, heading):
        """Configure the LED code for a demo of robot heading tracking."""
        pos = int(-1 * (heading * self.length / 360) + self.length/2)
        if pos > 143:
            pos = 143
        if pos < 3:
            pos = 3
        heading_pattern = [AddressableLED.LEDData(0, 0, 0)] * (pos - 2)
        heading_pattern = heading_pattern + [AddressableLED.LEDData(255, 0, 0)] * 5
        heading_pattern = heading_pattern + [AddressableLED.LEDData(0, 0, 0)] * (self.length-len(heading_pattern))
        self.m_ledBuffer = heading_pattern
        self.set_chain()
        self.current_state = "heading_lock"

    def fire(self, color: [], inverted: bool):

        if self.timer.get() - self.animation_delay > self.record_time:
            # Cool current strip
            for i in range(0, self.length):
                self.heat[i] = max(0, self.heat[i] - random.randint(0, int(5/self.length + 2)))

            # Generate random sparks in the strip
            for j in range(2, self.length - 2):
                if random.randint(0, 1000) >= 800:
                    self.heat[j] = random.randint(160, 255)
                    self.heat[j + 1] = random.randint(self.heat[j] - 50, self.heat[j])
                    self.heat[j + 2] = random.randint(self.heat[j] - 100, self.heat[j])
                    self.heat[j - 1] = random.randint(self.heat[j] - 50, self.heat[j])
                    self.heat[j - 2] = random.randint(self.heat[j] - 100, self.heat[j])
            if inverted:
                for m in range(0, self.length):
                    self.heat[m] = min(255, int((1 - ((self.length - m) / self.length)) * self.heat[m] * 1.3))
            else:
                for m in range(0, self.length):
                    self.heat[m] = min(255, int(((self.length - m) / self.length) * self.heat[m] * 1.3))

            # Transform brightness into color
            temp_buffer = [AddressableLED.LEDData(0, 0, 0)] * self.length
            if self.style == "RGB":
                for k in range(0, self.length):
                    r = int((self.heat[k] / 255) * color[0])
                    g = int((self.heat[k] / 255) * color[1])
                    b = int((self.heat[k] / 255) * color[2])
                    temp_buffer[k] = AddressableLED.LEDData(r, g, b)
            elif self.style == "GRB":
                for k in range(0, self.length):
                    g = int((self.heat[k] / 255) * color[0])
                    r = int((self.heat[k] / 255) * color[1])
                    b = int((self.heat[k] / 255) * color[2])
                    temp_buffer[k] = AddressableLED.LEDData(r, g, b)
            self.record_time = self.timer.get()
            self.m_ledBuffer = temp_buffer
        self.set_chain()
        self.current_state = "fire"

    def shoot_animator(self, speed: str):
        self.shooting = True
        if speed == "fast":
            index = 4
        elif speed == "fastest":
            index = 6
        else:
            index = 2
        if self.timer.get() - 0.02 > self.record_time:
            self.shooting_counter += index
            self.m_ledBuffer = self.shoot_pattern
            self.shoot_pattern = self.shoot_pattern[index:] + self.shoot_pattern[:index]
            self.record_time = self.timer.get()
        self.set_chain()
        self.current_state = "shooting"
        if self.shooting_counter >= self.length:
            self.shooting_counter = 0
            self.m_ledBuffer = [AddressableLED.LEDData(0, 0, 0)] * self.length
            self.set_chain()
            self.shooting = False
            self.shoot_pattern = [AddressableLED.LEDData(142, 254, 15)] * 5
            for i in range(0, self.length - 5):
                self.shoot_pattern.append(AddressableLED.LEDData(0, 0, 0))

    def amp_timer(self) -> None:
        self.amp_timer_on = True
        if self.timer.get() - 1 > self.record_time:
            self.m_ledBuffer = self.amp_pattern_displayed
            self.amp_pattern_displayed = [AddressableLED.LEDData(0, 255, 0)] * (8 - self.amp_index) * self.amp_length
            for i in range(0, self.amp_index * self.amp_length):
                self.amp_pattern_displayed.append(AddressableLED.LEDData(0, 0, 0))
            self.record_time = self.timer.get()
            self.amp_index += 1
        if self.amp_index == 10:
            self.amp_timer_on = False
            self.amp_index = 0
            self.amp_pattern_displayed = [AddressableLED.LEDData(0, 255, 0)] * (self.length - (self.amp_length * 2))
            for i in range(0, self.amp_length * 2):
                self.amp_pattern_displayed.append(AddressableLED.LEDData(0, 0, 0))
        self.set_chain()
