import wpilib
import math


class CustomHID:
    controller_type: str
    direction = 0

    def __init__(self, port: int, hid: str) -> None:
        super().__init__()
        if hid == "xbox":
            self.controller = wpilib.XboxController(port)
            self.controller_type = "xbox"
        elif hid == "ps4":
            self.controller = wpilib.PS4Controller(port)
            self.controller_type = "ps4"
        else:
            self.controller = wpilib.Joystick(port)
            self.controller_type = "generic"

    def reset_controller(self, hid, port):
        if hid == "xbox":
            self.controller = wpilib.XboxController(port)
            self.controller_type = "xbox"
        elif hid == "ps4":
            self.controller = wpilib.PS4Controller(port)
            self.controller_type = "ps4"
        else:
            self.controller = wpilib.Joystick(port)
            self.controller_type = "generic"

    def get_button(self, button: str) -> bool:
        value = False
        if self.controller_type == "xbox":
            if button == "A":
                value = self.controller.getAButton()
            if button == "B":
                value = self.controller.getBButton()
            if button == "X":
                value = self.controller.getXButton()
            if button == "Y":
                value = self.controller.getYButton()
            if button == "LB":
                value = self.controller.getLeftBumper()
            if button == "RB":
                value = self.controller.getRightBumper()
            if button == "VIEW":
                value = self.controller.getBackButton()
            if button == "MENU":
                value = self.controller.getStartButton()
            if button == "LTHUMB":
                value = self.controller.getLeftStickButton()
            if button == "RTHUMB":
                value = self.controller.getRightStickButton()
        if self.controller_type == "ps4":
            if button == "A":
                value = self.controller.getCrossButton()
            if button == "B":
                value = self.controller.getCircleButton()
            if button == "X":
                value = self.controller.getSquareButton()
            if button == "Y":
                value = self.controller.getTriangleButton()
            if button == "LB":
                value = self.controller.getL1Button()
            if button == "RB":
                value = self.controller.getR1Button()
            if button == "VIEW":
                value = self.controller.getShareButton()
            if button == "MENU":
                value = self.controller.getOptionsButton()
            if button == "LTHUMB":
                value = self.controller.getL3Button()
            if button == "RTHUMB":
                value = self.controller.getR3Button()
            if button == "TOUCHPAD":
                value = self.controller.getTouchpad()
            if button == "PS":
                value = self.controller.getPSButton()
        return value

    def get_trigger(self, trigger: str, threshold: float) -> bool:
        value = False
        if self.controller_type == "xbox" or "generic":
            if trigger == "R":
                if self.controller.getRawAxis(3) >= threshold:
                    value = True
            if trigger == "L":
                if self.controller.getRawAxis(2) >= threshold:
                    value = True
        if self.controller_type == "ps4":
            if trigger == "R":
                axis = self.controller.getR2Axis()
                if axis < 0:
                    axis = (axis * -1) * 0.5
                else:
                    axis = (axis * 0.5) + 0.5
                if axis >= threshold:
                    value = True
            if trigger == "L":
                axis = self.controller.getL2Axis()
                if axis < 0:
                    axis = (axis * -1) * 0.5
                else:
                    axis = (axis * 0.5) + 0.5
                if axis >= threshold:
                    value = True
        return value

    def get_axis(self, axis: str, deadband: float) -> float:
        value = 0.0
        if self.controller_type == "xbox" or "generic":
            if axis == "LX":
                if abs(self.controller.getRawAxis(0)) >= deadband:
                    value = self.controller.getRawAxis(0)
            if axis == "LY":
                if abs(self.controller.getRawAxis(1)) >= deadband:
                    value = self.controller.getRawAxis(1)
            if axis == "RX":
                if abs(self.controller.getRawAxis(4)) >= deadband:
                    value = self.controller.getRawAxis(4)
            if axis == "RY":
                if abs(self.controller.getRawAxis(5)) >= deadband:
                    value = self.controller.getRawAxis(5)
        if self.controller_type == "ps4":
            if axis == "LX":
                if abs(self.controller.getRawAxis(0)) >= deadband:
                    value = self.controller.getRawAxis(0)
            if axis == "LY":
                if abs(self.controller.getRawAxis(1)) >= deadband:
                    value = self.controller.getRawAxis(1)
            if axis == "RX":
                if abs(self.controller.getRawAxis(2)) >= deadband:
                    value = self.controller.getRawAxis(2)
            if axis == "RY":
                if abs(self.controller.getRawAxis(5)) >= deadband:
                    value = self.controller.getRawAxis(5)
        return value

    def get_d_pad(self) -> str:
        value = "Z"
        if self.controller.getPOV() == 0:
            value = "N"
        if self.controller.getPOV() == 45:
            value = "NE"
        if self.controller.getPOV() == 90:
            value = "E"
        if self.controller.getPOV() == 135:
            value = "SE"
        if self.controller.getPOV() == 180:
            value = "S"
        if self.controller.getPOV() == 225:
            value = "SW"
        if self.controller.getPOV() == 270:
            value = "W"
        if self.controller.getPOV() == 315:
            value = "NW"
        return value

    def get_d_pad_pull(self, direction: str):
        if self.get_d_pad() == direction:
            return True
        else:
            return False

    def set_rumble(self, strength: float) -> None:
        if self.controller_type == "xbox":
            self.controller.setRumble(wpilib.XboxController.RumbleType.kBothRumble, strength)
        if self.controller_type == "ps4":
            self.controller.setRumble(wpilib.PS4Controller.RumbleType.kBothRumble, strength)

    def get_controller(self):
        return self.controller

    def dir_est_ctrl(self, stick: str):
        """Directional estimation control. Transforms a thumbstick input into an estimated 'direction'."""
        if stick == "R":
            x_ax = -1 * self.get_axis("RX", 0.1)
            y_ax = self.get_axis("RY", 0.1)
        else:
            x_ax = -1 * self.get_axis("LX", 0.1)
            y_ax = self.get_axis("LY", 0.1)
        if math.sqrt(x_ax * x_ax + y_ax * y_ax) >= 0.99:
            self.direction = math.degrees(math.atan2(x_ax, y_ax)) - 180
            if self.direction < -180:
                self.direction += 360

        # print("DIRECTION: " + str(self.direction))
        return self.direction
