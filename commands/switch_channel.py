import commands2
from subsystems.utilsubsystem import UtilSubsystem


class SwitchPDHChannel(commands2.Command):

    def __init__(self, on: bool, util: UtilSubsystem):
        super().__init__()
        self.on = on
        self.util = util
        self.addRequirements(util)

    def initialize(self):
        self.util.toggle_channel(self.on)

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return True
