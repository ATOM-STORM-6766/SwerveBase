import time

from phoenix6 import StatusCode

import wpilib
from wpilib import Timer


class DeviceConfiguration:
    """This interface is where all of our logic for configuring each device goes."""

    def configureSettings(self) -> bool:
        """
        This method does all of the configuration logic for the device and returns
        true only if the configuration is good.
        """
        pass


class ErrorChecker:
    BOOT_ALLOWANCE_SECONDS = 3.0
    TRIES_TO_GENERATE_WARNING = 5

    @staticmethod
    def hasConfiguredWithoutErrors(*status_codes: StatusCode) -> bool:
        """This method takes a list of StatusCodes and returns true if they are all OK.
        When we configure our devices, we wrap all our our calls to the devices in this method
        to tell us if the device has configured correctly, or if there are errors."""
        okay = True
        for status_code in status_codes:
            okay = okay and status_code == StatusCode.OK
        return okay

    @staticmethod
    def configureDevice(
        config: DeviceConfiguration,
        name: str,
        boot_allowance_seconds: float = BOOT_ALLOWANCE_SECONDS,
        print_info: bool = True,
    ):
        """
        This method does the actual configuration for the device.
        It repeatedly calls `config.configureSettings()` until there is a successful configuration or until it times out.
        If `printInfo` is true, it will print if the configuration succeeded and how many tries it took.
        """
        good_configuration = False
        timer = Timer()
        timer.reset()
        timer.start()

        tries = 0
        while not good_configuration:
            tries += 1

            if timer.get() > boot_allowance_seconds:
                if print_info:
                    wpilib.reportError(
                        f"failed configuration for {name} initialization, took {tries} tries.",
                        False,
                    )
                return
            good_configuration = config.configureSettings()
            if not good_configuration:
                try:
                    time.sleep(0.1)
                except Exception as e:
                    print(e)

        if print_info and tries > ErrorChecker.TRIES_TO_GENERATE_WARNING:
            wpilib.reportError(
                f"Possible issue with {name}. Configuration took {tries} tries", False
            )
        elif print_info:
            print(f"[Config] {name} | configuration took {tries} tries.")
