import time


def notify_component_start(component_name):
    print("***************************************************************")
    print("     Starting {} component".format(component_name))
    print("***************************************************************")


class FrequencyTimer(object):
    def __init__(self, frequency_rate):
        self.time_available = 1e9 / frequency_rate

    def start_loop(self):
        self.start_time = time.time_ns()

    def check_time(self, frequency_rate):
        # if prev_check_time variable doesn't exist, create it
        if not hasattr(self, "prev_check_time"):
            self.prev_check_time = self.start_time

        curr_time = time.time_ns()
        if (curr_time - self.prev_check_time) > 1e9 / frequency_rate:
            self.prev_check_time = curr_time
            return True
        return False

    def end_loop(self):
        wait_time = self.time_available + self.start_time

        while time.time_ns() < wait_time:
            continue
