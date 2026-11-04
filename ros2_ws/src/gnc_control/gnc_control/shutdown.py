"""Clean stop of the nodes on Ctrl+C or on a stop request (no ROS dependency).

With a launch file, a Ctrl+C reaches each node twice: from the terminal and from the launch file. The second signal
would interrupt the stop half way, for example before the servo outputs are disabled. So the first signal ends
rclpy.spin(), and the next ones are ignored while the node finishes.
"""
import signal


def _stop(signum, frame):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    raise KeyboardInterrupt


def install_stop_handlers():
    """To call after rclpy.init(signal_handler_options=SignalHandlerOptions.NO): ROS then stays alive during the stop."""
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
