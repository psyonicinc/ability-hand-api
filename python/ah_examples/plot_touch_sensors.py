from ah_wrapper.ah_serial_client import AHSerialClient
from ah_wrapper.ah_api import create_pos_msg
from ah_plotting.plots import RealTimePlotTouch


def main():
    client = AHSerialClient()
    msg = create_pos_msg(reply_mode=1, positions=[40, 10, 40, 10, 30, -30])
    client.set_command(msg)
    plotter = RealTimePlotTouch(client.hand)
    try:
        plotter.start()
    except KeyboardInterrupt:
        pass
    finally:
        client.close()


if __name__ == "__main__":
    main()
