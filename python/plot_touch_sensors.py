from ah_wrapper.ah_serial_client import AHSerialClient
from ah_wrapper.ah_api import create_pos_msg
from plotting.plots import RealTimePlotTouch

if __name__ == "__main__":
    client = AHSerialClient(baud_rate=460800)
    msg = create_pos_msg(reply_mode=1, positions=[40, 10, 40, 10, 30, -30])
    client.set_command(msg)
    plotter = RealTimePlotTouch(client.hand)
    try:
        plotter.start()
    except KeyboardInterrupt:
        pass
    finally:
        client.close()
