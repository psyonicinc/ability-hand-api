function [saved_data, time] = ble_plot_imu(hand_name, buf_width)
    [saved_data, time] = ble_plot_float(hand_name,"P9",7,buf_width,-10,10);
end
