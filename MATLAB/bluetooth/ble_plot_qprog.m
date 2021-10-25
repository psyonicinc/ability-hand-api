function [saved_data, time] = ble_plot_qprog(hand_name, buf_width)
    [saved_data, time] = ble_plot_float(hand_name,"PD",6,buf_width,-1.1,1.1);
end
