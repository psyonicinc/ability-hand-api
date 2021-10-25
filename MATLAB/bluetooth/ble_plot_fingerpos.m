function [saved_data, time] = ble_plot_fingerpos(hand_name, buf_width)
    [saved_data, time] = ble_plot_float(hand_name,"P5",6,buf_width,-0.01,0.01);
end
