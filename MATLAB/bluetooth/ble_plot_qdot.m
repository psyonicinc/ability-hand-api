function [saved_data, time] = ble_plot_qdot(hand_name, buf_width)
[saved_data, time]= ble_plot_float(hand_name,"P8",6,buf_width,-100.1,100.1);
end
