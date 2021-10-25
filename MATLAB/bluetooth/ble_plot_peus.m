function [saved_data, time] = ble_plot_peus(hand_name, buf_width)
    [saved_data, time] = ble_plot_float(hand_name,"P6",6,buf_width,0,10);
end
