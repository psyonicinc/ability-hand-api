function [saved_data, time] = ble_plot_cost_cooled(hand_name, buf_width)
    [saved_data, time] = ble_plot_float(hand_name,"PA",6,buf_width,0,15);
end
