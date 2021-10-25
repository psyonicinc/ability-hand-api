%% Set the name of the hand to connect to
hand_name = "PSYONIC-RUDE_BOY"; 

%% Connect to the hand

bltable = blelist;
addr = 0;
for i = 1:size(bltable,1)
    str = bltable(i,:).Name;
    if(str == hand_name)
        addr = bltable(i,:).Address;
        break;
    end
end

try
    b=ble(addr);
catch
    disp("ERR: hand name not found");
    return;
end

w_chr = characteristic(b,"6E400001-B5A3-F393-E0A9-E50E24DCCA9E","6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
n_chr = characteristic(b, "6E400001-B5A3-F393-E0A9-E50E24DCCA9E", "6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
subscribe(n_chr, "notification");

%% Simple example use of ble_fctl

per = ones(1,6)*.2;
dwn = 45;
up = 15;
del = 0.2;
ble_fctl(w_chr,[dwn,up,up,up,5,-0.1],per);
pause(del);
ble_fctl(w_chr,[up,up,dwn,up,5,-0.1],per);
pause(del);
ble_fctl(w_chr,[up,dwn,up,up,5,-0.1],per);
pause(del);
ble_fctl(w_chr,[up,up,up,dwn,5,-0.1],per);
pause(del);
ble_fctl(w_chr,[up,up,up,up,55,-50],per);
pause(del);
ble_fctl(w_chr,[up,up,up,up,5,-0.1],per);
pause(del);

%% Example use of fctl, waggle demo
period = 2;         %This parameter is user-modifiable

% Initialize some helper variables to run this demo
num_phases = 6;     %six motors, each moves out of phase
interval = period/num_phases;   %this is the approximate time we will wait between consecutive calls to ble_fctl
assert(interval > .15, 'Error: period is too low!');    %we can only call ble_fctl about once every .15seconds
up_down_flag = zeros(1,num_phases); %keep track of which fingers are up and which are down
prev_phase_num = -1;    %mark previous phase
prev_send_ts = 0;   %used to display the time in seconds between each bluetooth fctl transmission

% Create a figure so that we can close it to end the program
f = figure(1);
pause(.1);
tic;    %begin timestamp

while(ishandle(f))
    
    tmod = mod(toc,period);    %tmod goes 0-period, forever and ever
    phase_num = floor(tmod*num_phases/period);  %mark where in the period you are, subdivided by the number of phases
    
    if(phase_num ~= prev_phase_num)
        
        %the start of each phase should correspond to the transmission of a
        %new sepoint. Pick the two indices to change. Desired sequence is:
        %(1,4), (2,5), (3,6), (4,1), (5,2), (6,3)
        first = phase_num+1;    
        second = mod(phase_num + num_phases/2, num_phases)+1;   %
        
        up_down_flag(first) = 1;    %1 -> 'up'
        up_down_flag(second) = 0;   %0 -> 'down'
        
        % map up/down to degree setpoints
        fpos = 50*up_down_flag + 10;    %move between 10-60 degrees
        fpos(6) = -fpos(6);         %-10 and -60 for the thumb rotator
        
        %set the period so that it has *just* finished tracking your
        %setpoint by the time you come around with a new one
        fper = ones(1,6)*period/2;
        
        %transmit the new command, and print to console to give some idea
        %of what's going on in this demo
        fprintf("write:pos = [");
        fprintf("%.2f,", fpos);
        fprintf("], period = [");
        fprintf("%.2f,",fper);
        fprintf("], delay = %f\n",toc-prev_send_ts);
        ble_fctl(w_chr,fpos,fper);
        prev_send_ts = toc;
    end
    prev_phase_num = phase_num; %keep track of the previous phase number so we can act on the transitions

    pause(.001);    %pause allows us to cancel by closing the figure.
end


