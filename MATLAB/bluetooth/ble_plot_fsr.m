function [saved_data, time] = ble_plot_fsr(hand_name, buf_width)

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

pause(.5);
write(w_chr, "P4");


f = figure(1);
H = uicontrol();
H.Position = [0,0,0,0];

num_lines = 30;

for i = 1:num_lines
    anim_line(i) = animatedline;
end

anim_line(1) = animatedline('Color',[.5 0 0] );
if(num_lines >= 3)
    anim_line(2) = animatedline('Color',[0 .5 0]);
    anim_line(3) = animatedline('Color', [0,0,.5] );
end

grid on;
tic;

linebuf = zeros(num_lines,buf_width);
timebuf = zeros(1,buf_width);

ylowerlim = 0;
yupperlim = 4095;
ylim([ylowerlim,yupperlim]);
xlim([0,buf_width]);
tic;

% data = zeros(num_lines,1);
% data_samp = zeros(15,1);
xidx = 0;
while(ishandle(H))

    data_samp = read(n_chr, 'latest');
    len = length(data_samp);
    if(len == 45)
        
        u8arr = uint8(data_samp);
        %data = typecast(u8arr,'single');
        data = unpack_8bit_into_12bit(u8arr, 30);

        if(isempty(data) ~= 1 && size(data,2) == num_lines)
            set(gcf,'Renderer','OpenGL');
            
            xidx = xidx + 1;
            for i = 1:num_lines
                v = double(data(i));
                if(ishandle(anim_line(i)))
                    addpoints(anim_line(i), xidx, v);
                    linebuf(i,xidx) = v;
                    timebuf(xidx) = toc;
                end
            end

            drawnow limitrate nocallbacks

            if(xidx >= buf_width)
                for i = 1:num_lines
                    clearpoints(anim_line(i));
                end
                xidx = 0;
                saved_data = linebuf;
                time = timebuf;
                linebuf = zeros(num_lines,buf_width);
            end
            
    %         timebuf(2:end) = timebuf(1:end-1);
    %         timebuf(1) = toc;
        end
    else
        disp("size mismatch");
    end    
end

write(w_chr, "P1");
unsubscribe(n_chr);
clear b;
clear w_chr;
clear n_chr;

end
