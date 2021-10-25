function [saved_data, time] = ble_plot_float(hand_name, activation_str, num_floats, buf_width, lowerlim, upperlim)

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
write(w_chr, activation_str);

saved_data = 0;
time = 0;

f = uifigure(1);
H = uicontrol();
H.Position = [0,0,0,0];
lbl = uilabel(f, 'Position', [130,100,100,15]);
txt = uieditfield(f, 'ValueChangedFcn', @(txt,event) textChanged(txt,lbl, w_chr));

num_lines = num_floats;

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

ylowerlim = lowerlim;
yupperlim = upperlim;
ylim([ylowerlim,yupperlim]);
xlim([0,buf_width]);
tic;

xidx = 0;
while(ishandle(H))
    
    try
        din = read(n_chr, 'latest');
    catch
        disp("read timeout");
        break;
    end
    len = length(din);
    if(len == 4*num_floats)
        
        u8arr = uint8(din);
        data = typecast(u8arr,'single');

        if(isempty(data) ~= 1 && size(data,2) == num_lines)
            set(gcf,'Renderer','OpenGL');
            
            xidx = xidx + 1;
            for i = 1:num_lines
                v = double(data(i));
                if(v < ylowerlim)
                    ylowerlim = v-.05*abs(v);
                    ylim([ylowerlim,yupperlim]);
                end
                if(v > yupperlim)
                    yupperlim = 1.05*v;
                    ylim([ylowerlim,yupperlim]);
                end
                
                if(ishandle(anim_line(i)))
                    addpoints(anim_line(i), xidx, v);
                    linebuf(i,xidx) = v;
                    timebuf(xidx) = toc;
                end
            end

            drawnow limitrate nocallbacks

            if(xidx >= buf_width)
                
                yupperlim = 1.05*max(max(linebuf));
                minlim = min(min(linebuf));
                ylowerlim = minlim - (.05*abs(minlim));
                ylim([ylowerlim,yupperlim]);

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
%         disp("size mismatch");
        fprintf('%s\n', din);
    end
end
close(f);
close all;
write(w_chr, "P1");
unsubscribe(n_chr);
clear b;
clear w_chr;
clear n_chr;

end
