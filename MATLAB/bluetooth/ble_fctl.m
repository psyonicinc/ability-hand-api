% This function uses an established write characteristic to 
% do individual finger control of a psyonic hand
%
%   INPUTS: 
%       -w_chr:
%           Matlab ble write characteristic, correctly established to
%           connect to the hand in question.
%       -fpos: 
%          Array of 6 setpoint values, one per finger. Order is 
%          as follows:
%               [index][middle][ring][pinky][thumb-flexor][thumb-rotator]
%          fpos(1:5) must be between 0 and 150.
%          fpos(6) must be between 0 and -150
%       
%       -fper:
%           Array of 6 period values, one per finger. Order is the 
%           same as for fpos. Period is in seconds. This is the time
%           the corresponding finger will take to go from where it 
%           currently is, to where you told it to go with fpos.
%
%           Valid range is 0.2 - 300. Values out of range will be
%           clamped.
function ble_fctl(w_chr, fpos, fper)

    assert(size(fpos,2) == 6, 'Error: incorrect size fpos arg');
    assert(size(fper,2) == 6, 'Error: incorrect size fper arg');
    assert(fpos(6) < 0, 'Error: thumb rotator setpoint is negative only');   % Thumb rotator setpoints must be less than 0!!!
    
    %threshold position vals
    fpos( fpos(1:5) > 150) = 150;
    fpos( fpos(1:5) < 0) = 0;
    fpos(fpos(6) > 0) = 0;
    fpos(fpos(6) < -150) = -150;
    
    %threshold period vals
    fper(fper(:) < 0.2) = 0.2;
    fper(fper(:) > 300) = 300;
    
    arr = uint8(zeros(1,25));   %create the write buffer
    arr(1) = 'M';   %first value is M, the header
    
    %load the write buffer
    M = int16([fpos'*(2^15-1)/150 fper'*(2^16-1)/300]); %TODO: clean this up
    for i = 1:6
        bs = 4; %byte size per-finger
        idx_range = (((i-1)*bs+1):(i*bs))+1;
        arr(idx_range) = typecast(int16(M(i,:)),'uint8');
    end
    
    %write the write buffer to the hand over ble
    write(w_chr, arr);
end