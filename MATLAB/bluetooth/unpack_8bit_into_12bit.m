function vals = unpack_8bit_into_12bit(arr, valsize)
    
    arr = uint8(arr);
    vals = uint16(zeros(1,valsize));
    
    for bidx = (valsize * 12 - 4):-4:0
        validx = floor(bidx / 12);
        arridx = floor(bidx / 8);
        shift_val = mod(bidx, 8);
        
        tmp = bitshift( uint16(arr(arridx+1)) , -shift_val, 'uint16');
        tmp = bitand(tmp, uint16(0x0F), 'uint16');
        
        lshiftV = mod(bidx, 12);
        tmp = bitshift(tmp,lshiftV,'uint16');
        vals(validx + 1) = bitor( uint16(vals(validx + 1)), tmp);
    end
end