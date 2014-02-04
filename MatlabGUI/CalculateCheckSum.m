function [ LRC ] = CalculateCheckSum( buffer )
    LRC = 0;
    for i = 1:length(buffer)
        LRC = bitxor(buffer(i),LRC);
    end
end

