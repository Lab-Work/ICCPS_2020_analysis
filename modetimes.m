function [t0,t1] = modetimes(modeTimeseries,modenum)
%Returns the start and end values of the mode bag times.
%   Detailed explanation goes here

x = find(modeTimeseries.Data == modenum);
t0 = modeTimeseries.Time(x(1));
t1 = modeTimeseries.Time(x(length(x)));

end

