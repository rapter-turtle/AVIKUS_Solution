function y=minmax(min, x, max)
% sat is the saturation function with unit limits and unit slope.
for i = 1:length(x)
    if x(i)>max
        y(i)=max;
    elseif x(i)<min 
        y(i)=min;
    else 
        y(i)=x(i);
    end
end