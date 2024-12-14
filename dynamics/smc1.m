function T = smc1(z)
%SMC1 Summary of this function goes here
% if z>0.55
%     c=0.55;
% elseif z<-0.55
%     c=-0.55;
% else
%     c=z;
% end
T=z*sign(z)
end

