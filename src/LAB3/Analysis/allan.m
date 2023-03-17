function [ad,ade,taus] = allan(data,tau)

  % normalize data
  data = data-mean(data);

  % compute Allan deviation
  taus = unique(round(logspace(0,log10(length(data)/4),50)));
  ad = zeros(1,length(taus));
  for i=1:length(taus)
    tau = taus(i);
    ad(i) = sqrt(0.5/((length(data)-2*tau)/tau)/tau*sum(diff(data(1:end-2*tau)).*diff(data(1+2*tau:end))));
  end

  % compute Allan deviation error
  if nargout>1
    ade = ad./sqrt(length(data)/2);
  end
end
