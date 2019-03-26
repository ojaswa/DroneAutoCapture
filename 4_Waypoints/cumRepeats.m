function cumrep = cumRepeats(rep)
cumrep = zeros(size(rep)); % Local prefix sum
i=0;
while sum(rep) >0
    i=i+1;
    h = [0; diff(rep)];
    h(h<0) = 0;
    p = find(h); % positions of first ones in repeats
    cumrep(p) = i;
    rep(p) = 0;
end