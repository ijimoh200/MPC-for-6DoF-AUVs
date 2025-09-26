function A_p = predA1(Aeta,N)
temp = [];
for i=1:N
 temp = [temp; Aeta^i];
end
A_p = temp;
end