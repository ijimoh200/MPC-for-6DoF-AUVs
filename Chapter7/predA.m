function A_p = predA(Geta,Aeta,N)
temp = [];
for i=1:N
 temp = [temp; Geta*Aeta^i];
end
A_p = temp;
end