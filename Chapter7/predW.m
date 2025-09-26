function w_p =predW(C,N, Nu)
[p,n]=size(C);
barC_= zeros(N*p,Nu*n);
firstCol = [];
for i=1:N
firstCol = [firstCol ;
C];
end
barC_(:,1:n) = firstCol;
 for col=1:(Nu-1)*n
 for row=1:(N-1)*p
barC_(row+p, col+n)=barC_(row,col);
 end
 end
w_p = barC_;