function B_p =predB1(A,B,N, Nu)
[n,m]=size(B);
barC_= zeros(N*n,Nu*m);
firstCol = [];
for i=1:N
firstCol = [firstCol ;
A^(i-1)*B];
end
barC_(:,1:m) = firstCol;
 for col=1:(Nu-1)*m
 for row=1:(N-1)*n
barC_(row+n, col+m)=barC_(row,col);
 end
 end
B_p = barC_;
end