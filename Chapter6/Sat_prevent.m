function [yr] = Sat_prevent(yauv,yref,R,Ts)

x1 = yauv(1); y1 = yauv(2);
xd = yref(1); yd = yref(2);

if (x1 - xd)^2 + (y1 - yd)^2 <= (R)^2

    yr = yref;

else

    yr = (yref - yauv)*Ts + yauv;

%     ydd = (yref - y1)*2*Ts + y1;

%     yr = [xdd ydd]';

end





% % %     F = @(q)[(y - q(2)) - k1*(x - q(1));
% % %         (q(1) - x)^2 + (q(2) - y)^2 - (n*tilde_r)^2];
% % % 
% % %     x0 = [(xd-x)/2; (yd-y)/2];
% % %     sol = fsolve(F,x0);


    %
    %         if (y(1,:)-yref(1,:))^2 +(y(2,:)-yref(2,:))^2 <= (n*1)^2
    %
    %             yr = [yref; 0];
    %
    %         else
    %
    %             k1 = (yref(2,1)-yref_(2,1))/(yref(1,1)-yref_(1,1));
    %
    %             Fun = @(V) [y(2,1) - V(2) - k1*(y(1,1) - V(1)); (y(1,1)-V(1))^2 +(y(2,1)-V(2,1))^2 - (n*1)^2];
    %
    %             InitialGuess = [y(1); y(2)];
    % Options = optimset('Display','iter');
    % yr_ = fsolve(Fun, InitialGuess, Options);


    %
    %
    %             fun = @(x)((x(1)+x(2)));
    %             k1 = (yref(2,:)-yref_(2,:))/(yref(1,:)-yref_(1,:));
    %             Aeq = [k1 -1];
    %             beq = k1*y(1,:)-y(2,:);
    %             circl = @(x)circle(x,y,n);
    %             nonlcon = circl;
    %             x0 = [y(1); y(2)];

    %             yr_ = fmincon(fun,x0,[],[],Aeq,beq,[],[],nonlcon);

% % %     phi_d = atan2((sol(2)-y),(sol(1)-x));
% % % 
% % %     yr = [sol; phi_d];

% end










% function [yr] = Sat_prevent(y,yref,yref_,n)
% 
%         if (y(1,:)-yref(1,:))^2 +(y(2,:)-yref(2,:))^2 <= (n*1)^2
% 
%             yr = [yref; 0];
% 
%         else
% 
%             fun = @(x)(-(x(1)+x(2)));
%             k1 = (yref(2,:)-yref_(2,:))/(yref(1,:)-yref_(1,:));
%             Aeq = [k1 -1];
%             beq = k1*y(1,:)-y(2,:);   
%             circl = @(x)circle(x,y,n);
%             nonlcon = circl;
%             x0 = [y(1); y(2)];
% 
%             yr_ = fmincon(fun,x0,[],[],Aeq,beq,[],[],nonlcon);  
% 
%             phi_d = atan2((yr_(2)-y(2)),(yr_(1)-y(1)));
% 
%             yr = [yr_; phi_d];
% 
%         end
% 
%         
% 
% end