
% A = [zeros(3) eye(3) zeros(3,6);
%     zeros(3,12);
%     zeros(3,9) eye(3);
%     zeros(3,12)]

A = [zeros(6) eye(6);
     zeros(6) zeros(6)];



 % C = [eye(3) zeros(3,9); 
 %      zeros(3,9) eye(3)]

 C=[eye(6) zeros(6)];

%calculating Lyap eq

delta = 0.1;
 
 while (delta>0 && delta<3)      
 
   C_hat= -2*transpose(C)*C;
   A_hat=transpose(A)+delta;
   B=A;

    P= sylvester(A_hat,B,C_hat); %sylvester equation

    try chol(P'*P)
        disp('Matrix is positive definite');
        K= inv(P)*transpose(C)
        
        disp ('delta=')
        disp (delta)
        break
    catch ME
        disp('Matrix is not positive definite')
    end

     % if (eig(0.5*(P+P'))>0)
     %    K= inv(P)*transpose(C)
     % end

    delta=delta+0.1;

 end