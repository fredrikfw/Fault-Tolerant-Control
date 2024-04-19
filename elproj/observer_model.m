function dobserver_model = observer_experimental(state, observer_state, u)

%% initializing variables
phi = observer_state(4); theta = observer_state(5); psi = observer_state(6); v = observer_state(7:9); p = observer_state(10); q = observer_state(11); r = observer_state(12); omega = observer_state(10:12);
global m g J Ix Iy Iz T;
cphi = cos(phi); sphi = sin(phi); ctheta = cos(theta); stheta = sin(theta); cpsi = cos(psi); spsi = sin(psi); ttheta = tan(theta);
f = u(1); M = u(2:4);

  
A = [zeros(6) eye(6);
     zeros(6) zeros(6)];

B = [zeros(9,4);  
     0 Ix 0 0; 
     0 0 Iy 0; 
     0 0 0 Iz];

C=[eye(6) zeros(6)];    

h = [zeros(6,1);
         -(T/m)*(cphi*ctheta*cpsi + spsi*sphi)
         -(T/m)*(cphi*stheta*spsi - cpsi*sphi)
         -(T/m)*(cphi*ctheta)
     zeros(3,1)];


%calculating Lyap eq

delta = 0.1;
 
 while (delta>0 && delta<3)      
 
   C_hat= -2*transpose(C)*C;
   A_hat=transpose(A)+delta;
   B=A;

    P= sylvester(A_hat,B,C_hat); %sylvester equation

    try chol(P'*P)
        disp('Matrix is positive definite');
        K= inv(P)*transpose(C);
       
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

%% function output ??

%dobserver_model = [v;dv;drpy;domega] + K*(state(1:12)-observer_state(1:12));
%dobserver_model = A*observer_state + B*u + h + K*(state(1:12)-observer_state(1:12));
dobserver_model = A * x_hat + h + B * u + K* (state_1_12 - C * x_hat)

end