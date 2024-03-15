function utilde = dynamic_compensator(state,v)

%% initializing variables

phi = state(7); theta = state(8); psi = state(9);
phidot = state(10); thetadot = state(11); psidot = state(12);

global m g J
Ix = J(1,1); Iy = J(2,2); Iz = J(3,3);

T = state(13);
Tdot = state(14);

%% Regular state feedback
sph=sin(phi);cph=cos(phi);st=sin(theta);ct=cos(theta);sps=sin(psi);cps=cos(psi); 

Jinv = [-m*(sph*sps+cph*cps*st)        m*(cps*sph-cph*sps*st)         -m*cph*ct       0;
         -m*Ix/T*(cph*sps-cps*sph*st)   m*Ix/T*(cph*cps+sph*sps*st)   m*Ix/T*ct*sph   Ix*st;
         -m*Iy/T*cps*ct/cph             -m*Iy/T*ct*sps/cph            m*Iy/T*st/cph   -Iy*ct*tan(phi);
         0                              0                             0               Iz              ];         

l1 = -2*Tdot/m*((cps*sph-sps*st*cph)*psidot+(cps*ct*cph)*thetadot+(sps*cph-cps*st*sph)*phidot)-...
     T/m*((-sps*sph*psidot+cps*cph*phidot-cps*st*cph*psidot-sps*ct*cph*thetadot+sps*st*sph*psidot)*psidot+...
     (-sps*ct*cph*psidot-cps*st*cph*thetadot-cps*ct*sph*phidot)*thetadot+...
     (cps*cph*psidot-sps*sph*phidot+sps*st*sph*psidot-cps*ct*sph*thetadot-cps*st*cph*phidot)*phidot);
 
l2 = -2*Tdot/m*((cps*st*cph+sph*sps)*psidot+(sps*ct*cph)*thetadot+(-sps*st*sph-cph*cps)*phidot)-...
     T/m*((-sps*st*cph*psidot+cps*ct*cph*thetadot-cps*st*sph*phidot+cph*sps*phidot+sph*cps*psidot)*psidot+...
     (cps*ct*cph*psidot-sps*st*cph*thetadot-sps*ct*sph*phidot)*thetadot+...
     (-cps*st*sph*psidot-sps*ct*sph*thetadot-sps*st*cph*phidot+sph*cps*phidot+cph*sps*psidot)*phidot);
 
l3 = -2*Tdot/m*(-st*cph*thetadot-ct*sph*phidot)-...
     T/m*((-ct*cph*thetadot+st*sph*phidot)*thetadot+(st*sph*thetadot-ct*cph*phidot)*phidot);
 
l4 = 0;

l = [l1 l2 l3 l4]';

utilde = Jinv*(-l+v);
   
end

