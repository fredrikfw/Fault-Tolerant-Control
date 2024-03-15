function J = decoupling_matrix(phi,theta,psi,T,Ix,Iy,Iz,m)

J = [ -1/m*(cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi))             -T/(m*Ix)*(sin(psi)*cos(phi)-cos(psi)*sin(theta)*sin(phi))           -T/(m*Iy)*(cos(psi)*cos(theta)*cos(phi))       -T/(m*Iz)*(cos(psi)*sin(phi) - sin(psi)*sin(theta)*cos(phi));
      -1/m*(sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi))             -T/(m*Ix)*(-cos(psi)*cos(phi)-sin(psi)*sin(theta)*sin(phi))          -T/(m*Iy)*(sin(psi)*cos(theta)*cos(phi))       -T/(m*Iz)*(sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi));
      -1/m*(cos(theta)*cos(phi))                                        T/(m*Ix)*(cos(theta)*sin(phi))                                        T/(m*Iy)*(sin(theta)*cos(phi))                 0;
        0                                                                  0                                                                  0                                              1/Iz];
    
end