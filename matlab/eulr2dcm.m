function DCM = eulr2dcm(eul)

    % Ensure proper input
    if nargin<1
        error('insufficient number of input arguments')
    else
    
        % Extract states
        phi = eul(1); % Roll
        theta = eul(2); % Pitch
        psi = eul(3); % Yaw
    
        % First rotation (Yaw)
        C1 = [cos(psi) -sin(psi) 0;
              sin(psi)  cos(psi) 0;
              0         0        1];
    
        % Second Rotation (Pitch)
        C2 = [cos(theta)  0   sin(theta);
              0           1   0;
             -sin(theta)  0   cos(theta)];

        % Third rotation (Roll)
        C3 = [1   0         0;
              0   cos(phi) -sin(phi);
              0   sin(phi)  cos(phi)];

        % Transformation Matrix
        DCM = C1*C2*C3;

end