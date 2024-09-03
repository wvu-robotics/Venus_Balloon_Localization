function eul = dcm2eulr(DCM)

    % Ensure proper input
    if nargin<1
        error('insufficient number of input arguments')
    elseif ~isreal(DCM)
        eul = NaN(3,1);
    else

        % Roll Angle
        phi = atan2(DCM(3,2),DCM(3,3));

        % Pitch Angle
        theta = -asin(DCM(3,1));

        % Yaw Angle
        psi = atan2(DCM(2,1),DCM(1,1));
    
        % Euler Angles
        eul = [phi theta psi]';
    
    end

