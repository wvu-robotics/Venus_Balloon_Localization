function NED = LLH2NED(LLH, refLLH)
    
    % Constants
    R = 6052000.0; % Radius of Venus in meters

    % Preallocate NED for output
    NED = zeros(size(LLH));

    % Extract reference latitude (lat), longitude (lon), and height (h)
    refLat = refLLH(1);
    refLon = refLLH(2);
    refH = refLLH(3);

    % Convert reference geodetic coordinates to ECEF
    cosRefLat = cos(refLat);
    sinRefLat = sin(refLat);
    cosRefLon = cos(refLon);
    sinRefLon = sin(refLon);
    N_ref = R + refH;
    refECEF = [N_ref * cosRefLat * cosRefLon; N_ref * cosRefLat * sinRefLon; N_ref * sinRefLat];

    % Transformation matrix from ECEF to NED at the reference point
    T = [-sinRefLat * cosRefLon, -sinRefLat * sinRefLon, cosRefLat;
         -sinRefLon,             cosRefLon,            0;
         -cosRefLat * cosRefLon, -cosRefLat * sinRefLon, -sinRefLat];

    % Iterate over each column if LLH has multiple columns
    for i = 1:size(LLH, 2)
        
        % Extract LLH components for current column
        lat = LLH(1, i);
        lon = LLH(2, i);
        h = LLH(3, i);

        % Convert LLH to ECEF
        cosLat = cos(lat);
        sinLat = sin(lat);
        cosLon = cos(lon);
        sinLon = sin(lon);
        N = R + h;
        ECEF = [N * cosLat * cosLon; N * cosLat * sinLon; N * sinLat];

        % Calculate ECEF coordinates relative to the reference point
        deltaECEF = ECEF - refECEF;

        % Convert the ECEF vector to NED
        deltaNED = T * deltaECEF;

        % Store the NED coordinates
        NED(:, i) = deltaNED;
    end
end