function LLH = NED2LLH(NED, refLLH)
    
    % Constants
    R = 6052000.0; % Radius of Venus in meters

    % Preallocate LLH for output
    LLH = zeros(size(NED));

    % Iterate over each column if NED has multiple columns
    for i = 1:size(NED, 2)

        % If inputs are not real
        if ~isreal(NED(:,i))

            % Give appropriate output
            LLH(:,i) = NaN(3,1);

        % If inputs are real
        else

            % Extract NED components for current column
            N = NED(1, i);
            E = NED(2, i);
            D = NED(3, i);

            % Extract reference latitude (lat), longitude (lon), and height (h)
            lat = refLLH(1);
            lon = refLLH(2);
            h = refLLH(3);

            % Convert reference geodetic coordinates to ECEF
            cosLat = cos(lat);
            sinLat = sin(lat);
            cosLon = cos(lon);
            sinLon = sin(lon);
            N_rad = R + h;
            refECEF = [N_rad * cosLat * cosLon; N_rad * cosLat * sinLon; N_rad * sinLat];

            % Calculate ECEF coordinates from NED offset
            ECEF = [-sinLat*cosLon*N - sinLon*E - cosLat*cosLon*D;
                -sinLat*sinLon*N + cosLon*E - cosLat*sinLon*D;
                cosLat*N - sinLat*D] + refECEF;

            % Convert ECEF back to geodetic coordinates
            p = sqrt(ECEF(1)^2 + ECEF(2)^2);
            latECEF = atan2(ECEF(3), p);
            lonECEF = atan2(ECEF(2), ECEF(1));
            hECEF = sqrt(ECEF(1)^2 + ECEF(2)^2 + ECEF(3)^2) - R;

            % Return geodetic coordinates (latitude, longitude, height)
            LLH(:,i) = [latECEF; lonECEF; hECEF];

        end
    end
end