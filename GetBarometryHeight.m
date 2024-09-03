function estimatedHeights = GetBarometryHeight(pressures, barometryData)

    % Constants for the function
    Columns = numel(barometryData);   % Number of pressure data points
    hRange = 30000;                   % Height range from 45000m to 75000m
    hCellSize = hRange / (Columns - 1);
    minHeight = 45000;                % Starting height

    % Initialize output vector
    estimatedHeights = zeros(1, numel(pressures));

    % Loop over each pressure value
    for i = 1:numel(pressures)

        % Extract the current pressure
        P = pressures(i);

        % If pressure is infinite
        if isinf(P)

            % Create an arbitrary output
            estimatedHeights(i) = inf;

        % Check the lower bound
        elseif P > barometryData(1)
            estimatedHeights(i) = minHeight;

        % Check the upper bound
        elseif P < barometryData(end)
            estimatedHeights(i) = minHeight + hRange;
        
        else
            
            % Find the two surrounding data points in barometryData
            for j = 1:(Columns - 1)
                if P >= barometryData(j+1) && P <= barometryData(j)
                    
                    % Linear interpolation to find the height
                    P1 = barometryData(j);
                    P2 = barometryData(j+1);
                    H1 = minHeight + (j - 1) * hCellSize;
                    H2 = H1 + hCellSize;

                    % Linearly interpolate
                    estimatedHeights(i) = H1 + (P - P1) * (H2 - H1) / (P2 - P1);
                    
                end
            end
        end
    end
end