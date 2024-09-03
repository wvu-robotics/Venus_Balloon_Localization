function interpolatedValues = GetBarometryData(LLHs, barometryData)

    % Constants for the vector
    Columns = size(barometryData, 2);
    hRange = 30000; % Height range from 45000m to 75000m

    % Initialize output vector
    interpolatedValues = zeros(1,size(LLHs,2));

    % Loop over each set of LLH coordinates
    for i = 1:size(LLHs, 2)

        % Extract the current height 
        H = LLHs(3,i);

        % If entry is not a number
        if any([anynan(H),~isreal(H),~allfinite(H)])
     
            % Create an arbitrary output
            interpolatedValues(i) = inf;

        % If entry is a number
        else
            
            % Calculate the size of the grid
            hCellSize = hRange / (Columns - 1);
    
            % Clip H to the data range
            H = ((H - 45000) / hCellSize) + 1;
            
            % Calculate the indices surrounding H
            H1 = floor(H);
            H2 = ceil(H);
    
            % Ensure indices are within the bounds of the dataArray
            H1 = max(min(H1,Columns), 1);
            H2 = max(min(H2,Columns), 1);
            
            % Calculate the fractional part of x
            dH = H - H1;
            
            % Handle the case where x is exactly at the data point
            if dH == 0
                interpolatedValues(i) = barometryData(H1);
            else
                % Linear interpolation
                q1 = barometryData(H1);
                q2 = barometryData(H2);
                interpolatedValues(i) = q1 * (1 - dH) + q2 * dH;
            end
        end
    end
end
