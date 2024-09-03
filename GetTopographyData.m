function interpolatedValues = GetTopographyData(LLHs, topographyData)
    
    % Constants for the grid
    Rows = size(topographyData, 1);
    Cols = size(topographyData, 2);
    latRange = pi; % Latitude range from -pi/2 to pi/2
    lonRange = 2 * pi; % Longitude range from -pi to pi
    
    % Initialize output vector
    interpolatedValues = zeros(1, size(LLHs, 2));

     % Loop over each set of LLH coordinates
    for i = 1:size(LLHs, 2)

        % Extract the current LLH coordinates
        LLH = LLHs(:,i);

        % If entry is not a number
        if any([anynan(LLH),~isreal(LLH),~allfinite(LLH)])
    
            % Create an arbitrary output
            interpolatedValues(i) = inf;

        % If entry is a number
        else
    
            % Constrain latitude and longitude
            LLH(1) = max(min(LLH(1), pi/2), -pi/2);  % Constrain latitude to [-pi/2, pi/2]
            LLH(2) = mod(LLH(2) + pi, 2 * pi) - pi;  % Wrap longitude to [-pi, pi]
    
            % Calculate the size of each grid cell
            latCellSize = latRange / (Rows - 1);
            lonCellSize = lonRange / (Cols - 1);
    
            % Calculate the exact floating-point indices for latitude and longitude
            xf = (LLH(1) + pi / 2) / latCellSize;
            yf = (LLH(2) + pi) / lonCellSize;
    
            % Define a tolerance for floating-point comparison
            tolerance = 1e-10;
    
            % Adjust xf and yf to account for rounding errors
            dx = xf - floor(xf);
            dy = yf - floor(yf);
    
            % Check if dx and dy are within tolerance of being an integer
            isExactX = abs(dx) < tolerance || abs(dx - 1) < tolerance;
            isExactY = abs(dy) < tolerance || abs(dy - 1) < tolerance;
    
            % Adjust indices for MATLAB's 1-based indexing
            x1 = floor(xf)+1;
            y1 = floor(yf)+1;
            x2 = min(x1+1, Rows);
            y2 = min(y1+1, Cols);
    
            % Case 1: Both dx and dy are integers (or close within tolerance)
            if isExactX && isExactY
                interpolatedValues(i) = topographyData(round(xf)+1, round(yf)+1);
    
                % Case 2: dx is an integer (or close within tolerance), but dy is not
            elseif isExactX
                q1 = topographyData(round(xf)+1, y1);
                q2 = topographyData(round(xf)+1, y2);
                interpolatedValues(i) = q1 * (1 - dy) + q2 * dy;
    
                % Case 3: dy is an integer (or close within tolerance), but dx is not
            elseif isExactY
                q1 = topographyData(x1, round(yf)+1);
                q2 = topographyData(x2, round(yf)+1);
                interpolatedValues(i) = q1 * (1 - dx) + q2 * dx;
    
                % Case 4: Neither dx nor dy is close to an integer
            else
                % Fetch the four surrounding points for bilinear interpolation
                q11 = topographyData(x1, y1); % Top-left
                q21 = topographyData(x2, y1); % Top-right
                q12 = topographyData(x1, y2); % Bottom-left
                q22 = topographyData(x2, y2); % Bottom-right
    
                % Perform bilinear interpolation
                interpolatedValues(i) = q11 * (1 - dx) * (1 - dy) + ...
                    q21 * dx * (1 - dy) + ...
                    q12 * (1 - dx) * dy + ...
                    q22 * dx * dy;
            end
        end
    end