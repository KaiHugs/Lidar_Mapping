
% Modify the following line with your own serial port details
% Currently set COM4 as serial port at 115.2kbps 8N1
% Refer to MATLAB Serial API for other options.

ports = serialportlist("available");
s = serialport("COM4", 115200, "Timeout", 135);

% Display the port name
disp("Opening1: " + s.Port);

% Reset input and output buffers
flush(s);

% Wait for user input to start communication
input("Press Enter to start communication...", 's');

% Send the character 's' to MCU via UART
write(s, 's', "char");

while true
    data = strtrim(readline(s));  % Read and trim whitespace/newline
    if data == "0"  % Check if the received value is exactly "0"
        break;  % Exit loop when "0" is detected
    end
    disp(data);
end

fileW = fopen("model.xyz", "w");

% Initialize counter
i = 0;
while i < 100000000
    x = readline(s);
    if strtrim(x) == "999"
        break;
    end
    fprintf(fileW, "%s\n", x);  % Write to file
    disp(x);  % Display received string
    i = i + 1;
end

% Close the file
fclose(fileW);

file = fopen('model.xyz', 'r'); % Open the file for reading
data = fscanf(file, '%f %f %f', [3, Inf]); % Read the data into a 3xN matrix
fclose(file); % Close the file


% Transpose the data to get columns as vectors
data = data';


x = data(:,1); % First column as x
y = data(:,2); % Second column as y
z = data(:,3); % Third column as z


% Plotting
figure;
hold on;

% Insert NaN values every 64 points to break connections
num_points = size(data, 1);
for i = 1:64:num_points
    end_idx = min(i+63, num_points);
    plot3(x(i:end_idx), y(i:end_idx), z(i:end_idx), 'b-', 'Marker', 'o', 'MarkerFaceColor', 'black', 'MarkerSize', 4, 'LineWidth', 10);
end


xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Plot of Box Model');
grid on;
hold off;
% Close the serial port
disp("Closing: " + s.Port);
clear s;  % Clears the serial object from memory