function [TIME1, TIME2, TIME3, TIME4, TIME5] = move_motor(position)

speed = 0.2; %in m/s

%placeholder
hori_dist = 0; %in m
vert_dist = 0; %in m
if position == 1 || position == 2
    vert_dist = 0.25;
    if position == 1
        hori_dist = 0.625;
    else 
        hori_dist = 0.375;
    end
elseif position == 3 || position == 4
    vert_dist = 0.5;
    if position == 3
        hori_dist = 0.625;
    else 
        hori_dist = 0.375;
    end
end

%find the time for vertical column to go down
TIME1 = ceil(1000*(vert_dist - 0.03)/speed); %in ms
%find the time for column to go left
TIME2 = ceil(1000*hori_dist/speed);
%find the time for column to drop the tray into location
TIME3 = ceil(1000*0.06/speed); %in ms
%find the time for column to go to the left
TIME4 = ceil(1000*hori_dist/speed);
%find the time for column to go back to initial position
TIME5 = ceil(1000*(vert_dist + 0.03)/speed);

end