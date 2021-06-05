%%ECE183DB Storage Sub-System Simulation
in_s = [0 1];

%%this is a general framework of the simulation. It will need to be
%%contained in a repetitive loop or in some manner if we have a defined set
%%of input
% a = [0 0];
% s = in_s;

%%Create arrays of inputs placeholder
num = 100;
a = zeros(num,2);
s = a;
s(1,:) = in_s;
a(5,1)=1; a(18,1)=1; a(34,1)=1; a(44,1)=1; a(56,1)=1;a(76,1)=1;
a(65,2)=1;

for k = 1:1:num-1
    if a(k,2)==1 %we received input from user that the storage is cleared
        s(k+1,1)=0;
        s(k+1,2)=1;
    elseif (a(k,1)==0 || s(k,2)==0)
        %do nothing
        s(k+1,:) = s(k,:);
    elseif (a(k,1)==1)    
        %move_motor(s(1)+1);
        s(k+1,1) = s(k,1)+1;
        s(k+1,2) = s(k,2);
        if s(k+1,1)==4 %the storage is full
            s(k+1,2)=0;
        end
    end 
end

figure(1)
plot(1:1:num,a(:,1),1:1:num,a(:,2))
title('Input of Storage System')
xlabel('iter (n)');
legend('a0','a1')

figure(2)
plot(1:1:num,s(:,1),1:1:num,s(:,2))
title('State of Storage System')
xlabel('iter (n)');
legend('s0','s1')
