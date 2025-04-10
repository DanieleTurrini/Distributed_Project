clear all;
close all;
clc;

num = 3;
dt = 0.01;
T = 5;
count = 0;
freq = 50;
LastMeas = ones(num,1);

turn = 3;

Q = [0.8 0.1 0.1;
      0.8 0.15 0.05;
      0.9 0.04  0.06];
xStor = zeros(num,(T-1)/dt);
temp = @(t) 0.5 * t;

x = [30; 
     40;
     50];

for i = 1:dt:T

    count = count + 1;

    xStor(:,count) = x;

    if mod(count, freq) == 0

        if turn == 1

            LastMeas(1) = 1;

            x(1) = x(1) + 5;

            for k = 1:num
                Q(k,2) = 1/LastMeas(2) + rand(1,1) * 0.3;
                Q(k,3) = 1/LastMeas(3) + rand(1,1) * 0.3;
                Q(k,1) = 1 - Q(k,2) - Q(k,3);
            end

        elseif turn == 2

            LastMeas(2) = 1;

            x(2) = x(2) + 5;

            for k = 1:num
                Q(k,1) = 1/LastMeas(1) + rand(1,1) * 0.3;
                Q(k,3) = 1/LastMeas(3) + rand(1,1) * 0.3;
                Q(k,2) = 1 - Q(k,1) - Q(k,3);
            end

        elseif turn == 3

            LastMeas(3) = 1;

            x(3) = x(3) + 5;

            for k = 1:num
                Q(k,2) = 1/LastMeas(2) + rand(1,1) * 0.3;
                Q(k,1) = 1/LastMeas(1) + rand(1,1) * 0.3;
                Q(k,3) = 1 - Q(k,2) - Q(k,1);
            end

        end

        turn = turn + 1;

        if turn == 4

            turn = 1;

        end

    end

    x = Q * x;

    disp(Q);

    LastMeas = LastMeas + 1;

end

figure 
hold on 
for k = 1:num

    plot(xStor(k,:));

end
