clear all;
close all;
clc;

num = 3;
dt = 0.01;
T = 5;
count = 0;
freq = 10;

Q = [0.25 0.5 0.25;
      0.7 0.1  0.2;
      0.2 0.4  0.4];
xStor = zeros(num,(T-1)/dt);
temp = @(t) 0.5 * t;

x = [30; 
     40;
     50];

m = mean(x)

for i = 1:dt:T

    count = count + 1;

    xStor(:,count) = x;

    if mod(count, freq) == 0

        x(1) = x(1) + 5;
    end

    x = Q * x;

end

figure 
hold on 
for k = 1:num

    plot(xStor(k,:));


end
