% x = 0:0.1:10;
% y = gaussmf(x,[2 5]);
% plot(x,y)
% x1 = [-3:.1:3];
% y1 = normpdf(x1,0,1);
% x2 = [-3:.1:3];
% y2 = normpdf(x2,0,0.5);
% plot(x1,x2)
clear all;
close all;
sample = 1000;
x_mean = 1;
x_sigma = 5;
y_sigma = 0.5;
y_mean = 1;
mu = [x_mean y_mean];
sigma = [x_sigma 0; 0 y_sigma];
R = chol(sigma);
z = repmat(mu,sample,1) + randn(sample,2)*R;

scatter(z(:,1), z(:,2));

hold;

a=x_sigma*2; % horizontal radius
b=y_sigma*2; % vertical radius
x0=x_mean; % x0,y0 ellipse centre coordinates
y0=y_mean;
t=-pi:0.01:pi;
x=x0+a*cos(t);
y=y0+b*sin(t);
plot(x,y)
xrange = x_mean+ x_sigma + 3;
yrange = y_mean+ y_sigma + 3;
xlim([-xrange, xrange]);
ylim([-yrange, yrange]);

correct_dir = -[x_mean, y_mean, 0];
r = correct_dir/norm(correct_dir);
angle=acosd(dot(r,[0, 1, 0]));
u=cross([0,1, 0],r);
u=u/norm(u);
axang = [u, angle/180*pi];
rotm = axang2rotm(axang);
z = [z zeros(1000,1)];
z = (rotm * z')';

figure;
scatter(z(:,1), z(:,2));
