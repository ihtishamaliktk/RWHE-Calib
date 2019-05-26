function [X,Y,time]=HandeyeShah(AA,BB)
% Solves the problem AX=YB
% using the formulation of
%
% Simultaneous Robot/World and Tool/Flange 
% Calibration by Solving Homogeneous Transformation 
% Equations of the form AX=YB
% M. Shah
%
% Mili Shah
% July 2014
tic
[m,n]=size(AA); n = n/4;
A = zeros(9*n,18);
T = zeros(9,9);
b = zeros(9*n,1);
for i = 1:n
    Ra = AA(1:3,4*i-3:4*i-1);
    Rb = BB(1:3,4*i-3:4*i-1);
    T = T + kron(Rb,Ra);
end
[u,s,v]=svd(T);
x = v(:,1);
y = u(:,1);
X = reshape(x,3,3);
X = sign(det(X))/abs(det(X))^(1/3)*X;
[u,s,v]=svd(X); X = u*v';
Y = reshape(y,3,3);
Y = sign(det(Y))/abs(det(Y))^(1/3)*Y;
[u,s,v]=svd(Y); Y = u*v';
A = zeros(3*n,6);
b = zeros(3*n,1);
for i = 1:n
    A(3*i-2:3*i,:) = [-AA(1:3,4*i-3:4*i-1) eye(3)];
    b(3*i-2:3*i,:) = AA(1:3,4*i) - kron(BB(1:3,4*i)',eye(3))*reshape(Y,9,1);
end
t = A\b;

X = [X t(1:3);[0 0 0 1]];
Y = [Y t(4:6);[0 0 0 1]];
time=toc;