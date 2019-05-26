function out=svdfunc(M)
[U,W,V]=svd(M);
out=U*V';