function x = leastqr(A, b)

if issparse(A)
    R = qr(A); 
else
    R = triu(qr(A));
end
x = R\(R'\(A'*b));
r = b - A*x;
e = R\(R'\(A'*r));
x = x + e;

end