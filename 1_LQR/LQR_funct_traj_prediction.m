function [x,u, K] = LQR_funct(A, B, C, Q, R, N, x0)
K=[];
H=[];
H(:,:,N+1) = Q;
K(:,:,N+1) = -(R + B'*H(:,:,N+1)*B)\(B'*H(:,:,N+1)*A);

x([1 2],1)=x0;

for k = N:-1:1
    K(:,:,k) = -(R + B'*H(:,:,k+1)*B)\(B'*H(:,:,k+1)*A);
    H(:,:,k) = Q + K(:,:,k)'*R*K(:,:,k) + (A + B*K(:,:,k))'*H(:,:,k+1)*(A +B*K(:,:,k));
end

u(1) = K(:,:,1)*x(:,1);

for k=2:N
    x(:,k)= A*x(:,k-1)+B*u(k-1);
    u(k) = K(:,:,k)*x(:,k);
end
end